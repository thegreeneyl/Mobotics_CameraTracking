#include "LensCorrector.h"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "ofLog.h"

namespace {

float clampf(float v, float lo, float hi){ return std::min(hi, std::max(lo, v)); }

} // namespace

void LensCorrector::ensureMaps(int paneW, int paneH){
	LensCorrection p = params;
	p.fovDeg = clampf(p.fovDeg, 40.0f, 175.0f);
	p.balance = clampf(p.balance, 0.0f, 1.0f);
	p.centerX = clampf(p.centerX, 0.05f, 0.95f);
	p.centerY = clampf(p.centerY, 0.05f, 0.95f);
	if(builtW == paneW && builtH == paneH && built == p && !map1.empty()) return;

	// Equidistant fisheye: a ray at angle theta from the axis lands at
	// r = f * theta. The left and right edges are fov/2 from the axis and
	// paneW/2 pixels from the center, so f = paneW / fov.
	const double fov = static_cast<double>(p.fovDeg) * CV_PI / 180.0;
	const double f = static_cast<double>(paneW) / fov;
	const cv::Matx33d K(f, 0, p.centerX * paneW,
	                    0, f, p.centerY * paneH,
	                    0, 0, 1);
	const cv::Vec4d D(p.k1, p.k2, p.k3, p.k4);
	const cv::Mat eye = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat P;
	cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
		K, D, cv::Size(paneW, paneH), eye, P, p.balance, cv::Size(paneW, paneH), 1.0);
	cv::fisheye::initUndistortRectifyMap(K, D, eye, P, cv::Size(paneW, paneH), CV_16SC2, map1, map2);
	built = p;
	builtW = paneW;
	builtH = paneH;
}

void LensCorrector::apply(const cv::Mat & src, cv::Mat & dst){
	if(src.empty() || !params.enabled){
		src.copyTo(dst);
		return;
	}
	const int panes = src.cols >= src.rows * 2 ? 2 : 1;
	const int paneW = src.cols / panes;
	const int paneH = src.rows;
	if(paneW < 16 || paneH < 16){
		src.copyTo(dst);
		return;
	}
	try {
		ensureMaps(paneW, paneH);
	} catch(const cv::Exception & e){
		ofLogWarning("LensCorrector") << "map build failed, frame passed through: " << e.what();
		builtW = 0;
		src.copyTo(dst);
		return;
	}
	if(map1.empty() || map1.rows != paneH || map1.cols != paneW){
		src.copyTo(dst);
		return;
	}
	dst.create(src.rows, src.cols, src.type());
	for(int i = 0; i < panes; i++){
		const cv::Rect roi(i * paneW, 0, paneW, paneH);
		cv::remap(src(roi), dst(roi), map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	}
	const int used = panes * paneW;
	if(used < src.cols){
		const cv::Rect tail(used, 0, src.cols - used, src.rows);
		src(tail).copyTo(dst(tail));
	}
}
