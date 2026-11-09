#include "LensCorrector.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "ofLog.h"

namespace {

float clampf(float v, float lo, float hi){ return std::min(hi, std::max(lo, v)); }

// Where the source-image border lands in the filled frame's pixel space.
// Points that fail to project (folded rays) are ignored. The box always
// contains the filled frame itself, and it is capped at one extra frame
// each side so a bad sample cannot allocate a huge map.
void recoveredBounds(const cv::Matx33d & K, const cv::Vec4d & D, const cv::Mat & P,
                     int paneW, int paneH, int & x0, int & y0, int & x1, int & y1){
	x0 = 0;
	y0 = 0;
	x1 = paneW;
	y1 = paneH;
	const int n = 64;
	std::vector<cv::Point2d> src;
	src.reserve(static_cast<size_t>(n) * 4);
	for(int i = 0; i < n; i++){
		const double t = (static_cast<double>(i) + 0.5) / n;
		src.emplace_back(t * paneW, 0.0);
		src.emplace_back(t * paneW, paneH - 1.0);
		src.emplace_back(0.0, t * paneH);
		src.emplace_back(paneW - 1.0, t * paneH);
	}
	std::vector<cv::Point2d> dst;
	const cv::Mat eye = cv::Mat::eye(3, 3, CV_64F);
	cv::fisheye::undistortPoints(src, dst, K, D, eye, P);
	if(dst.size() != src.size()) return;
	double minX = 0, minY = 0, maxX = paneW, maxY = paneH;
	const double capX = paneW, capY = paneH;
	for(const cv::Point2d & p : dst){
		if(!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
		if(p.x < -capX || p.x > paneW + capX || p.y < -capY || p.y > paneH + capY) continue;
		minX = std::min(minX, p.x);
		minY = std::min(minY, p.y);
		maxX = std::max(maxX, p.x);
		maxY = std::max(maxY, p.y);
	}
	x0 = std::max(static_cast<int>(std::floor(minX)), -paneW);
	y0 = std::max(static_cast<int>(std::floor(minY)), -paneH);
	x1 = std::min(static_cast<int>(std::ceil(maxX)), paneW + paneW);
	y1 = std::min(static_cast<int>(std::ceil(maxY)), paneH + paneH);
	if(x1 <= x0 || y1 <= y0){
		x0 = 0;
		y0 = 0;
		x1 = paneW;
		y1 = paneH;
	}
}

// The filled frame must come out of the extended map unchanged, or every
// saved lane would shift. Samples the integer source lookup.
int maxInnerDelta(const cv::Mat & innerMap, const cv::Mat & extMap, int ox, int oy){
	int worst = 0;
	const int stepX = std::max(1, innerMap.cols / 8);
	const int stepY = std::max(1, innerMap.rows / 8);
	for(int y = 0; y < innerMap.rows; y += stepY){
		for(int x = 0; x < innerMap.cols; x += stepX){
			const cv::Vec2s a = innerMap.at<cv::Vec2s>(y, x);
			const cv::Vec2s b = extMap.at<cv::Vec2s>(y + oy, x + ox);
			worst = std::max(worst, std::max(std::abs(a[0] - b[0]), std::abs(a[1] - b[1])));
		}
	}
	return worst;
}

} // namespace

LensGeometry LensCorrector::identity(int panes, int paneW, int paneH){
	LensGeometry g;
	g.panes = std::max(1, panes);
	g.outPaneW = std::max(0, paneW);
	g.outPaneH = std::max(0, paneH);
	g.innerX = 0;
	g.innerY = 0;
	g.innerW = g.outPaneW;
	g.innerH = g.outPaneH;
	return g;
}

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
	P.convertTo(P, CV_64F);

	int x0 = 0, y0 = 0, x1 = paneW, y1 = paneH;
	try {
		recoveredBounds(K, D, P, paneW, paneH, x0, y0, x1, y1);
	} catch(const cv::Exception & e){
		ofLogWarning("LensCorrector") << "border extent failed, filled frame only: " << e.what();
		x0 = 0;
		y0 = 0;
		x1 = paneW;
		y1 = paneH;
	}
	const int marginL = -x0;
	const int marginT = -y0;
	const int outW = x1 - x0;
	const int outH = y1 - y0;

	cv::Mat Pext = P.clone();
	Pext.at<double>(0, 2) += marginL;
	Pext.at<double>(1, 2) += marginT;
	cv::Mat ext1, ext2;
	cv::fisheye::initUndistortRectifyMap(K, D, eye, Pext, cv::Size(outW, outH), CV_16SC2, ext1, ext2);

	// Keep the filled frame byte-for-byte the old correction. A mismatch
	// means the border shift is wrong, so drop the border rather than move
	// every lane.
	if(marginL != 0 || marginT != 0 || outW != paneW || outH != paneH){
		cv::Mat ref1, ref2;
		cv::fisheye::initUndistortRectifyMap(K, D, eye, P, cv::Size(paneW, paneH), CV_16SC2, ref1, ref2);
		const int delta = maxInnerDelta(ref1, ext1, marginL, marginT);
		if(delta > 1){
			ofLogWarning("LensCorrector")
				<< "recovered border mismatched the filled frame by " << delta
				<< "px — keeping the filled frame";
			ext1 = ref1;
			ext2 = ref2;
			x0 = 0;
			y0 = 0;
			x1 = paneW;
			y1 = paneH;
		}
	}

	map1 = ext1;
	map2 = ext2;
	built = p;
	builtW = paneW;
	builtH = paneH;
	builtGeom.panes = 1;
	builtGeom.outPaneW = x1 - x0;
	builtGeom.outPaneH = y1 - y0;
	builtGeom.innerX = -x0;
	builtGeom.innerY = -y0;
	builtGeom.innerW = paneW;
	builtGeom.innerH = paneH;
	if(builtGeom.hasMargin()){
		ofLogNotice("LensCorrector")
			<< "filled frame " << paneW << "x" << paneH
			<< " plus recovered border L" << builtGeom.innerX
			<< " T" << builtGeom.innerY
			<< " R" << (builtGeom.outPaneW - builtGeom.innerX - builtGeom.innerW)
			<< " B" << (builtGeom.outPaneH - builtGeom.innerY - builtGeom.innerH);
	}
}

void LensCorrector::apply(const cv::Mat & src, cv::Mat & dst){
	if(src.empty()){
		dst.release();
		geom = identity(1, 0, 0);
		return;
	}
	const int panes = src.cols >= src.rows * 2 ? 2 : 1;
	const int paneW = src.cols / panes;
	const int paneH = src.rows;
	if(!params.enabled || paneW < 16 || paneH < 16){
		geom = identity(panes, paneW, paneH);
		src.copyTo(dst);
		return;
	}
	try {
		ensureMaps(paneW, paneH);
	} catch(const cv::Exception & e){
		ofLogWarning("LensCorrector") << "map build failed, frame passed through: " << e.what();
		builtW = 0;
		geom = identity(panes, paneW, paneH);
		src.copyTo(dst);
		return;
	}
	if(map1.empty() || builtGeom.outPaneW <= 0 || builtGeom.outPaneH <= 0
	   || map1.cols != builtGeom.outPaneW || map1.rows != builtGeom.outPaneH){
		geom = identity(panes, paneW, paneH);
		src.copyTo(dst);
		return;
	}
	geom = builtGeom;
	geom.panes = panes;
	dst.create(geom.outPaneH, panes * geom.outPaneW, src.type());
	for(int i = 0; i < panes; i++){
		const cv::Rect srcRoi(i * paneW, 0, paneW, paneH);
		const cv::Rect dstRoi(i * geom.outPaneW, 0, geom.outPaneW, geom.outPaneH);
		cv::remap(src(srcRoi), dst(dstRoi), map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	}
}
