#include "LensCorrector.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "ofLog.h"

namespace {

float clampf(float v, float lo, float hi){ return std::min(hi, std::max(lo, v)); }

struct Proj {
	cv::Matx33d K;
	cv::Vec4d D;
	cv::Matx33d P;
	int w = 0;
	int h = 0;
};

cv::Matx33d sourceK(const LensCorrection & p, int W, int H){
	const double fov = static_cast<double>(clampf(p.fovDeg, 40.0f, 175.0f)) * CV_PI / 180.0;
	const double f = static_cast<double>(W) / fov;
	const double cx = clampf(p.centerX, 0.05f, 0.95f) * W;
	const double cy = clampf(p.centerY, 0.05f, 0.95f) * H;
	return cv::Matx33d(f, 0, cx, 0, f, cy, 0, 0, 1);
}

cv::Vec4d sourceD(const LensCorrection & p){
	return cv::Vec4d(p.k1, p.k2, p.k3, p.k4);
}

// Largest filled rectangle: the center row's full width and the center
// column's full height. Corner rays flare past this and are left out, so
// the rectangle stays inside the sensor. frameFovDeg narrows the width.
bool crossProjection(const LensCorrection & p, int W, int H, Proj & out){
	out.K = sourceK(p, W, H);
	out.D = sourceD(p);
	std::vector<cv::Point2d> src{
		{W * 0.5, 0.0}, {W * 0.5, H - 1.0}, {0.0, H * 0.5}, {W - 1.0, H * 0.5}};
	std::vector<cv::Point2d> dst;
	cv::fisheye::undistortPoints(src, dst, out.K, out.D);
	if(dst.size() != 4) return false;
	for(const cv::Point2d & q : dst){
		if(!std::isfinite(q.x) || !std::isfinite(q.y)) return false;
	}
	double nx0 = dst[2].x, nx1 = dst[3].x, ny0 = dst[0].y, ny1 = dst[1].y;
	if(p.frameFovDeg > 1.0f){
		const double half = static_cast<double>(clampf(p.frameFovDeg, 20.0f, 150.0f)) * 0.5 * CV_PI / 180.0;
		const double span = std::tan(half);
		nx0 = std::max(nx0, -span);
		nx1 = std::min(nx1, span);
	}
	if(!(nx1 > nx0 + 1e-3) || !(ny1 > ny0 + 1e-3)) return false;
	if(nx1 - nx0 > 6.0 || ny1 - ny0 > 6.0) return false; // a folded ray, not a lens
	const double f = out.K(0, 0);
	out.w = std::max(16, static_cast<int>(std::lround(f * (nx1 - nx0))));
	out.h = std::max(16, static_cast<int>(std::lround(f * (ny1 - ny0))));
	out.P = cv::Matx33d(f, 0, -nx0 * f, 0, f, -ny0 * f, 0, 0, 1);
	return true;
}

// The previous filled frame: source aspect, balance 0. Used only to move
// lane anchors that were saved in that crop.
bool balanceProjection(const LensCorrection & p, int W, int H, Proj & out){
	out.K = sourceK(p, W, H);
	out.D = sourceD(p);
	cv::Mat P;
	cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
		out.K, out.D, cv::Size(W, H), cv::Mat::eye(3, 3, CV_64F), P, 0.0, cv::Size(W, H), 1.0);
	if(P.empty()) return false;
	P.convertTo(P, CV_64F);
	out.P = cv::Matx33d(P.at<double>(0, 0), P.at<double>(0, 1), P.at<double>(0, 2),
	                    P.at<double>(1, 0), P.at<double>(1, 1), P.at<double>(1, 2),
	                    P.at<double>(2, 0), P.at<double>(2, 1), P.at<double>(2, 2));
	out.w = W;
	out.h = H;
	return out.P(0, 0) > 1.0;
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
	p.frameFovDeg = std::max(0.0f, p.frameFovDeg);
	p.centerX = clampf(p.centerX, 0.05f, 0.95f);
	p.centerY = clampf(p.centerY, 0.05f, 0.95f);
	if(builtW == paneW && builtH == paneH && built == p && !map1.empty()) return;

	Proj proj;
	bool ok = false;
	try {
		ok = crossProjection(p, paneW, paneH, proj);
	} catch(const cv::Exception & e){
		ofLogWarning("LensCorrector") << "frame failed, source aspect kept: " << e.what();
	}
	if(!ok){
		try {
			ok = balanceProjection(p, paneW, paneH, proj);
		} catch(const cv::Exception & e){
			ofLogWarning("LensCorrector") << "fallback frame failed: " << e.what();
		}
	}
	if(!ok){
		map1.release();
		map2.release();
		builtW = 0;
		return;
	}

	const cv::Mat eye = cv::Mat::eye(3, 3, CV_64F);
	cv::fisheye::initUndistortRectifyMap(proj.K, proj.D, eye, cv::Mat(proj.P),
		cv::Size(proj.w, proj.h), CV_16SC2, map1, map2);
	built = p;
	builtW = paneW;
	builtH = paneH;
	builtGeom.panes = 1;
	builtGeom.outPaneW = proj.w;
	builtGeom.outPaneH = proj.h;
	builtGeom.innerX = 0;
	builtGeom.innerY = 0;
	builtGeom.innerW = proj.w;
	builtGeom.innerH = proj.h;
	ofLogNotice("LensCorrector")
		<< "filled frame " << proj.w << "x" << proj.h
		<< " from " << paneW << "x" << paneH
		<< " (fov " << p.fovDeg << " deg)";
}

bool migrateBalanceUv(const LensCorrection & lens, float & u, float & v){
	// 16:9, matching the camera. Both projections scale with the pane, so
	// the UV result does not depend on this size.
	const int W = 1920, H = 1080;
	Proj oldF, neu;
	try {
		if(!balanceProjection(lens, W, H, oldF) || !crossProjection(lens, W, H, neu)) return false;
		std::vector<cv::Point2d> pix{{static_cast<double>(u) * W, static_cast<double>(v) * H}};
		std::vector<cv::Point2d> src, dst;
		cv::fisheye::distortPoints(pix, src, cv::Mat(oldF.P), oldF.K, oldF.D);
		cv::fisheye::undistortPoints(src, dst, neu.K, neu.D, cv::Mat::eye(3, 3, CV_64F), cv::Mat(neu.P));
		if(dst.size() != 1 || !std::isfinite(dst[0].x) || !std::isfinite(dst[0].y)) return false;
		u = static_cast<float>(dst[0].x / neu.w);
		v = static_cast<float>(dst[0].y / neu.h);
		return true;
	} catch(const cv::Exception & e){
		ofLogWarning("LensCorrector") << "lane migration failed: " << e.what();
		return false;
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
