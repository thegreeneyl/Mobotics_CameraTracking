#include "LaneWarp.h"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace tracking {

namespace {

cv::Point2f applyH(const cv::Mat & h, float x, float y){
	const double * m = h.ptr<double>();
	const double w = m[6] * x + m[7] * y + m[8];
	if(std::fabs(w) < 1e-12) return {0, 0};
	return {static_cast<float>((m[0] * x + m[1] * y + m[2]) / w),
	        static_cast<float>((m[3] * x + m[4] * y + m[5]) / w)};
}

LaneConfig fullPane(){
	LaneConfig lane;
	lane.quad = FrameQuad{};
	return lane;
}

} // namespace

LaneMap LaneMap::build(const LaneConfig & laneIn, float paneW, float paneH){
	LaneMap map;
	if(paneW < 1.0f) paneW = 1.0f;
	if(paneH < 1.0f) paneH = 1.0f;
	const LaneConfig fallback = fullPane();
	const LaneConfig & lane = laneIn.isWarpable() ? laneIn : fallback;

	std::vector<std::array<float, 2>> top, bottom;
	lane.borders(top, bottom);
	const int n = static_cast<int>(top.size());
	if(n < 2) return map;

	std::vector<float> acc(static_cast<size_t>(n), 0.0f);
	for(int i = 0; i < n - 1; i++){
		const float lt = std::hypot((top[i + 1][0] - top[i][0]) * paneW, (top[i + 1][1] - top[i][1]) * paneH);
		const float lb = std::hypot((bottom[i + 1][0] - bottom[i][0]) * paneW, (bottom[i + 1][1] - bottom[i][1]) * paneH);
		acc[static_cast<size_t>(i + 1)] = acc[static_cast<size_t>(i)] + 0.5f * (lt + lb);
	}
	const float total = acc.back();
	if(total < 1e-3f) return map;
	for(float & a : acc) a /= total;
	acc.front() = 0.0f;
	acc.back() = 1.0f;

	float ribs = 0;
	for(int i = 0; i < n; i++){
		ribs += std::hypot((bottom[i][0] - top[i][0]) * paneW, (bottom[i][1] - top[i][1]) * paneH);
	}
	map.srcW = total;
	map.srcH = ribs / static_cast<float>(n);

	map.segs.resize(static_cast<size_t>(n - 1));
	for(int i = 0; i < n - 1; i++){
		Seg & seg = map.segs[static_cast<size_t>(i)];
		seg.u0 = acc[static_cast<size_t>(i)];
		seg.u1 = acc[static_cast<size_t>(i + 1)];
		const cv::Point2f src[4] = {{seg.u0, 0}, {seg.u1, 0}, {seg.u1, 1}, {seg.u0, 1}};
		const cv::Point2f dst[4] = {
			{top[i][0] * paneW, top[i][1] * paneH},
			{top[i + 1][0] * paneW, top[i + 1][1] * paneH},
			{bottom[i + 1][0] * paneW, bottom[i + 1][1] * paneH},
			{bottom[i][0] * paneW, bottom[i][1] * paneH}};
		seg.unitToPane = cv::getPerspectiveTransform(src, dst);
	}
	map.ok = !map.segs.empty() && !map.segs[0].unitToPane.empty();
	return map;
}

void LaneMap::rectifiedSize(int maxW, int minH, int paneLimitH, int & w, int & h) const {
	const float aspect = std::max(srcW / std::max(srcH, 1e-3f), 0.05f);
	w = std::clamp(static_cast<int>(std::lround(srcW)), 32, std::max(32, maxW));
	h = std::max(minH, static_cast<int>(std::lround(w / aspect)));
	h = std::min(h, std::max(32, paneLimitH));
}

void LaneMap::prepare(int dstW, int dstH){
	laneW = std::max(1, dstW);
	laneH = std::max(1, dstH);
	paneToLane.release();
	if(segs.empty()) return;
	const float W = static_cast<float>(laneW);
	const float H = static_cast<float>(laneH);
	for(size_t i = 0; i < segs.size(); i++){
		Seg & seg = segs[i];
		if(seg.unitToPane.empty()) continue;
		cv::Mat inv;
		if(!cv::invert(seg.unitToPane, inv)) continue;
		cv::Mat scale = cv::Mat::eye(3, 3, CV_64F);
		scale.at<double>(0, 0) = W;
		scale.at<double>(1, 1) = H;
		// pane px -> lane unit -> lane px. For one slice this is the homography
		// that used to be built directly from the four corners.
		seg.paneToLane = scale * inv;
	}
	if(segs.size() == 1) paneToLane = segs[0].paneToLane;
}

void LaneMap::warp(const cv::Mat & pane, cv::Mat & dst, int interpolation) const {
	if(pane.empty() || laneW <= 0 || laneH <= 0) return;
	if(segs.size() == 1 && !paneToLane.empty()){
		cv::warpPerspective(pane, dst, paneToLane, cv::Size(laneW, laneH), interpolation);
		return;
	}
	dst.create(laneH, laneW, pane.type());
	dst.setTo(0);
	for(size_t i = 0; i < segs.size(); i++){
		const Seg & seg = segs[i];
		if(seg.paneToLane.empty()) continue;
		cv::Mat full;
		cv::warpPerspective(pane, full, seg.paneToLane, cv::Size(laneW, laneH), interpolation);
		int x0 = static_cast<int>(std::lround(seg.u0 * laneW));
		int x1 = static_cast<int>(std::lround(seg.u1 * laneW));
		if(i == 0) x0 = 0;
		if(i + 1 == segs.size()) x1 = laneW;
		x0 = std::clamp(x0, 0, laneW);
		x1 = std::clamp(x1, 0, laneW);
		if(x1 <= x0) continue;
		full(cv::Rect(x0, 0, x1 - x0, laneH)).copyTo(dst(cv::Rect(x0, 0, x1 - x0, laneH)));
	}
}

bool LaneMap::toPane(float u, float v, float & x, float & y) const {
	if(segs.empty()) return false;
	u = std::min(1.0f, std::max(0.0f, u));
	v = std::min(1.0f, std::max(0.0f, v));
	size_t s = segs.size() - 1;
	for(size_t i = 0; i < segs.size(); i++){
		if(u <= segs[i].u1 || i + 1 == segs.size()){ s = i; break; }
	}
	if(segs[s].unitToPane.empty()) return false;
	const cv::Point2f p = applyH(segs[s].unitToPane, u, v);
	x = p.x;
	y = p.y;
	return true;
}

cv::Rect2f LaneMap::paneBounds(float u0, float u1, float v0, float v1) const {
	if(u0 > u1) std::swap(u0, u1);
	if(v0 > v1) std::swap(v0, v1);
	std::vector<float> us{u0, u1};
	for(const Seg & seg : segs){
		if(seg.u0 > u0 && seg.u0 < u1) us.push_back(seg.u0);
		if(seg.u1 > u0 && seg.u1 < u1) us.push_back(seg.u1);
	}
	float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
	const float vs[2] = {v0, v1};
	for(float u : us){
		for(float v : vs){
			float x, y;
			if(!toPane(u, v, x, y)) continue;
			minX = std::min(minX, x); maxX = std::max(maxX, x);
			minY = std::min(minY, y); maxY = std::max(maxY, y);
		}
	}
	if(maxX <= minX || maxY <= minY) return {};
	return {minX, minY, maxX - minX, maxY - minY};
}

} // namespace tracking
