#include "FlowMotionDetector.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace tracking {

namespace {
constexpr double kRefFps = 30.0; // flow thresholds are specified per 1/30 s

float median(std::vector<float> & v){
	if(v.empty()) return 0.0f;
	const size_t mid = v.size() / 2;
	std::nth_element(v.begin(), v.begin() + mid, v.end());
	return v[mid];
}
} // namespace

FlowMotionDetector::FlowMotionDetector(const FlowDetectorConfig & config) : cfg(config){
	rebuild();
}

void FlowMotionDetector::rebuild(){
	int preset = cv::DISOpticalFlow::PRESET_FAST;
	if(cfg.preset <= 0) preset = cv::DISOpticalFlow::PRESET_ULTRAFAST;
	else if(cfg.preset >= 2) preset = cv::DISOpticalFlow::PRESET_MEDIUM;
	dis = cv::DISOpticalFlow::create(preset);
	mog2 = cv::createBackgroundSubtractorMOG2(cfg.mogHistory, cfg.mogVarThreshold, true);
	const int openPx = std::max(1, cfg.morphOpenPx);
	openKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(openPx, openPx));
	prev.release();
	flowEma.release();
	busy.release();
}

void FlowMotionDetector::setConfig(const FlowDetectorConfig & config){
	const bool structural = config.preset != cfg.preset
		|| config.mogHistory != cfg.mogHistory
		|| config.mogVarThreshold != cfg.mogVarThreshold
		|| config.morphOpenPx != cfg.morphOpenPx;
	cfg = config;
	if(structural) rebuild();
}

void FlowMotionDetector::reset(){
	rebuild();
}

std::vector<Detection> FlowMotionDetector::detect(const cv::Mat & lane, double dtSec){
	std::vector<Detection> out;
	if(lane.empty() || lane.type() != CV_8UC1) return out;

	const int W = lane.cols, H = lane.rows;
	if(prev.empty() || prev.size() != lane.size()){
		prev = lane.clone();
		flowEma = cv::Mat::zeros(H, W, CV_32FC2);
		busy = cv::Mat::zeros(H, W, CV_32F);
		mask = cv::Mat::zeros(H, W, CV_8U);
		fgRaw = cv::Mat::zeros(H, W, CV_8U);
		if(cfg.useMog2) mog2->apply(lane, fg);
		return out;
	}

	// 1. dense flow prev -> lane, in px/frame; normalize to px per 1/30 s so
	//    thresholds do not depend on the source frame rate / dropped frames.
	dis->calc(prev, lane, flow);
	lane.copyTo(prev);
	const double dtFrames = std::clamp(dtSec * kRefFps, 0.25, 6.0);
	const float rateScale = static_cast<float>(1.0 / dtFrames);
	cv::Mat flowNorm = flow * rateScale;

	// 2. temporal smoothing: coherent motion survives, zero-mean ripple cancels.
	const float a = std::clamp(cfg.flowEma, 0.05f, 1.0f);
	cv::addWeighted(flowEma, 1.0f - a, flowNorm, a, 0.0, flowEma);

	std::vector<cv::Mat> ch(2), chInst(2);
	cv::split(flowEma, ch);
	cv::split(flowNorm, chInst);
	cv::Mat mag, instMag;
	cv::magnitude(ch[0], ch[1], mag);
	cv::magnitude(chInst[0], chInst[1], instMag);

	const float thr = cfg.minFlowPx;
	cv::Mat moving = mag > thr;

	// 3. busy map: pixels that flicker (instantaneous motion without
	//    sustained direction) are water sparkle / foliage — suppress them.
	cv::Mat sparkle = (instMag > thr) & (mag < thr * cfg.sparkleRatio);
	cv::Mat sparkleF;
	sparkle.convertTo(sparkleF, CV_32F, 1.0 / 255.0);
	cv::addWeighted(busy, 1.0f - cfg.busyAlpha, sparkleF, cfg.busyAlpha, 0.0, busy);
	cv::Mat busyMask = busy > cfg.busyThresh;
	moving &= ~busyMask;

	// 4. MOG2 confirmation: flow alone also fires on JPEG breathing along
	//    strong edges; real objects are foreground as well.
	if(cfg.useMog2){
		mog2->apply(lane, fg);
		cv::threshold(fg, fg, 200, 255, cv::THRESH_BINARY);
		fg.copyTo(fgRaw); // silhouette source, before the dilation
		if(cfg.mogDilatePx > 1){
			cv::dilate(fg, fg, cv::getStructuringElement(cv::MORPH_ELLIPSE,
				cv::Size(cfg.mogDilatePx, cfg.mogDilatePx)));
		}
		moving &= fg;
		// flow-seeded fill (see FlowDetectorConfig::fillWFrac)
		const int fillW = static_cast<int>(std::lround(cfg.fillWFrac * W));
		const int fillH = static_cast<int>(std::lround(cfg.fillHFrac * H));
		if(fillW > 0 || fillH > 0){
			moving.copyTo(seedMask); // velocity is measured on these pixels only
			// constrained reconstruction: grow the seed one step at a time and
			// keep only foreground, so the fill spreads through foreground that
			// is *connected* to the moving blob and cannot jump across static
			// ground to an unrelated foreground patch within reach
			const int steps = std::max(fillW, fillH);
			const int kw = 1 + 2 * (fillW > 0 ? (fillW + steps - 1) / steps : 0);
			const int kh = 1 + 2 * (fillH > 0 ? (fillH + steps - 1) / steps : 0);
			const cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kw, kh));
			cv::Mat grow = moving.clone(), tmp;
			for(int i = 0; i < steps; i++){
				cv::dilate(grow, tmp, k);
				tmp &= fg;
				if(cv::countNonZero(tmp != grow) == 0) break;
				grow = tmp;
			}
			moving |= grow;
		}else{
			seedMask.release();
		}
	}else{
		seedMask.release();
	}

	// 5. cleanup: kill speckle, then bridge gaps along the lane axis.
	cv::morphologyEx(moving, moving, cv::MORPH_OPEN, openKernel);
	const int closeW = std::max(1, static_cast<int>(std::lround(cfg.closeWFrac * W)));
	const int closeH = std::max(1, static_cast<int>(std::lround(cfg.closeHFrac * H)));
	if(closeW > 1 || closeH > 1){
		cv::morphologyEx(moving, moving, cv::MORPH_CLOSE,
		                 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closeW, closeH)));
	}
	mask = moving;

	// 6. blobs with their median flow vector.
	cv::Mat labels, stats, centroids;
	const int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
	const float laneArea = static_cast<float>(W) * H;
	const float minArea = cfg.minAreaFrac * laneArea;
	const float minW = cfg.minWidthFrac * W;
	const float minH = cfg.minHeightFrac * H;
	// px per 1/30 s -> lane fractions per second
	const float toVx = static_cast<float>(kRefFps) / W;
	const float toVy = static_cast<float>(kRefFps) / H;

	std::vector<float> vxs, vys, mags;
	std::vector<Detection> blobs;
	for(int l = 1; l < n; l++){
		const int area = stats.at<int>(l, cv::CC_STAT_AREA);
		if(area < minArea) continue;
		const int bx = stats.at<int>(l, cv::CC_STAT_LEFT);
		const int by = stats.at<int>(l, cv::CC_STAT_TOP);
		const int bw = stats.at<int>(l, cv::CC_STAT_WIDTH);
		const int bh = stats.at<int>(l, cv::CC_STAT_HEIGHT);
		if(bw < minW || bh < minH) continue;

		// median flow over the blob — over its flow-seed pixels only when the
		// fill is on (filled pixels have no usable flow by definition)
		const bool useSeed = !seedMask.empty();
		for(int pass = 0; pass < 2; pass++){
			vxs.clear(); vys.clear(); mags.clear();
			vxs.reserve(area); vys.reserve(area); mags.reserve(area);
			const bool onlySeed = useSeed && pass == 0;
			for(int y = by; y < by + bh; y++){
				const int * lrow = labels.ptr<int>(y);
				const cv::Vec2f * frow = flowEma.ptr<cv::Vec2f>(y);
				const uint8_t * srow = onlySeed ? seedMask.ptr<uint8_t>(y) : nullptr;
				for(int x = bx; x < bx + bw; x++){
					if(lrow[x] != l) continue;
					if(srow && !srow[x]) continue;
					vxs.push_back(frow[x][0]);
					vys.push_back(frow[x][1]);
					mags.push_back(std::hypot(frow[x][0], frow[x][1]));
				}
			}
			if(!vxs.empty() || !onlySeed) break;
		}
		if(vxs.empty()) continue;
		float mvx = median(vxs);
		float mvy = median(vys);
		float meanMag = 0;
		for(const float m : mags) meanMag += m;
		meanMag /= mags.size();
		const float coh = std::hypot(mvx, mvy) / std::max(meanMag, 1e-6f);
		if(coh < cfg.minCoherence) continue;
		if(cfg.lockAxis) mvy = 0.0f;

		Detection d;
		d.x0 = bx / static_cast<float>(W);
		d.x1 = (bx + bw) / static_cast<float>(W);
		d.y0 = by / static_cast<float>(H);
		d.y1 = (by + bh) / static_cast<float>(H);
		d.vx = mvx * toVx;
		d.vy = mvy * toVy;
		const float sp = d.speed();
		if(sp < cfg.minSpeed) continue;
		if(sp > cfg.maxSpeed){
			d.vx *= cfg.maxSpeed / sp;
			d.vy *= cfg.maxSpeed / sp;
		}
		d.hasVelocity = true;
		d.coherence = std::min(1.0f, coh);
		d.area = area / laneArea;
		d.touchesLeft = bx <= cfg.edgeMarginPx;
		d.touchesRight = bx + bw >= W - cfg.edgeMarginPx;
		blobs.push_back(std::move(d));
	}

	// 7. blobs moving together are one object.
	GroupingParams gp;
	gp.gapFrac = cfg.mergeGapFrac;
	gp.angleDeg = cfg.mergeAngleDeg;
	gp.speedRatio = cfg.mergeSpeedRatio;
	gp.lateralOverlap = cfg.mergeLateralOverlap;
	gp.minSpeed = cfg.minSpeed;
	out = groupDetections(blobs, gp);
	if(static_cast<int>(out.size()) > cfg.maxDetections) out.resize(cfg.maxDetections);
	return out;
}

} // namespace tracking
