#include "ShapeEstimator.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "ofLog.h"

namespace tracking {

namespace {
constexpr int kUp = 4; // grid upsampling for the contour (sub-cell outline)
constexpr float kUnobserved = -1.0f;

ObjectShape frameFor(const TrackedObject & o, const ShapeConfig & cfg){
	ObjectShape f;
	f.gridW = std::max(4, cfg.gridW);
	f.gridH = std::max(4, cfg.gridH);
	f.fx0 = o.x0 - cfg.padX * o.w;
	f.fx1 = o.x1 + cfg.padX * o.w;
	f.fy0 = o.y0 - cfg.padY * o.h;
	f.fy1 = o.y1 + cfg.padY * o.h;
	return f;
}

cv::Point2f applyH(const cv::Mat & h, float x, float y){
	const double * m = h.ptr<double>();
	const double w = m[6] * x + m[7] * y + m[8];
	return {static_cast<float>((m[0] * x + m[1] * y + m[2]) / w),
	        static_cast<float>((m[3] * x + m[4] * y + m[5]) / w)};
}

// Lane-normalized rect -> pane px bounding rect through the inverse lane
// homography (lane px -> pane px).
cv::Rect2f laneRectToPane(const cv::Mat & hInv, float lx0, float lx1, float ly0, float ly1, int laneW, int laneH){
	float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
	const float xs[2] = {lx0 * laneW, lx1 * laneW}, ys[2] = {ly0 * laneH, ly1 * laneH};
	for(float x : xs) for(float y : ys){
		const cv::Point2f p = applyH(hInv, x, y);
		minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
		minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
	}
	return {minX, minY, maxX - minX, maxY - minY};
}
} // namespace

void ShapeEstimator::setConfig(const ShapeConfig & config){
	const bool modelChanged = config.modelPath != globalCfg.modelPath
		|| config.modelInputW != globalCfg.modelInputW || config.modelInputH != globalCfg.modelInputH;
	globalCfg = config;
	if(modelChanged){
		model.reset();
		modelFailed = false;
	}
}

void ShapeEstimator::reset(){
	states.clear();
}

void ShapeEstimator::ensureModel(){
	if(model || modelFailed) return;
	model = std::make_unique<SegModel>();
	if(globalCfg.modelPath.empty()
	   || !model->load(globalCfg.modelPath, globalCfg.modelInputW, globalCfg.modelInputH)){
		ofLogWarning("ShapeEstimator")
			<< "segmentation model not available (" << globalCfg.modelPath
			<< ") — shape source falls back to the MOG2 foreground";
		model.reset();
		modelFailed = true;
	}else{
		ofLogNotice("ShapeEstimator") << "segmentation model " << globalCfg.modelPath
			<< " (input " << globalCfg.modelInputW << "x" << globalCfg.modelInputH << ")";
	}
}

bool ShapeEstimator::wantsColor(const ShapeConfig & c) const {
	return c.enabled && c.source == 2 && !modelFailed;
}

void ShapeEstimator::beginFrame(const cv::Mat & bgr, int pw, int ph, uint64_t index){
	frameBgr = bgr;
	paneW = pw;
	paneH = ph;
	frameIndex = index;
	modelRunsThisFrame = 0;
	modelMsThisFrame = 0;
}

void ShapeEstimator::endFrame(){
	for(auto it = states.begin(); it != states.end();){
		if(it->second.lastSeen != frameIndex) it = states.erase(it);
		else ++it;
	}
	frameBgr.release();
}

// Area-averages the lane mask into the grid cells that this lane sees.
// obs: 32F grid, kUnobserved where no lane has looked yet; a cell seen by
// two overlapping lanes keeps the larger occupancy.
void ShapeEstimator::sampleLane(const cv::Mat & laneMask, const LaneInput & li, const ObjectShape & f, cv::Mat & obs,
                                const cv::Mat * seen) const {
	if(laneMask.empty() || laneMask.type() != CV_8UC1) return;
	const int W = laneMask.cols, H = laneMask.rows;
	const LaneConfig & lane = *li.lane;
	const float span = std::fabs(lane.s1 - lane.s0);
	if(span < 1e-6f || f.frameW() <= 0 || f.frameH() <= 0) return;

	cv::Mat bin, integ;
	cv::threshold(laneMask, bin, 0, 1, cv::THRESH_BINARY);
	cv::integral(bin, integ, CV_32S);
	const auto sum = [&](int x0, int y0, int x1, int y1){
		return integ.at<int>(y1, x1) - integ.at<int>(y0, x1) - integ.at<int>(y1, x0) + integ.at<int>(y0, x0);
	};
	cv::Mat seenInteg;
	const bool useSeen = seen && !seen->empty() && seen->size() == laneMask.size() && seen->type() == CV_8UC1;
	if(useSeen){
		cv::Mat seenBin;
		cv::threshold(*seen, seenBin, 0, 1, cv::THRESH_BINARY);
		cv::integral(seenBin, seenInteg, CV_32S);
	}
	const auto seenSum = [&](int x0, int y0, int x1, int y1){
		return seenInteg.at<int>(y1, x1) - seenInteg.at<int>(y0, x1) - seenInteg.at<int>(y1, x0) + seenInteg.at<int>(y0, x0);
	};

	// cell footprint in lane px
	const float cellLaneW = f.frameW() / f.gridW / span * W;
	const float cellLaneH = f.frameH() / f.gridH * H;
	for(int gu = 0; gu < f.gridW; gu++){
		const float s = f.toS(gu + 0.5f);
		const float lx = lane.toLane(s);
		if(lx < 0.0f || lx >= 1.0f) continue;
		const float cx = lx * W;
		const int x0 = std::clamp(static_cast<int>(std::floor(cx - cellLaneW * 0.5f)), 0, W - 1);
		const int x1 = std::clamp(static_cast<int>(std::ceil(cx + cellLaneW * 0.5f)), x0 + 1, W);
		for(int gv = 0; gv < f.gridH; gv++){
			const float ly = f.toY(gv + 0.5f);
			if(ly < 0.0f || ly >= 1.0f) continue;
			const float cy = ly * H;
			const int y0 = std::clamp(static_cast<int>(std::floor(cy - cellLaneH * 0.5f)), 0, H - 1);
			const int y1 = std::clamp(static_cast<int>(std::ceil(cy + cellLaneH * 0.5f)), y0 + 1, H);
			const int cellPx = (x1 - x0) * (y1 - y0);
			if(useSeen && seenSum(x0, y0, x1, y1) * 2 < cellPx) continue; // not looked at
			const float occ = static_cast<float>(sum(x0, y0, x1, y1)) / static_cast<float>(cellPx);
			float & cell = obs.at<float>(gv, gu);
			cell = cell < 0.0f ? occ : std::max(cell, occ);
		}
	}
}

// Segments the object in this lane's pane and returns the instance mask
// warped into lane space (8U laneH x laneW).
bool ShapeEstimator::runModel(const TrackedObject & o, const ShapeConfig & cfg, const LaneInput & li,
                              const ObjectShape & f, State & st, cv::Mat & laneMask, cv::Mat & laneSeen){
	if(!model || frameBgr.empty() || !li.homography || li.homography->empty()) return false;
	const LaneConfig & lane = *li.lane;
	if(lane.pane < 0 || (lane.pane + 1) * paneW > frameBgr.cols) return false;

	// the padded frame, where this lane sees it
	float lx0, lx1;
	if(!lane.corridorRangeToLane(f.fx0, f.fx1, lx0, lx1)) return false;
	const float ly0 = std::clamp(f.fy0, 0.0f, 1.0f), ly1 = std::clamp(f.fy1, 0.0f, 1.0f);
	if(ly1 <= ly0) return false;
	cv::Mat hInv;
	cv::invert(*li.homography, hInv);
	cv::Rect2f region = laneRectToPane(hInv, lx0, lx1, ly0, ly1, li.laneW, li.laneH);
	// crop margin so the model sees the whole object, then clamp to the pane
	const float padW = region.width * cfg.modelPad, padH = region.height * cfg.modelPad;
	int rx0 = std::clamp(static_cast<int>(std::floor(region.x - padW)), 0, paneW - 1);
	int ry0 = std::clamp(static_cast<int>(std::floor(region.y - padH)), 0, paneH - 1);
	int rx1 = std::clamp(static_cast<int>(std::ceil(region.x + region.width + padW)), rx0 + 1, paneW);
	int ry1 = std::clamp(static_cast<int>(std::ceil(region.y + region.height + padH)), ry0 + 1, paneH);
	if(rx1 - rx0 < 16 || ry1 - ry0 < 16) return false;
	const int cropW = rx1 - rx0, cropH = ry1 - ry0;

	// A crop much wider than the network input is cut into overlapping
	// tiles; one tile per call, round-robin, so a call costs one forward
	// pass regardless of the object's length. Cells outside the tile are
	// reported as unobserved via laneSeen.
	const float maxAspect = std::max(1.0f, cfg.modelMaxAspect);
	const int tiles = std::max(1, static_cast<int>(std::ceil(static_cast<float>(cropW) / cropH / maxAspect)));
	const float overlap = std::clamp(cfg.modelTileOverlap, 0.0f, 0.5f);
	const float tileW = tiles == 1 ? cropW : cropW / (tiles - (tiles - 1) * overlap);
	const int k = tiles == 1 ? 0 : (st.tileCursor % tiles); // advanced once per update, not per lane
	const int tx0 = std::clamp(static_cast<int>(std::lround(k * tileW * (1.0f - overlap))), 0, cropW - 1);
	const int tx1 = std::clamp(static_cast<int>(std::lround(tx0 + tileW)), tx0 + 1, cropW);
	const int ax0 = rx0 + tx0, ax1 = rx0 + tx1; // tile in pane px
	const cv::Rect crop(ax0 + lane.pane * paneW, ry0, ax1 - ax0, cropH);
	const cv::Mat cropImg = frameBgr(crop);

	// the unpadded tracked box in tile px = what the instance must cover
	float tbx0, tbx1;
	if(!lane.corridorRangeToLane(o.x0, o.x1, tbx0, tbx1)) return false;
	cv::Rect2f target = laneRectToPane(hInv, tbx0, tbx1, std::clamp(o.y0, 0.0f, 1.0f), std::clamp(o.y1, 0.0f, 1.0f),
	                                   li.laneW, li.laneH);
	target &= cv::Rect2f(static_cast<float>(ax0), static_cast<float>(ry0), static_cast<float>(ax1 - ax0), static_cast<float>(cropH));
	if(target.area() <= 0) return false;
	target.x -= ax0;
	target.y -= ry0;

	SegModel::Result r;
	modelRunsThisFrame++;
	const bool ok = model->segment(cropImg, target, cfg.modelConf, cfg.modelMinIou, cfg.modelMaskThresh, r);
	modelMsThisFrame += model->lastForwardMs();

	// tile px -> pane px -> lane px; the seen area is the tile itself
	cv::Mat T = cv::Mat::eye(3, 3, CV_64F);
	T.at<double>(0, 2) = ax0;
	T.at<double>(1, 2) = ry0;
	const cv::Mat M = *li.homography * T;
	const cv::Mat seenTile(cropH, ax1 - ax0, CV_8U, cv::Scalar(255));
	cv::warpPerspective(seenTile, laneSeen, M, cv::Size(li.laneW, li.laneH), cv::INTER_NEAREST);
	if(!ok) return false;
	cv::warpPerspective(r.mask, laneMask, M, cv::Size(li.laneW, li.laneH), cv::INTER_NEAREST);
	return true;
}

void ShapeEstimator::extractOutline(const cv::Mat & ema, const ShapeConfig & cfg, ObjectShape & shape){
	shape.outline.clear();
	cv::Mat big;
	cv::resize(ema, big, cv::Size(ema.cols * kUp, ema.rows * kUp), 0, 0, cv::INTER_LINEAR);
	cv::Mat bin = big >= cfg.threshold;
	if(cfg.closeCells > 0){
		const int k = cfg.closeCells * kUp * 2 + 1;
		cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k)));
	}
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	int best = -1;
	double bestArea = 0;
	for(size_t i = 0; i < contours.size(); i++){
		const double a = cv::contourArea(contours[i]);
		if(a > bestArea){ bestArea = a; best = static_cast<int>(i); }
	}
	// at least two grid cells worth of area
	if(best < 0 || bestArea < 2.0 * kUp * kUp) return;
	std::vector<cv::Point> poly;
	cv::approxPolyDP(contours[best], poly, std::max(0.0f, cfg.simplify) * kUp, true);
	if(poly.size() < 3) return;
	shape.outline.reserve(poly.size());
	for(const auto & p : poly){
		shape.outline.emplace_back(shape.toS((p.x + 0.5f) / kUp), shape.toY((p.y + 0.5f) / kUp));
	}
}

void ShapeEstimator::updateCorridor(CorridorResult & cr, const ShapeConfig & cfg, const std::vector<LaneInput> & lanes){
	if(!cfg.enabled) return;
	const bool useModel = cfg.source == 2;
	if(useModel) ensureModel();
	const bool modelOk = useModel && model && !frameBgr.empty();

	// largest objects first: they get the model budget
	std::vector<TrackedObject *> order;
	for(auto & o : cr.objects) order.push_back(&o);
	std::sort(order.begin(), order.end(), [](const TrackedObject * a, const TrackedObject * b){
		return a->w * a->h > b->w * b->h;
	});

	for(TrackedObject * op : order){
		TrackedObject & o = *op;
		if(cfg.confirmedOnly && !o.confirmed) continue;
		if(o.w <= 0 || o.h <= 0) continue;

		State & st = states[o.id];
		st.lastSeen = frameIndex;
		ObjectShape f = frameFor(o, cfg);
		if(st.ema.empty() || st.ema.cols != f.gridW || st.ema.rows != f.gridH){
			st.ema = cv::Mat::zeros(f.gridH, f.gridW, CV_32F);
			st.obs.clear();
			st.fresh = true;
		}

		cv::Mat obs(f.gridH, f.gridW, CV_32F, cv::Scalar(kUnobserved));
		std::string src;
		float alpha = std::clamp(cfg.ema, 0.01f, 1.0f);
		const bool stopped = o.phase == "stopped";
		if(!stopped){
			if(modelOk){
				const bool due = ((frameIndex + static_cast<uint64_t>(o.id)) % std::max(1, cfg.modelEveryN)) == 0
				                 && modelRunsThisFrame < cfg.modelMaxPerFrame;
				if(due){
					bool any = false;
					for(int li : cr.lanes){
						if(li < 0 || li >= static_cast<int>(lanes.size()) || !lanes[li].lane) continue;
						cv::Mat laneMask, laneSeen;
						if(runModel(o, cfg, lanes[li], f, st, laneMask, laneSeen)){
							sampleLane(laneMask, lanes[li], f, obs, &laneSeen);
							any = true;
						}
					}
					st.tileCursor++;
					if(any){ st.modelMiss = 0; src = "model"; }
					else st.modelMiss++;
				}
				if(src.empty() && st.modelMiss > cfg.modelHoldFrames){
					// the model has gone silent: lean on the closed detector
					// mask, gently if the grid still holds a model silhouette
					for(int li : cr.lanes){
						if(li < 0 || li >= static_cast<int>(lanes.size()) || !lanes[li].lane || !lanes[li].mask) continue;
						sampleLane(*lanes[li].mask, lanes[li], f, obs);
					}
					src = "mask";
					if(st.lastSource == "model" || st.lastSource == "model+mask"){
						alpha = std::clamp(cfg.fallbackEma, 0.01f, alpha);
						src = "model+mask";
					}
				}
			}else{
				for(int li : cr.lanes){
					if(li < 0 || li >= static_cast<int>(lanes.size()) || !lanes[li].lane) continue;
					const cv::Mat * m = cfg.source == 0 ? lanes[li].mask : lanes[li].foreground;
					if(m) sampleLane(*m, lanes[li], f, obs);
				}
				src = cfg.source == 0 ? "mask" : "foreground";
			}
		}

		if(!src.empty()){
			const float a = alpha;
			bool anyObserved = false;
			// cells not looked at this call (other tile, outside the lane)
			// keep their last observation for the pixel layer
			if(st.obs.size() != static_cast<size_t>(f.gridW) * f.gridH){
				st.obs.assign(static_cast<size_t>(f.gridW) * f.gridH, 128);
			}
			for(int y = 0; y < f.gridH; y++){
				const float * orow = obs.ptr<float>(y);
				float * erow = st.ema.ptr<float>(y);
				for(int x = 0; x < f.gridW; x++){
					if(orow[x] < 0.0f) continue;
					anyObserved = true;
					erow[x] = st.fresh ? orow[x] : erow[x] * (1.0f - a) + orow[x] * a;
					st.obs[static_cast<size_t>(y) * f.gridW + x] = orow[x] >= 0.5f ? 255 : 0;
				}
			}
			if(anyObserved){
				st.fresh = false;
				st.lastSource = src;
			}
		}

		ObjectShape & shape = o.shape;
		shape = f;
		shape.valid = !st.fresh;
		shape.source = st.lastSource;
		shape.ema.resize(static_cast<size_t>(f.gridW) * f.gridH);
		for(int y = 0; y < f.gridH; y++){
			const float * erow = st.ema.ptr<float>(y);
			for(int x = 0; x < f.gridW; x++){
				shape.ema[static_cast<size_t>(y) * f.gridW + x] =
					static_cast<uint8_t>(std::clamp(erow[x], 0.0f, 1.0f) * 255.0f + 0.5f);
			}
		}
		shape.obs = st.obs.empty() ? std::vector<uint8_t>(shape.ema.size(), 128) : st.obs;
		if(shape.valid) extractOutline(st.ema, cfg, shape);
	}
}

} // namespace tracking
