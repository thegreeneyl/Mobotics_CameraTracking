#include "TrackingManager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>

#include <opencv2/imgproc.hpp>

#include "ofLog.h"

#include "../util/JpegDecode.h"
#include "DetectionGrouping.h"
#include "TrackingParams.h"

namespace tracking {

namespace {
// Frame gaps beyond this (or backwards time) reset background models and
// tracks — covers live<->playback switches and recording wraps.
constexpr double kMaxGapMs = 5000.0;

constexpr int kFlow = 0, kBgs = 1, kYolo = 2;

int kindFromName(const std::string & n){
	if(n == "yolo") return kYolo;
	if(n == "bgs") return kBgs;
	return kFlow;
}
const char * kindName(int k){
	return k == kYolo ? "yolo" : k == kBgs ? "bgs" : "flow";
}

// Result presentation. The tracker box stays the measurement; this only
// rewrites what is published. A boat with a real silhouette (not the
// rectangle stand-in the smoother draws before the first outline) gets
// that outline's bounds as its box and the center of those bounds as its
// center. The flow velocity is left alone. A pinned corridor end is not
// stretched back onto the box: the visible hull is the result, and the
// clipped flags still say the object continues past the frame. Trains are
// not touched — the model outline under-covers a long train.
void presentResults(std::vector<TrackedObject> & objects,
                    std::map<int, std::vector<std::pair<float, float>>> & trails,
                    int trailLen){
	std::set<int> seen;
	for(auto & o : objects){
		o.rawX0 = o.x0; o.rawX1 = o.x1; o.rawY0 = o.y0; o.rawY1 = o.y1;
		o.rawX = o.x; o.rawY = o.y; o.rawW = o.w; o.rawH = o.h;
		o.rawTrail = o.trail;
		o.hullBox = false;
		const bool boat = o.label == "boat" || o.corridor == "boat";
		if(boat && o.shape.outline.size() >= 3 && o.w > 1e-4f && o.h > 1e-4f){
			const float mx = 0.02f * o.w, my = 0.02f * o.h;
			bool interior = false;
			float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
			for(const auto & p : o.shape.outline){
				x0 = std::min(x0, p.first); x1 = std::max(x1, p.first);
				y0 = std::min(y0, p.second); y1 = std::max(y1, p.second);
				if(p.first > o.x0 + mx && p.first < o.x1 - mx
				   && p.second > o.y0 + my && p.second < o.y1 - my) interior = true;
			}
			if(interior && x1 - x0 > 1e-4f && y1 - y0 > 1e-4f){
				o.x0 = x0; o.x1 = x1; o.y0 = y0; o.y1 = y1;
				o.w = x1 - x0; o.h = y1 - y0;
				o.x = (x0 + x1) * 0.5f;
				o.y = (y0 + y1) * 0.5f;
				o.hullBox = true;
			}
		}
		if(boat){
			auto & tr = trails[o.id];
			tr.emplace_back(o.x, o.y);
			if(trailLen >= 0 && static_cast<int>(tr.size()) > trailLen){
				tr.erase(tr.begin(), tr.begin() + (static_cast<int>(tr.size()) - trailLen));
			}
			o.trail = tr;
		}
		seen.insert(o.id);
	}
	for(auto it = trails.begin(); it != trails.end();){
		if(!seen.count(it->first)) it = trails.erase(it);
		else ++it;
	}
}

bool sameLayouts(const std::vector<LayoutConfig> & a, const std::vector<LayoutConfig> & b){
	if(a.size() != b.size()) return false;
	for(size_t i = 0; i < a.size(); i++){
		if(a[i].name != b[i].name || a[i].panes != b[i].panes
		   || a[i].recordings != b[i].recordings || a[i].lanes != b[i].lanes) return false;
	}
	return true;
}
} // namespace

// ------------------------------------------------------------ TrackingConfig

FlowDetectorConfig TrackingConfig::flowFor(const std::string & corridorId) const {
	FlowDetectorConfig f = flow;
	if(const CorridorConfig * c = corridor(corridorId)) applyFlowOverrides(f, c->flow);
	return f;
}

TrackerConfig TrackingConfig::trackerFor(const std::string & corridorId) const {
	TrackerConfig t = tracker;
	if(const CorridorConfig * c = corridor(corridorId)) applyTrackerOverrides(t, c->tracker);
	return t;
}

ShapeConfig TrackingConfig::shapeFor(const std::string & corridorId) const {
	ShapeConfig s = shape;
	if(const CorridorConfig * c = corridor(corridorId)) applyShapeOverrides(s, c->shape);
	return s;
}

// ----------------------------------------------------------- TrackingManager

TrackingManager::~TrackingManager(){
	stop();
}

double TrackingManager::nowMs(){
	using namespace std::chrono;
	return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

void TrackingManager::setup(const TrackingConfig & config){
	stop();
	cfg = config;
	lensCorrector.set(cfg.lens);
	shapes.setConfig(cfg.shape);
	enabled = cfg.enabled;
	detectorKind = kindFromName(cfg.detector);
	running = true;
	worker = std::thread(&TrackingManager::threadFn, this);
}

void TrackingManager::setupSync(const TrackingConfig & config){
	stop();
	cfg = config;
	lensCorrector.set(cfg.lens);
	shapes.setConfig(cfg.shape);
	enabled = cfg.enabled;
	detectorKind = kindFromName(cfg.detector);
}

void TrackingManager::stop(){
	running = false;
	inputCv.notify_all();
	if(worker.joinable()) worker.join();
	lanes.clear();
	corridors.clear();
	shapes.reset();
	builtPanes = 0;
	builtKind = -1;
	lastTMs = -1;
}

void TrackingManager::setDetectorType(const std::string & type){
	detectorKind = kindFromName(type);
	yoloUnavailable = false;
}

std::string TrackingManager::getDetectorType() const {
	return kindName(detectorKind);
}

void TrackingManager::setSourceName(const std::string & name){
	std::lock_guard<std::mutex> lock(configMutex);
	pendingSource = name;
	hasPendingSource = true;
}

void TrackingManager::setLiveConfig(const TrackingConfig & config){
	std::lock_guard<std::mutex> lock(configMutex);
	pendingCfg = config;
	hasPendingCfg = true;
}

// Worker thread: pull the newest GUI tuning / source name into cfg. A
// layout or source change invalidates the pipeline geometry -> rebuild;
// parameter changes are pushed into the live detectors and trackers.
void TrackingManager::applyPendingConfig(){
	TrackingConfig next;
	bool haveCfg = false;
	std::string source;
	bool haveSource = false;
	{
		std::lock_guard<std::mutex> lock(configMutex);
		if(hasPendingCfg){
			next = pendingCfg;
			hasPendingCfg = false;
			haveCfg = true;
		}
		if(hasPendingSource){
			source = pendingSource;
			hasPendingSource = false;
			haveSource = true;
		}
	}
	bool rebuild = false;
	if(haveSource && source != cfg.sourceName){
		cfg.sourceName = source;
		rebuild = true;
	}
	if(haveCfg){
		if(!sameLayouts(cfg.layouts, next.layouts)) rebuild = true;
		const bool lensChanged = cfg.lens != next.lens;
		const bool shapeSourceChanged = cfg.shape.source != next.shape.source
			|| cfg.shape.gridW != next.shape.gridW || cfg.shape.gridH != next.shape.gridH;
		cfg.layouts = next.layouts;
		cfg.corridors = next.corridors;
		cfg.canvas = next.canvas;
		cfg.flow = next.flow;
		cfg.bgs = next.bgs;
		cfg.tracker = next.tracker;
		cfg.smoothing = next.smoothing;
		cfg.laneMaxWidth = next.laneMaxWidth;
		cfg.laneMinHeight = next.laneMinHeight;
		cfg.yolo.confThreshold = next.yolo.confThreshold;
		cfg.yolo.nmsThreshold = next.yolo.nmsThreshold;
		cfg.lens = next.lens;
		// model path / input stay as loaded (startup-only); everything else
		// in `shape` is live
		const std::string modelPath = cfg.shape.modelPath, modelFile = cfg.shape.modelFile;
		const int modelInputW = cfg.shape.modelInputW, modelInputH = cfg.shape.modelInputH;
		cfg.shape = next.shape;
		cfg.shape.modelPath = modelPath;
		cfg.shape.modelFile = modelFile;
		cfg.shape.modelInputW = modelInputW;
		cfg.shape.modelInputH = modelInputH;
		shapes.setConfig(cfg.shape);
		if(shapeSourceChanged) shapes.reset(); // a different source draws a different silhouette
		if(lensChanged){
			// The whole image moved. Background models and tracks were
			// learned on the previous warp and must start over.
			lensCorrector.set(cfg.lens);
			for(auto & lp : lanes) if(lp.detector) lp.detector->reset();
			for(auto & cp : corridors){
				if(cp.tracker) cp.tracker->reset();
				cp.feedback.clear();
				cp.smoother.reset();
				cp.resultTrails.clear();
			}
			shapes.reset();
		}
		if(!rebuild) applyParams();
	}
	if(rebuild){
		lanes.clear();
		corridors.clear();
		builtPanes = 0;
	}
}

void TrackingManager::applyParams(){
	for(auto & lp : lanes){
		const std::string & cid = lp.lane.corridor;
		if(auto * f = dynamic_cast<FlowMotionDetector *>(lp.detector.get())){
			f->setConfig(cfg.flowFor(cid));
		}else if(auto * b = dynamic_cast<BgsDetector *>(lp.detector.get())){
			b->setConfig(cfg.bgs);
		}else if(auto * y = dynamic_cast<YoloDetector *>(lp.detector.get())){
			y->setThresholds(cfg.yolo.confThreshold, cfg.yolo.nmsThreshold);
		}
	}
	for(auto & cp : corridors){
		cp.tracker->setConfig(cfg.trackerFor(cp.id));
		cp.smoother.setConfig(cfg.smoothing);
		const FlowDetectorConfig f = cfg.flowFor(cp.id);
		cp.grouping.gapFrac = f.mergeGapFrac;
		cp.grouping.angleDeg = f.mergeAngleDeg;
		cp.grouping.speedRatio = f.mergeSpeedRatio;
		cp.grouping.lateralOverlap = f.mergeLateralOverlap;
		cp.grouping.minSpeed = f.minSpeed;
	}
}

void TrackingManager::submitRawJpeg(const uint8_t * data, size_t size, double tMs){
	if(!running || !enabled || size == 0) return;
	{
		std::lock_guard<std::mutex> lock(inputMutex);
		pendingJpeg.assign(data, data + size); // stale pending frame is replaced
		pendingTMs = tMs;
		hasPending = true;
	}
	inputCv.notify_one();
}

bool TrackingManager::getLatestResults(TrackingResults & out, uint64_t & lastSeenRevision) const {
	std::lock_guard<std::mutex> lock(resultsMutex);
	if(results.revision == 0 || results.revision == lastSeenRevision) return false;
	out = results;
	lastSeenRevision = results.revision;
	return true;
}

// ------------------------------------------------------------------ layouts

LayoutConfig TrackingManager::fallbackLayout(int paneCount){
	LayoutConfig l;
	l.name = paneCount > 1 ? "dual (auto)" : "single (auto)";
	l.panes = paneCount;
	for(int i = 0; i < paneCount; i++){
		LaneConfig lane;
		lane.id = "pane" + std::to_string(i + 1);
		lane.pane = i;
		lane.corridor = lane.id;
		l.lanes.push_back(lane);
	}
	return l;
}

LayoutConfig TrackingManager::findLayout(const std::vector<LayoutConfig> & layouts, int paneCount,
                                         const std::string & sourceName){
	const LayoutConfig * generic = nullptr;
	for(const auto & l : layouts){
		if(l.panes != paneCount || l.lanes.empty()) continue;
		for(const auto & r : l.recordings){
			if(!r.empty() && sourceName.find(r) != std::string::npos) return l;
		}
		if(l.recordings.empty() && !generic) generic = &l;
	}
	if(generic) return *generic;
	return fallbackLayout(paneCount);
}

std::unique_ptr<IDetector> TrackingManager::makeDetector(const std::string & corridorId){
	switch(builtKind){
		case kYolo: return std::make_unique<YoloDetector>(cfg.yolo);
		case kBgs: return std::make_unique<BgsDetector>(cfg.bgs);
		default: return std::make_unique<FlowMotionDetector>(cfg.flowFor(corridorId));
	}
}

void TrackingManager::retargetMaps(int originX, int originY){
	const float iw = static_cast<float>(builtPaneW);
	const float ih = static_cast<float>(builtPaneH);
	for(auto & lp : lanes){
		const int w = lp.laneW, h = lp.laneH;
		lp.map = LaneMap::build(lp.lane, iw, ih, static_cast<float>(originX), static_cast<float>(originY));
		if(w > 0 && h > 0) lp.map.prepare(w, h);
		lp.homography = lp.map.paneToLane;
	}
	builtOriginX = originX;
	builtOriginY = originY;
}

void TrackingManager::rebuildPipelines(int paneCount, int paneW, int paneH, int originX, int originY){
	lanes.clear();
	corridors.clear();

	builtKind = detectorKind;
	if(builtKind == kYolo){
		auto probe = std::make_unique<YoloDetector>(cfg.yolo);
		if(!probe->isLoaded()){
			ofLogWarning("TrackingManager")
				<< "YOLO model not available (" << cfg.yolo.modelPath
				<< ") — falling back to optical flow";
			detectorKind = kFlow;
			builtKind = kFlow;
			yoloUnavailable = true;
		}
	}

	const LayoutConfig layout = findLayout(cfg.layouts, paneCount, cfg.sourceName);
	layoutName = layout.name;

	for(const auto & lane : layout.lanes){
		if(lane.pane < 0 || lane.pane >= paneCount) continue;
		LanePipeline lp;
		lp.lane = lane;
		if(!lane.isWarpable()){
			ofLogWarning("TrackingManager") << "lane " << lane.id
				<< ": invalid strip, using the full pane";
		}
		// Rectified size: the strip's centreline length and mean height, cap
		// the width, floor the height (a thin viaduct band would otherwise
		// end up a few pixels tall and starve the flow estimator).
		lp.map = LaneMap::build(lane, static_cast<float>(paneW), static_cast<float>(paneH),
		                        static_cast<float>(originX), static_cast<float>(originY));
		lp.map.rectifiedSize(cfg.laneMaxWidth, cfg.laneMinHeight, paneH, lp.laneW, lp.laneH);
		lp.map.prepare(lp.laneW, lp.laneH);
		lp.homography = lp.map.paneToLane;
		lp.detector = makeDetector(lane.corridor);
		lanes.push_back(std::move(lp));
	}

	for(size_t i = 0; i < lanes.size(); i++){
		const std::string & cid = lanes[i].lane.corridor;
		auto it = std::find_if(corridors.begin(), corridors.end(),
		                       [&](const CorridorPipeline & c){ return c.id == cid; });
		if(it == corridors.end()){
			CorridorPipeline cp;
			cp.id = cid;
			cp.label = cfg.corridor(cid) ? cfg.corridor(cid)->label : cid;
			cp.tracker = std::make_unique<MultiObjectTracker>(cfg.trackerFor(cid));
			corridors.push_back(std::move(cp));
			it = corridors.end() - 1;
		}
		it->lanes.push_back(static_cast<int>(i));
	}
	// lane ends inside the corridor (several overlapping lanes): the flow
	// gain is not learnt from blob edges there, see setInternalBoundaries
	for(auto & cp : corridors){
		std::vector<float> inner;
		for(int li : cp.lanes){
			const LaneConfig & lane = lanes[li].lane;
			if(lane.s0 > 0.001f) inner.push_back(lane.s0);
			if(lane.s1 < 0.999f) inner.push_back(lane.s1);
		}
		cp.tracker->setInternalBoundaries(std::move(inner));
	}
	applyParams(); // fills the grouping params

	builtPanes = paneCount;
	builtPaneW = paneW;
	builtPaneH = paneH;
	builtOriginX = originX;
	builtOriginY = originY;
	lastTMs = -1;
	ofLogNotice("TrackingManager")
		<< "layout '" << layoutName << "' for " << paneCount << " pane(s), "
		<< lanes.size() << " lane(s), " << corridors.size() << " corridor(s), detector "
		<< kindName(builtKind);
}

// ------------------------------------------------------------------- worker

void TrackingManager::threadFn(){
	std::vector<uint8_t> jpeg;
	double tMs = 0;

	while(running){
		{
			std::unique_lock<std::mutex> lock(inputMutex);
			inputCv.wait(lock, [this]{ return !running || hasPending; });
			if(!running) break;
			jpeg.swap(pendingJpeg);
			tMs = pendingTMs;
			hasPending = false;
		}
		if(!enabled) continue;

		TrackingResults res;
		if(!processFrame(jpeg.data(), jpeg.size(), tMs, res)) continue;

		{
			std::lock_guard<std::mutex> lock(resultsMutex);
			res.revision = ++resultsRevision;
			results = std::move(res);
		}
	}
}

bool TrackingManager::processFrame(const uint8_t * data, size_t size, double tMs, TrackingResults & res){
	applyPendingConfig();
	int wantKind = detectorKind;
	if(wantKind == kYolo && yoloUnavailable) wantKind = kFlow;

	// Analysis decode: reduced scaled decode of only what is needed —
	// grayscale for flow/bgs, BGR for YOLO or when the shape estimator's
	// segmentation model is on (the detector then gets a gray copy).
	const double t0 = nowMs();
	bool shapeColor = shapes.wantsColor(cfg.shape);
	for(const auto & cc : cfg.corridors) shapeColor = shapeColor || shapes.wantsColor(cfg.shapeFor(cc.id));
	const bool wantColor = wantKind == kYolo || shapeColor;
	cv::Mat full, fullBgr;
	const bool decoded = wantColor
		? jpegdecode::decodeToBgrMat(data, size, full, cfg.analysisReduce)
		: jpegdecode::decodeToGrayMat(data, size, full, cfg.analysisReduce);
	if(!decoded || full.empty()){
		ofLogWarning("TrackingManager") << "analysis decode failed (" << size << " bytes)";
		return false;
	}
	{
		cv::Mat corrected;
		lensCorrector.apply(full, corrected);
		full = corrected;
	}
	if(wantColor){
		fullBgr = full;
		if(wantKind != kYolo) cv::cvtColor(fullBgr, full, cv::COLOR_BGR2GRAY);
	}
	const double decodeMs = nowMs() - t0;

	// Pane count comes from the source (the corrector), not from the
	// extended frame: a tall recovered border can drop a side-by-side
	// frame under the 2:1 test. UV 0..1 is the filled frame; the mat
	// around it is the recovered border.
	const LensGeometry g = lensCorrector.geometry();
	const int paneCount = std::max(1, g.panes);
	const int outW = g.outPaneW > 0 ? g.outPaneW : full.cols / paneCount;
	const int outH = g.outPaneH > 0 ? g.outPaneH : full.rows;
	const int innerW = g.innerW > 0 ? g.innerW : outW;
	const int innerH = g.innerH > 0 ? g.innerH : outH;
	const int originX = g.innerW > 0 ? g.innerX : 0;
	const int originY = g.innerW > 0 ? g.innerY : 0;

	if(resetRequested.exchange(false)) builtPanes = 0;
	if(builtPanes != paneCount || builtPaneW != innerW || builtPaneH != innerH || builtKind != wantKind){
		rebuildPipelines(paneCount, innerW, innerH, originX, originY);
	}else if(builtOriginX != originX || builtOriginY != originY){
		retargetMaps(originX, originY);
	}
	const double dtSec = lastTMs < 0 ? 1.0 / 30.0 : (tMs - lastTMs) / 1000.0;
	if(lastTMs >= 0 && (tMs < lastTMs || tMs - lastTMs > kMaxGapMs)){
		for(auto & lp : lanes) lp.detector->reset();
		for(auto & cp : corridors){
			cp.tracker->reset();
			cp.feedback.clear();
			cp.smoother.reset();
			cp.resultTrails.clear();
		}
		shapes.reset();
	}
	lastTMs = tMs;

	res = TrackingResults{};
	res.layoutName = layoutName;
	res.paneCount = paneCount;
	res.lanes.resize(lanes.size());
	double detectMs = 0, trackMs = 0;

	// 1. per lane: warp + detect (lane-normalized detections)
	std::vector<std::vector<Detection>> laneDets(lanes.size());
	for(size_t i = 0; i < lanes.size(); i++){
		LanePipeline & lp = lanes[i];
		const cv::Rect paneRoi(lp.lane.pane * outW, 0, outW, outH);
		if(paneRoi.x < 0 || paneRoi.width <= 0 || paneRoi.height <= 0
		   || paneRoi.x + paneRoi.width > full.cols || paneRoi.y + paneRoi.height > full.rows) continue;
		const cv::Mat pane = full(paneRoi);
		cv::Mat laneImg;
		lp.map.warp(pane, laneImg, cv::INTER_LINEAR);

		const double d0 = nowMs();
		laneDets[i] = lp.detector->detect(laneImg, std::max(dtSec, 1e-3));
		const double dms = nowMs() - d0;
		detectMs += dms;
		for(auto & d : laneDets[i]) d.lane = static_cast<int>(i);

		LaneDebug & dbg = res.lanes[i];
		dbg.lane = lp.lane;
		dbg.detections = laneDets[i];
		dbg.detectMs = dms;
		const cv::Mat & m = lp.detector->debugMask();
		if(!m.empty() && m.type() == CV_8UC1 && m.isContinuous()){
			dbg.maskW = m.cols;
			dbg.maskH = m.rows;
			dbg.mask.assign(m.data, m.data + static_cast<size_t>(m.cols) * m.rows);
		}
	}

	// 2. per corridor: map lanes -> corridor axis, group, track
	res.corridors.resize(corridors.size());
	for(size_t ci = 0; ci < corridors.size(); ci++){
		CorridorPipeline & cp = corridors[ci];
		std::vector<Detection> dets;
		for(int li : cp.lanes){
			const LaneConfig & lane = lanes[li].lane;
			const float span = lane.s1 - lane.s0;
			const bool rev = lane.reversed();
			for(const Detection & ld : laneDets[li]){
				// optional lateral band: the road behind the viaduct is
				// inside the quad but outside the band
				if(lane.hasYBand() && (ld.cy() < lane.yBand0 || ld.cy() > lane.yBand1)) continue;
				Detection d = ld;
				const float sa = lane.toCorridor(ld.x0);
				const float sb = lane.toCorridor(ld.x1);
				d.x0 = std::min(sa, sb);
				d.x1 = std::max(sa, sb);
				d.vx = ld.vx * span; // sign flips for reversed lanes
				d.area = ld.area * std::fabs(span);
				// Clipped only where the touched lane end IS a corridor end.
				// Lane x=0 sits at s0, lane x=1 at s1; for a reversed lane
				// s0 is the corridor's far end, so the flags swap sides.
				const bool s0IsStart = lane.s0 <= 0.001f, s1IsEnd = lane.s1 >= 0.999f;
				const bool s0IsEnd = lane.s0 >= 0.999f, s1IsStart = lane.s1 <= 0.001f;
				if(!rev){
					d.touchesLeft = ld.touchesLeft && s0IsStart;
					d.touchesRight = ld.touchesRight && s1IsEnd;
				}else{
					d.touchesLeft = ld.touchesRight && s1IsStart;
					d.touchesRight = ld.touchesLeft && s0IsEnd;
				}
				dets.push_back(d);
			}
		}
		if(cp.lanes.size() > 1) dets = groupDetections(dets, cp.grouping);

		const double k0 = nowMs();
		cp.tracker->update(dets, tMs, &cp.feedback);
		CorridorResult & cr = res.corridors[ci];
		cr.id = cp.id;
		cr.label = cp.label;
		cr.lanes = cp.lanes;
		cr.placement = cfg.placementFor(cp.id);
		cr.objects = cp.tracker->getObjects();
		for(auto & o : cr.objects){
			o.label = cp.label;
			o.corridor = cp.id;
		}
		trackMs += nowMs() - k0;
	}

	// 3. per tracked object: silhouette from the lane masks / the model
	const double s0 = nowMs();
	{
		std::vector<ShapeEstimator::LaneInput> inputs(lanes.size());
		for(size_t i = 0; i < lanes.size(); i++){
			inputs[i].lane = &lanes[i].lane;
			inputs[i].mask = &lanes[i].detector->debugMask();
			inputs[i].foreground = &lanes[i].detector->shapeMask();
			inputs[i].homography = &lanes[i].homography;
			inputs[i].warp = &lanes[i].map;
			inputs[i].laneW = lanes[i].laneW;
			inputs[i].laneH = lanes[i].laneH;
		}
		shapes.beginFrame(fullBgr, outW, outH, ++frameCounter);
		for(size_t ci = 0; ci < res.corridors.size() && ci < corridors.size(); ci++){
			CorridorResult & cr = res.corridors[ci];
			const ShapeConfig scfg = cfg.shapeFor(cr.id);
			shapes.updateCorridor(cr, scfg, inputs);
			// what the estimator learned goes back to the tracker next frame
			std::map<int, ShapeFeedback> & fb = corridors[ci].feedback;
			fb.clear();
			const bool modelAvailable = shapes.modelActive(scfg);
			for(const auto & o : cr.objects) fb[o.id] = ShapeEstimator::feedbackFor(o, modelAvailable);
		}
		shapes.endFrame();
	}
	const double shapeMs = nowMs() - s0;

	// 4. publisher-side smoothing, then the result box. Smoothing still
	//    rides the tracker box (nothing feeds back). Afterwards a boat's
	//    published box and center are taken from the drawn outline; the
	//    tracker box is kept beside it for the raw view.
	for(size_t ci = 0; ci < res.corridors.size() && ci < corridors.size(); ci++){
		corridors[ci].smoother.apply(res.corridors[ci].objects, tMs);
		const int trailLen = cfg.trackerFor(res.corridors[ci].id).trailLen;
		presentResults(res.corridors[ci].objects, corridors[ci].resultTrails, trailLen);
	}

	res.canvas = cfg.canvas;
	res.frameTMs = tMs;
	res.decodeMs = decodeMs;
	res.detectMs = detectMs;
	res.trackMs = trackMs;
	res.shapeMs = shapeMs;
	res.analysisW = outW;
	res.analysisH = outH;
	res.lensInnerX = originX;
	res.lensInnerY = originY;
	res.lensInnerW = innerW;
	res.lensInnerH = innerH;
	res.detectorName = kindName(builtKind);
	return true;
}

} // namespace tracking
