#include "TrackingManager.h"

#include <algorithm>
#include <chrono>
#include <cmath>

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
			for(auto & cp : corridors) if(cp.tracker) cp.tracker->reset();
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

void TrackingManager::rebuildPipelines(int paneCount, int paneW, int paneH){
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
		FrameQuad quad = lane.quad;
		if(!quad.isConvex()){
			ofLogWarning("TrackingManager") << "lane " << lane.id << ": invalid quad, using full pane";
			quad = FrameQuad{};
		}
		// Rectified size: keep the quad's on-screen aspect, cap the width,
		// floor the height (a thin viaduct band would otherwise end up a
		// few pixels tall and starve the flow estimator).
		const float aspect = std::max(quad.aspect(paneW, paneH), 0.05f);
		const auto len = [&](int a, int b){
			return std::hypot((quad.pts[b][0] - quad.pts[a][0]) * paneW,
			                  (quad.pts[b][1] - quad.pts[a][1]) * paneH);
		};
		const float srcW = (len(0, 1) + len(3, 2)) * 0.5f;
		lp.laneW = std::clamp(static_cast<int>(std::lround(srcW)), 32, std::max(32, cfg.laneMaxWidth));
		lp.laneH = std::max(cfg.laneMinHeight, static_cast<int>(std::lround(lp.laneW / aspect)));
		lp.laneH = std::min(lp.laneH, std::max(32, paneH));
		const cv::Point2f src[4] = {
			{quad.pts[0][0] * paneW, quad.pts[0][1] * paneH},
			{quad.pts[1][0] * paneW, quad.pts[1][1] * paneH},
			{quad.pts[2][0] * paneW, quad.pts[2][1] * paneH},
			{quad.pts[3][0] * paneW, quad.pts[3][1] * paneH}};
		const cv::Point2f dst[4] = {
			{0, 0},
			{static_cast<float>(lp.laneW), 0},
			{static_cast<float>(lp.laneW), static_cast<float>(lp.laneH)},
			{0, static_cast<float>(lp.laneH)}};
		lp.homography = cv::getPerspectiveTransform(src, dst);
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
	applyParams(); // fills the grouping params

	builtPanes = paneCount;
	builtPaneW = paneW;
	builtPaneH = paneH;
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

	// Side-by-side dual-module frames are twice as wide as tall.
	const int paneCount = full.cols >= full.rows * 2 ? 2 : 1;
	const int paneW = full.cols / paneCount;
	const int paneH = full.rows;

	if(resetRequested.exchange(false)) builtPanes = 0;
	if(builtPanes != paneCount || builtPaneW != paneW || builtPaneH != paneH || builtKind != wantKind){
		rebuildPipelines(paneCount, paneW, paneH);
	}
	const double dtSec = lastTMs < 0 ? 1.0 / 30.0 : (tMs - lastTMs) / 1000.0;
	if(lastTMs >= 0 && (tMs < lastTMs || tMs - lastTMs > kMaxGapMs)){
		for(auto & lp : lanes) lp.detector->reset();
		for(auto & cp : corridors) cp.tracker->reset();
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
		const cv::Mat pane = full(cv::Rect(lp.lane.pane * paneW, 0, paneW, paneH));
		cv::Mat laneImg;
		cv::warpPerspective(pane, laneImg, lp.homography, cv::Size(lp.laneW, lp.laneH), cv::INTER_LINEAR);

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
		cp.tracker->update(dets, tMs);
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
			inputs[i].laneW = lanes[i].laneW;
			inputs[i].laneH = lanes[i].laneH;
		}
		shapes.beginFrame(fullBgr, paneW, paneH, ++frameCounter);
		for(auto & cr : res.corridors) shapes.updateCorridor(cr, cfg.shapeFor(cr.id), inputs);
		shapes.endFrame();
	}
	const double shapeMs = nowMs() - s0;

	res.canvas = cfg.canvas;
	res.frameTMs = tMs;
	res.decodeMs = decodeMs;
	res.detectMs = detectMs;
	res.trackMs = trackMs;
	res.shapeMs = shapeMs;
	res.analysisW = paneW;
	res.analysisH = paneH;
	res.detectorName = kindName(builtKind);
	return true;
}

} // namespace tracking
