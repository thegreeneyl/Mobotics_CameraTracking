#include "TrackingManager.h"

#include <chrono>

#include "ofLog.h"

#include "../util/JpegDecode.h"

namespace tracking {

namespace {
// Frame gaps beyond this (or backwards time) reset background models and
// tracks — covers live<->playback switches and recording wraps.
constexpr double kMaxGapMs = 5000.0;
} // namespace

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
	enabled = cfg.enabled;
	useYolo = (cfg.detector == "yolo");
	running = true;
	worker = std::thread(&TrackingManager::threadFn, this);
}

void TrackingManager::stop(){
	running = false;
	inputCv.notify_all();
	if(worker.joinable()) worker.join();
	pipelines.clear();
	lastTMs = -1;
}

void TrackingManager::setDetectorType(const std::string & type){
	useYolo = (type == "yolo");
}

std::string TrackingManager::getDetectorType() const {
	return useYolo ? "yolo" : "bgs";
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

void TrackingManager::rebuildPipelines(int paneCount, bool wantYolo){
	pipelines.clear();
	pipelinesAreYolo = false;

	if(wantYolo){
		// Probe-load the model once; per-pane instances share nothing, so
		// just build one per pane (the net itself is small to construct).
		auto probe = std::make_unique<YoloDetector>(cfg.yolo);
		if(probe->isLoaded()){
			pipelinesAreYolo = true;
		}else{
			ofLogWarning("TrackingManager")
				<< "YOLO model not available (" << cfg.yolo.modelPath
				<< ") — falling back to background subtraction";
			useYolo = false;
		}
	}

	for(int i = 0; i < paneCount; i++){
		Pipeline p;
		if(pipelinesAreYolo){
			p.detector = std::make_unique<YoloDetector>(cfg.yolo);
		}else{
			const auto zones = i < static_cast<int>(cfg.moduleZones.size())
				? cfg.moduleZones[i] : std::vector<Zone>{};
			p.detector = std::make_unique<BgsDetector>(cfg.bgs, zones);
		}
		p.tracker = std::make_unique<MultiObjectTracker>(cfg.tracker);
		pipelines.push_back(std::move(p));
	}
	lastTMs = -1;
}

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

		const bool wantYolo = useYolo;

		// Analysis decode: reduced scaled decode of only what the detector
		// needs (grayscale for BGS, BGR for YOLO).
		const double t0 = nowMs();
		cv::Mat full;
		const bool decoded = wantYolo
			? jpegdecode::decodeToBgrMat(jpeg.data(), jpeg.size(), full, cfg.analysisReduce)
			: jpegdecode::decodeToGrayMat(jpeg.data(), jpeg.size(), full, cfg.analysisReduce);
		if(!decoded || full.empty()){
			ofLogWarning("TrackingManager") << "analysis decode failed (" << jpeg.size() << " bytes)";
			continue;
		}
		const double decodeMs = nowMs() - t0;

		// Side-by-side dual-module frames are twice as wide as tall.
		const int paneCount = full.cols >= full.rows * 2 ? 2 : 1;

		if(resetRequested.exchange(false)) pipelines.clear();
		if(static_cast<int>(pipelines.size()) != paneCount || pipelinesAreYolo != wantYolo){
			rebuildPipelines(paneCount, wantYolo);
		}
		if(lastTMs >= 0 && (tMs < lastTMs || tMs - lastTMs > kMaxGapMs)){
			for(auto & p : pipelines){
				p.detector->reset();
				p.tracker->reset();
			}
		}
		lastTMs = tMs;

		TrackingResults res;
		res.modules.resize(paneCount);
		const int paneW = full.cols / paneCount;
		double detectMs = 0, trackMs = 0;

		for(int i = 0; i < paneCount; i++){
			const cv::Mat pane = full(cv::Rect(i * paneW, 0, paneW, full.rows));
			const double d0 = nowMs();
			const auto detections = pipelines[i].detector->detect(pane);
			detectMs += nowMs() - d0;

			const double k0 = nowMs();
			pipelines[i].tracker->update(detections, tMs);
			res.modules[i].objects = pipelines[i].tracker->getObjects();
			trackMs += nowMs() - k0;
		}

		res.frameTMs = tMs;
		res.decodeMs = decodeMs;
		res.detectMs = detectMs;
		res.trackMs = trackMs;
		res.analysisW = paneW;
		res.analysisH = full.rows;
		res.detectorName = pipelinesAreYolo ? "yolo" : "bgs";

		{
			std::lock_guard<std::mutex> lock(resultsMutex);
			res.revision = ++resultsRevision;
			results = std::move(res);
		}
	}
}

} // namespace tracking
