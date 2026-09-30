#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "BgsDetector.h"
#include "FlowMotionDetector.h"
#include "MultiObjectTracker.h"
#include "TrackingTypes.h"
#include "YoloDetector.h"

namespace tracking {

// Per-corridor settings: label (published class) plus optional overrides of
// the global flow / tracker parameters (flat key -> value, see
// TrackingParams.h for the key names).
struct CorridorConfig {
	std::string id;
	std::string label;
	CorridorPlacement placement; // where the corridor sits on the output canvas
	std::map<std::string, double> flow;
	std::map<std::string, double> tracker;
};

struct TrackingConfig {
	bool enabled = true;
	std::string detector = "flow"; // "flow" | "bgs" | "yolo"
	// DCT-domain reduction of the source JPEG for analysis (1|2|4|8).
	// 2 => a 3840x1080 frame is analysed at 1920x540 (960x540 per module).
	int analysisReduce = 2;
	// Rectified lane images are at most this wide (px) and at least this
	// tall; the lane keeps its aspect unless the height floor kicks in.
	int laneMaxWidth = 800;
	int laneMinHeight = 64;
	FlowDetectorConfig flow;
	BgsConfig bgs;
	TrackerConfig tracker;
	YoloConfig yolo;
	std::vector<LayoutConfig> layouts;
	std::vector<CorridorConfig> corridors;
	CanvasConfig canvas; // output canvas proportions (Lines area 832x442)
	// Name of the current source (recording folder name or "live"); used
	// to pick a layout whose `recordings` filter matches.
	std::string sourceName;

	const CorridorConfig * corridor(const std::string & id) const {
		for(const auto & c : corridors) if(c.id == id) return &c;
		return nullptr;
	}
	// Placement of a corridor (default full canvas for unconfigured ones).
	CorridorPlacement placementFor(const std::string & id) const {
		const CorridorConfig * c = corridor(id);
		return c ? c->placement : CorridorPlacement{};
	}
	// Effective per-corridor parameters (global + overrides).
	FlowDetectorConfig flowFor(const std::string & corridorId) const;
	TrackerConfig trackerFor(const std::string & corridorId) const;
};

// Owns the tracking worker thread. Input: raw JPEG bytes tapped from the
// live client or the playback player (latest-only slot — stale frames are
// dropped, never queued, so a slow detector can never stall the sources or
// the UI). The worker does its OWN reduced decode, picks the layout for the
// frame (pane count from the aspect ratio: width >= 2*height means the
// side-by-side "BOTH" view; source name for layout filters), warps every
// lane quad to an upright rectangle, runs one detector per lane, maps the
// detections to corridor coordinates, groups blobs of one object across
// lanes and runs one tracker per corridor. Publishes a TrackingResults
// snapshot per analysed frame.
//
// No UI dependencies (same rule as src/mobotix/).
class TrackingManager {
public:
	~TrackingManager();

	void setup(const TrackingConfig & config); // starts the worker thread
	void stop();

	void setEnabled(bool e){ enabled = e; }
	bool isEnabled() const { return enabled; }

	// "flow", "bgs" or "yolo". YOLO is loaded lazily on the worker thread;
	// if the model cannot be loaded it logs a warning and falls back to flow.
	void setDetectorType(const std::string & type);
	std::string getDetectorType() const;

	// Source identity for layout matching. A change rebuilds the pipeline.
	void setSourceName(const std::string & name);

	// Live tuning from the GUI, applied on the worker before the next
	// frame: flow / bgs / tracker values, corridor overrides, YOLO
	// thresholds and the layouts. `enabled`, `detector` and
	// `analysisReduce` are not touched (atomics / startup-only). Changed
	// lane geometry rebuilds the pipeline (background models + tracks).
	void setLiveConfig(const TrackingConfig & config);

	// Any thread. tMs is the frame timestamp (steady clock for live,
	// recording-relative for playback); backwards jumps auto-reset.
	void submitRawJpeg(const uint8_t * data, size_t size, double tMs);

	// Copies out the newest results if newer than lastSeenRevision.
	bool getLatestResults(TrackingResults & out, uint64_t & lastSeenRevision) const;

	// Async: clears background models and all tracks (kept ids stay unique).
	void reset(){ resetRequested = true; }

	// Synchronous (headless / batch) use: no worker thread. setupSync()
	// stores the config; processFrame() decodes + detects + tracks one
	// frame on the calling thread and fills `res`. Never mix with setup().
	void setupSync(const TrackingConfig & config);
	bool processFrame(const uint8_t * data, size_t size, double tMs, TrackingResults & res);
	const TrackingConfig & getConfig() const { return cfg; }

	// Layout selection: a layout whose `recordings` filter matches the
	// source name wins, then the first generic one with the right pane
	// count, else a synthesized full-pane layout (one lane per pane).
	static LayoutConfig findLayout(const std::vector<LayoutConfig> & layouts, int paneCount,
	                               const std::string & sourceName);
	static LayoutConfig fallbackLayout(int paneCount);

private:
	struct LanePipeline {
		LaneConfig lane;
		std::unique_ptr<IDetector> detector;
		cv::Mat homography; // pane px -> lane px
		int laneW = 0, laneH = 0;
	};
	struct CorridorPipeline {
		std::string id;
		std::string label;
		std::vector<int> lanes; // indices into lanes
		std::unique_ptr<MultiObjectTracker> tracker;
		GroupingParams grouping;
	};

	void threadFn();
	void rebuildPipelines(int paneCount, int paneW, int paneH);
	void applyPendingConfig(); // worker thread only
	void applyParams();        // push cfg params into live detectors/trackers
	std::unique_ptr<IDetector> makeDetector(const std::string & corridorId);
	static double nowMs();

	TrackingConfig cfg;

	// live tuning handoff (main -> worker)
	std::mutex configMutex;
	TrackingConfig pendingCfg;
	bool hasPendingCfg = false;
	std::string pendingSource;
	bool hasPendingSource = false;

	std::thread worker;
	std::atomic<bool> running{false};
	std::atomic<bool> enabled{true};
	std::atomic<int> detectorKind{0}; // 0 flow, 1 bgs, 2 yolo
	std::atomic<bool> resetRequested{false};

	// latest-only input slot
	mutable std::mutex inputMutex;
	std::condition_variable inputCv;
	std::vector<uint8_t> pendingJpeg;
	double pendingTMs = 0;
	bool hasPending = false;

	// worker-thread state
	std::vector<LanePipeline> lanes;
	std::vector<CorridorPipeline> corridors;
	std::string layoutName;
	int builtPanes = 0, builtPaneW = 0, builtPaneH = 0;
	int builtKind = -1;
	bool yoloUnavailable = false;
	double lastTMs = -1;

	// published results
	mutable std::mutex resultsMutex;
	TrackingResults results;
	uint64_t resultsRevision = 0;
};

} // namespace tracking
