#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "BgsDetector.h"
#include "MultiObjectTracker.h"
#include "TrackingTypes.h"
#include "YoloDetector.h"

namespace tracking {

struct TrackingConfig {
	bool enabled = true;
	std::string detector = "bgs"; // "bgs" | "yolo"
	// DCT-domain reduction of the source JPEG for analysis (1|2|4|8).
	// 2 => a 3840x1080 frame is analysed at 1920x540 (960x540 per module).
	int analysisReduce = 2;
	BgsConfig bgs;
	TrackerConfig tracker;
	YoloConfig yolo;
	std::vector<std::vector<Zone>> moduleZones; // [module] -> class-tag zones
};

// Owns the tracking worker thread. Input: raw JPEG bytes tapped from the
// live client or the playback player (latest-only slot — stale frames are
// dropped, never queued, so a slow detector can never stall the sources or
// the UI). The worker does its OWN reduced decode (grayscale for BGS, color
// for YOLO), splits the frame into module panes (side-by-side "BOTH" frames
// are detected by aspect ratio: width >= 2*height), runs detector + tracker
// per pane, and publishes a TrackingResults snapshot.
//
// No UI dependencies (same rule as src/mobotix/).
class TrackingManager {
public:
	~TrackingManager();

	void setup(const TrackingConfig & config); // starts the worker thread
	void stop();

	void setEnabled(bool e){ enabled = e; }
	bool isEnabled() const { return enabled; }

	// "bgs" or "yolo". YOLO is loaded lazily on the worker thread; if the
	// model cannot be loaded it logs a warning and falls back to BGS.
	void setDetectorType(const std::string & type);
	std::string getDetectorType() const;

	// Any thread. tMs is the frame timestamp (steady clock for live,
	// recording-relative for playback); backwards jumps auto-reset.
	void submitRawJpeg(const uint8_t * data, size_t size, double tMs);

	// Copies out the newest results if newer than lastSeenRevision.
	bool getLatestResults(TrackingResults & out, uint64_t & lastSeenRevision) const;

	// Async: clears background models and all tracks (kept ids stay unique).
	void reset(){ resetRequested = true; }

private:
	struct Pipeline {
		std::unique_ptr<IDetector> detector;
		std::unique_ptr<MultiObjectTracker> tracker;
	};

	void threadFn();
	void rebuildPipelines(int paneCount, bool useYolo);
	static double nowMs();

	TrackingConfig cfg;

	std::thread worker;
	std::atomic<bool> running{false};
	std::atomic<bool> enabled{true};
	std::atomic<bool> useYolo{false};
	std::atomic<bool> resetRequested{false};

	// latest-only input slot
	mutable std::mutex inputMutex;
	std::condition_variable inputCv;
	std::vector<uint8_t> pendingJpeg;
	double pendingTMs = 0;
	bool hasPending = false;

	// worker-thread state
	std::vector<Pipeline> pipelines;
	bool pipelinesAreYolo = false;
	double lastTMs = -1;

	// published results
	mutable std::mutex resultsMutex;
	TrackingResults results;
	uint64_t resultsRevision = 0;
};

} // namespace tracking
