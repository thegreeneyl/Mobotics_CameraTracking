#pragma once

#include <functional>
#include <string>

#include "ofJson.h"

#include "TrackingManager.h"

// JSON <-> TrackingConfig for the config document. Shared by the session
// (load + save) and the headless batch runner so both run the exact same
// configuration.
//
// Document shape (bin/data/config/config.json):
//   cameras      — read/written by CameraTrackingSession
//   recordingsDir — relative to the data path
//   display      — {decodeScale, arrowScale}: GUI-side only
//   tracking     — everything below (TrackingConfig)
namespace tracking {

struct LoadedTrackingConfig {
	TrackingConfig config;
	std::string recordingsDir = "recordings";
	int displayDecodeScale = 2;      // display.decodeScale: 1 full, 2 DCT half
	float displayArrowScale = 1.0f;  // display.arrowScale: overlay velocity arrows
};

// Parses recordingsDir, display and tracking. resolvePath turns the
// relative shape-model path into an absolute one (ofToDataPath in the
// app). Older documents are still read: `tracking.overlay` (arrowScale,
// trailFrames) is folded into display.arrowScale / tracker.trailLen, and
// `tracking.detector`, `tracking.bgs`, `tracking.yolo` are ignored. None of
// those keys is ever written again.
LoadedTrackingConfig loadTrackingConfig(const ofJson & root,
                                        const std::function<std::string(const std::string &)> & resolvePath);

// The complete `tracking` object for cfg: enabled, analysisReduce, lane
// size limits, lens, flow, tracker, shape (+model), smoothing, canvas,
// corridors (placement + overrides) and layouts (lane anchors).
ofJson trackingJson(const TrackingConfig & cfg);

// {decodeScale, arrowScale}
ofJson displayJson(int decodeScale, float arrowScale);

} // namespace tracking
