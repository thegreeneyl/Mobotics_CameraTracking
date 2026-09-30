#pragma once

#include <string>

#include "ofJson.h"

#include "TrackingManager.h"

// JSON <-> TrackingConfig for the "tracking" section of config.json. Shared
// by the GUI app (load + save-button) and the headless batch runner so both
// run the exact same configuration.
namespace tracking {

struct LoadedTrackingConfig {
	TrackingConfig config;
	float overlayArrowScale = 1.0f;
	std::string recordingsDir = "recordings"; // relative to data path
	int displayDecodeScale = 2;
};

// Parses the whole config.json document (recordingsDir, display, tracking).
// modelPathResolver turns the relative YOLO model path into an absolute one
// (ofToDataPath in the app).
LoadedTrackingConfig loadTrackingConfig(const ofJson & root,
                                        const std::function<std::string(const std::string &)> & resolvePath);

// Writes the live-tunable values (detector params, tracker, overlay, lanes)
// into root["tracking"], leaving all other keys untouched.
void writeTrackingTuning(ofJson & root, const TrackingConfig & cfg, float overlayArrowScale);

} // namespace tracking
