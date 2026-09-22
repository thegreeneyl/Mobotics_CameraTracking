#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

// Plain data types shared by the tracking pipeline.
//
// Coordinate convention: everything is normalized by the module (pane)
// WIDTH. x runs 0..1 across the module; y runs 0..paneH/paneW (≈0.28 for a
// 1920x540 half-res pane). Using one divisor for both axes keeps distances,
// velocities and the Kalman state isotropic — a velocity of 0.1 means
// "one tenth of the camera width per second" in any direction.
//
// No UI / OF dependencies (same rule as src/mobotix/), so
// YOUniverse_Steuerung can reuse this later.
namespace tracking {

// One detector hit in a single analysis frame.
struct Detection {
	float x = 0, y = 0; // bbox centroid, width-normalized
	float w = 0, h = 0; // bbox size, width-normalized
	float confidence = 1.0f;
	std::string label; // empty = unknown
};

// One tracked object with a stable unique id.
struct TrackedObject {
	int id = 0;
	std::string label; // majority vote over the track's detections
	float x = 0, y = 0;   // filtered position, width-normalized
	float vx = 0, vy = 0; // velocity, module-widths per second
	float w = 0, h = 0;   // smoothed bbox size, width-normalized
	int hits = 0;         // matched detections so far
	int misses = 0;       // consecutive frames without a match
	int ageFrames = 0;
	bool confirmed = false; // survived confirmFrames consecutive hits
	float colorR = 1, colorG = 1, colorB = 1; // stable per-id debug color
	std::vector<std::pair<float, float>> trail; // recent positions, newest last
};

struct ModuleResult {
	std::vector<TrackedObject> objects;
};

// Snapshot published by the tracking thread after each analysed frame.
struct TrackingResults {
	std::vector<ModuleResult> modules; // one entry per optical module pane
	double frameTMs = 0;  // timestamp of the analysed frame
	double decodeMs = 0;  // analysis decode cost
	double detectMs = 0;  // detector cost (all panes)
	double trackMs = 0;   // tracker cost (all panes)
	int analysisW = 0;    // per-pane analysis resolution (px)
	int analysisH = 0;
	std::string detectorName; // "bgs" | "yolo"
	uint64_t revision = 0;    // increments per published frame
};

} // namespace tracking
