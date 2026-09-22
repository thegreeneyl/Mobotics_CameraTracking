#pragma once

#include <map>
#include <string>
#include <vector>

#include <opencv2/video/tracking.hpp>

#include "TrackingTypes.h"

namespace tracking {

struct TrackerConfig {
	int confirmFrames = 5;  // hits before a track counts as confirmed
	int maxMisses = 12;     // confirmed track dies after this many misses
	int tentativeMaxMisses = 2; // unconfirmed tracks die faster
	float minIoU = 0.1f;    // first association pass gate
	float maxDist = 0.05f;  // second pass: centroid gate (module widths)
	int trailLen = 30;      // debug trail length (frames)
	float bboxSmoothing = 0.3f; // EMA alpha for bbox w/h
	double maxDtSec = 5.0;  // larger frame gaps reset the tracker
};

// SORT-style multi-object tracker: one constant-velocity Kalman filter per
// track, greedy IoU association with a centroid-distance fallback, and a
// tentative -> confirmed -> dead lifecycle that suppresses one-off noise
// (water glint) while keeping IDs stable through short occlusions.
// All units are width-normalized (see TrackingTypes.h); velocity is in
// module-widths per second, derived from real frame timestamps so live and
// playback behave identically.
class MultiObjectTracker {
public:
	explicit MultiObjectTracker(const TrackerConfig & config);

	// detections: current frame's detector output; tMs: frame timestamp.
	void update(const std::vector<Detection> & detections, double tMs);

	// All live tracks (tentative ones flagged via TrackedObject::confirmed).
	std::vector<TrackedObject> getObjects() const;

	void reset();

private:
	struct Track {
		cv::KalmanFilter kf;
		TrackedObject obj;
		std::map<std::string, int> labelVotes;
	};

	Track makeTrack(const Detection & d);
	static float iou(float ax, float ay, float aw, float ah,
	                 float bx, float by, float bw, float bh);
	static void colorForId(int id, float & r, float & g, float & b);

	TrackerConfig cfg;
	std::vector<Track> tracks;
	int nextId = 1;
	double lastTMs = -1;
};

} // namespace tracking
