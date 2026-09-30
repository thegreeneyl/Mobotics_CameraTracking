#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/video/tracking.hpp>

#include "TrackingTypes.h"

namespace tracking {

struct TrackerConfig {
	// lifecycle
	int confirmFrames = 4;       // hits before a track counts as confirmed
	int maxMisses = 20;          // confirmed track coasts this many frames without a match
	int tentativeMaxMisses = 3;  // unconfirmed tracks die faster
	float minSpeedConfirm = 0.004f; // a confirmed object must move (units/s)
	// association (corridor units)
	float gateGapFrac = 0.10f;   // max along-axis gap predicted box <-> detection (+2*|v|*dt)
	float gateLateral = 0.20f;   // min lateral overlap fraction
	float velAngleDeg = 80.0f;   // direction veto between track and detection
	float velScale = 0.10f;      // |dv| normalisation in the cost
	// one-to-many absorption: fragments inside the expanded predicted box
	float absorbGapFrac = 0.15f;
	float absorbLateral = 0.4f;  // min lateral overlap for a fragment (stricter than the gate)
	// a tentative track travelling right next to a confirmed one in the same
	// direction is a fragment / wake of that object: never confirm it.
	// Reach of the check: along-axis gap <= shadowGapFrac, lateral gap
	// <= shadowLateral x the confirmed object's height (0 = must touch).
	// A boat's wake spreads far behind and beside the hull, so the boat
	// corridor uses much larger values than trains.
	bool shadowSuppress = true;
	float shadowGapFrac = 0.15f;
	float shadowLateral = 0.25f;
	// stopped objects: a confirmed, unclipped track that loses its
	// detections at low speed (a train halting at the platform) is held in
	// place with v = 0 and phase "stopped" for up to stoppedHoldMs so it
	// keeps its id when it moves again
	float stopSpeed = 0.04f;
	double stoppedHoldMs = 60000;
	float stopEdgeFrac = 0.15f;  // no "stopped" this close to the corridor end it heads for: it left
	// only an object at least this long (corridor fraction) can halt; a small
	// blob that vanishes (a car behind the viaduct, a glitter patch) is gone
	float stopMinWidthFrac = 0.15f;
	// track-track merge
	float mergeGapFrac = 0.10f;
	float mergeVelTol = 0.03f;   // |dv| (units/s)
	int mergeFrames = 6;         // consecutive frames the pair must qualify
	// id inheritance for a track that reappears where a dead one was heading
	double reacquireMs = 2000;
	float reacquireGapFrac = 0.15f;
	// axis lock (trains): vy is forced to 0, only speed along the corridor
	bool lockAxis = false;
	// size constancy: a confirmed object's detection shorter than
	// sizeJumpTrust x (or longer than 1/sizeJumpTrust x) the predicted box
	// is a fragment / inflated blob — its edges get sizeJumpNoiseGain x the
	// position noise so the box follows only if the change persists
	float sizeJumpTrust = 0.6f;
	float sizeJumpNoiseGain = 25.0f;
	// edge pin/release hysteresis: the detection must disagree with the
	// current clipped state for this many consecutive frames before an edge
	// is pinned to / released from the corridor end. A pin/release re-seeds
	// the edge outside the filter, so a flickering mask at the object's end
	// would otherwise make the box jump by the lost part every frame.
	int clipFrames = 3;
	// Kalman noise
	float posMeasNoise = 1.0e-4f;    // (1% corridor)^2
	float velMeasNoise = 6.0e-4f;    // flow velocity measurement
	float posProcNoise = 4.0e-6f;    // per second
	float velProcNoise = 3.0e-3f;    // per second
	float clippedMeasNoise = 1.0e-7f; // a pinned edge is known exactly
	// misc
	int trailLen = 30;           // debug trail length (frames)
	double maxDtSec = 5.0;       // larger frame gaps reset the tracker
};

// Corridor tracker. One constant-velocity Kalman filter per track on the
// bbox EDGES [x0, x1, y0, y1] plus [vx, vy]; velocity is measured directly
// from the detector's optical flow, so it stays right while a long object is
// still entering (box growing) or spans the whole corridor (box pinned).
// An edge that touches a corridor end is "clipped": it is held there and
// decoupled from the velocity in the transition until the object's real end
// enters.
//
// Association: cost on along-axis gap, IoU and velocity agreement with a
// direction veto; then one-to-many absorption (fragments in the expanded
// predicted box join the track's measurement), track-track merging (two
// tracks that travel together for mergeFrames become one, older id wins)
// and id inheritance (a track born where a recently dead one was heading
// reuses that id). Ids are never reused otherwise. Objects that halt inside
// the corridor are kept as "stopped" (see TrackerConfig) instead of dying.
//
// All units are corridor-normalized (see TrackingTypes.h).
class MultiObjectTracker {
public:
	explicit MultiObjectTracker(const TrackerConfig & config);

	void setConfig(const TrackerConfig & config){ cfg = config; }

	// detections: this frame's (already grouped) corridor detections;
	// tMs: frame timestamp.
	void update(const std::vector<Detection> & detections, double tMs);

	// All live tracks (tentative ones flagged via TrackedObject::confirmed).
	std::vector<TrackedObject> getObjects() const;

	void reset();

private:
	struct Track {
		cv::KalmanFilter kf;
		TrackedObject obj;
		int dirSign = 0;        // +1 / -1 dominant direction, 0 unknown
		int dirVotes = 0;       // consecutive agreeing frames
		float lastMatchVx = 0, lastMatchVy = 0; // velocity at the last matched frame
		double stoppedSinceMs = -1; // >= 0 while held as "stopped"
		bool isShadow = false;  // flagged as a fragment/wake once: never confirms
		int clipVotesL = 0, clipVotesR = 0; // consecutive frames the detection disagreed with clippedLeft/Right
	};
	struct Grave {
		TrackedObject obj;
		double tMs = 0;
	};

	Track makeTrack(const Detection & d, double tMs);
	void predict(Track & t, double dtSec);
	void correct(Track & t, const Detection & d, double dtSec);
	void refreshDerived(Track & t);
	void mergeInto(Track & keep, const Track & gone);
	int reacquireId(const Detection & d, double tMs, TrackedObject * prior);
	static void colorForId(int id, float & r, float & g, float & b);

	TrackerConfig cfg;
	std::vector<Track> tracks;
	std::vector<Grave> graveyard;
	std::map<std::pair<int, int>, int> mergeCounts; // (idA,idB) -> consecutive qualifying frames
	int nextId = 1;
	double lastTMs = -1;
};

} // namespace tracking
