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
	// --- edge rule: trains and boats come in from a corridor end and leave
	// through one. A track confirms only if it was born clipped at an end or
	// with its box within edgeBirthFrac of one (a track that inherits an id,
	// or one born within edgeGraceMs after a reset, is exempt). Everything
	// that appears in mid corridor — water glitter, a moored boat rocking,
	// a wake fragment, a car glimpsed behind the viaduct — stays tentative.
	bool edgeRule = true;
	float edgeBirthFrac = 0.08f;
	double edgeGraceMs = 2000;
	// A track must be at least this long (corridor fraction) to confirm: a
	// car on the road behind the viaduct is 0.03-0.05 of the corridor and
	// never grows, a train has passed this after a couple of seconds of
	// entering. 0 = off.
	float minConfirmLen = 0.0f;
	// --- model gate (needs ShapeFeedback): with a segmentation model
	// running, a tentative track confirms only once the model has found an
	// instance of the corridor's class on it, and not while the model has
	// called it another class (car / truck behind the viaduct) at least
	// modelGateOtherHits times and more than twice as often as its own.
	bool modelGate = true;
	int modelGateOtherHits = 3;
	// A confirmed object that is leaving (front at the end it heads for)
	// on which the model has seen nothing at all — neither its own class
	// nor another one — for modelLostMs is no longer fed by the motion
	// blobs at that lane end: it coasts out as "occluded". Sun glitter and
	// wake at the lane edge kept a departed boat "exiting" for 45 s
	// otherwise. (A boat the model reads as "surfboard" is still
	// something and keeps being tracked.) 0 = off.
	double modelLostMs = 8000;
	// --- occlusion: a confirmed object that was moving and loses its
	// detections is not dead, it is hidden (behind another train, a mast,
	// the sun on the water). It coasts at its last velocity, size frozen,
	// phase "occluded", until a same-direction detection overlaps its
	// prediction (same id) or its trailing edge has passed the far corridor
	// end. occludedMaxMs is the safety cap; the association gate widens by
	// occludedGateGrow per second hidden.
	bool coastOccluded = true;
	double occludedMaxMs = 30000;
	float occludedGateGrow = 0.05f;
	// --- shape-driven box (needs ShapeFeedback): while a model silhouette
	// no older than shapeBoxMaxAge frames exists, its bounding box trims
	// the motion blob — the hull without the wake the closing glued to it,
	// the roof without the dilated foreground. The blob stays the outer
	// bound and the anchor (the silhouette is measured in the box's own
	// frame, so on its own it could only ever confirm that box, and creep
	// inward wherever the mask falls short of the crop border): a
	// silhouette edge counts only inside the blob and may take at most
	// shapeBoxTrim of the blob's length / height off each end. An edge the
	// blob has at a corridor end stays pinned there.
	bool shapeBox = true;
	int shapeBoxMaxAge = 6;
	float shapeBoxTrim = 0.25f;
	// --- hidden rear / front: two trains meeting on the viaduct, the far
	// one partly behind the near one. An object first seen in mid corridor
	// whose rear side is covered by an oncoming (visible, confirmed) object
	// did not appear out of nowhere — it came in behind that object. Its
	// rear is pinned at the entry end it must have come from, and a pinned
	// rear stays pinned while the oncoming object still covers the spot
	// where the blob's rear edge is: the blob ends where the occluder
	// begins, not where the object does. Likewise a blob front edge that
	// lies inside an oncoming object's box is ignored and the nose coasts
	// at v instead of being pushed back by the other train's tail. A
	// detection right next to a confirmed same-direction object is a
	// fragment of it and never gets a hidden rear.
	bool hiddenRear = true;
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
// the corridor are kept as "stopped", objects that vanish while moving coast
// as "occluded" (see TrackerConfig) instead of dying.
//
// Lifecycle:  tentative --(edge rule, model gate, length, confirmFrames)-->
// confirmed --(detections lost while moving)--> occluded --(re-detected)-->
// confirmed, or --(trailing edge past the far end)--> gone. A tentative
// track that fails a gate simply never confirms and dies with its
// detections; it is never published.
//
// All units are corridor-normalized (see TrackingTypes.h).
class MultiObjectTracker {
public:
	explicit MultiObjectTracker(const TrackerConfig & config);

	void setConfig(const TrackerConfig & config){ cfg = config; }

	// detections: this frame's (already grouped) corridor detections;
	// tMs: frame timestamp; feedback: per track id, what the shape
	// estimator saw on the previous frame (optional).
	void update(const std::vector<Detection> & detections, double tMs,
	            const std::map<int, ShapeFeedback> * feedback = nullptr);

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
		double stoppedSinceMs = -1;  // >= 0 while held as "stopped"
		double occludedSinceMs = -1; // >= 0 while coasting hidden
		bool isShadow = false;  // flagged as a fragment/wake once: never confirms
		bool bornByTouch = false;   // born clipped at a corridor end
		bool hiddenRearChecked = false; // hidden-rear decision taken (once, when the direction settled)
		int clipVotesL = 0, clipVotesR = 0; // consecutive frames the detection disagreed with clippedLeft/Right
		ShapeFeedback fb;       // newest feedback for this id
		double modelSeenMs = -1; // when the model last saw anything (own or other class) on this track
		int modelOtherSeen = 0;  // otherClassHits at that time
	};
	struct Grave {
		TrackedObject obj;
		double tMs = 0;
	};

	Track makeTrack(const Detection & d, double tMs);
	void predict(Track & t, double dtSec);
	void correct(Track & t, const Detection & d, double dtSec, bool holdRear, bool frontHidden);
	void refreshDerived(Track & t);
	// Is the measurement's edge on `side` (+1: x1, -1: x0) inside the box
	// of an oncoming, visible, confirmed object that continues past it?
	bool edgeCovered(int heading, int side, const Detection & m, int selfId) const;
	// Is a confirmed same-direction object right next to the measurement?
	bool nearSameDirection(int heading, const Detection & m, int selfId) const;
	bool sliverInOncoming(int heading, const Detection & m, int selfId) const;
	void mergeInto(Track & keep, const Track & gone);
	void adoptInto(Track & occluded, const Track & fresh);
	int reacquireId(const Detection & d, double tMs, TrackedObject * prior);
	int headingOf(const Track & t) const;
	bool leaving(const Track & t) const;
	bool pastFarEnd(const Track & t) const;
	static void colorForId(int id, float & r, float & g, float & b);

	TrackerConfig cfg;
	std::vector<Track> tracks;
	std::vector<Grave> graveyard;
	std::map<std::pair<int, int>, int> mergeCounts; // (idA,idB) -> consecutive qualifying frames
	int nextId = 1;
	double lastTMs = -1;
	double resetTMs = -1; // time of the last (re)start, for the edge-rule grace
};

} // namespace tracking
