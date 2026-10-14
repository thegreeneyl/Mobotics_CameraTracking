#include "MultiObjectTracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "DetectionGrouping.h"

namespace tracking {

namespace {
// state indices
constexpr int X0 = 0, X1 = 1, Y0 = 2, Y1 = 3, VX = 4, VY = 5;
constexpr int kStates = 6, kMeas = 6;

float iou(const TrackedObject & o, const Detection & d){
	const float ix = std::max(0.0f, std::min(o.x1, d.x1) - std::max(o.x0, d.x0));
	const float iy = std::max(0.0f, std::min(o.y1, d.y1) - std::max(o.y0, d.y0));
	const float inter = ix * iy;
	const float uni = (o.x1 - o.x0) * (o.y1 - o.y0) + d.w() * d.h() - inter;
	return uni <= 0 ? 0.0f : inter / uni;
}

Detection asDetection(const TrackedObject & o){
	Detection d;
	d.x0 = o.x0; d.x1 = o.x1; d.y0 = o.y0; d.y1 = o.y1;
	d.vx = o.vx; d.vy = o.vy;
	d.hasVelocity = true;
	d.area = (o.x1 - o.x0) * (o.y1 - o.y0);
	return d;
}

int signOf(float v, float eps){
	return v > eps ? 1 : v < -eps ? -1 : 0;
}
} // namespace

MultiObjectTracker::MultiObjectTracker(const TrackerConfig & config) : cfg(config){}

void MultiObjectTracker::reset(){
	tracks.clear();
	graveyard.clear();
	mergeCounts.clear();
	lastTMs = -1;
	resetTMs = -1;
	// nextId is NOT reset: ids stay unique across resets so a re-detected
	// object after a source switch never reuses a previous id.
}

// Stable, well-separated debug color per id (golden-ratio hue walk).
void MultiObjectTracker::colorForId(int id, float & r, float & g, float & b){
	const float h = std::fmod(static_cast<float>(id) * 0.61803398875f, 1.0f) * 6.0f;
	const float x = 1.0f - std::fabs(std::fmod(h, 2.0f) - 1.0f);
	switch(static_cast<int>(h)){
		case 0: r = 1; g = x; b = 0; break;
		case 1: r = x; g = 1; b = 0; break;
		case 2: r = 0; g = 1; b = x; break;
		case 3: r = 0; g = x; b = 1; break;
		case 4: r = x; g = 0; b = 1; break;
		default: r = 1; g = 0; b = x; break;
	}
}

// Direction the object travels in: the filtered velocity while it is seen,
// the velocity of the last match while it is hidden or stopped.
int MultiObjectTracker::headingOf(const Track & t) const {
	const int live = signOf(t.obj.vx, cfg.minSpeedConfirm);
	if(live != 0 && t.obj.misses == 0) return live;
	const int last = signOf(t.lastMatchVx, cfg.minSpeedConfirm);
	return last != 0 ? last : (t.dirSign != 0 ? t.dirSign : live);
}

// Is the measurement's edge on `side` (+1: x1, -1: x0) inside the box of
// an oncoming object that is confirmed and being seen right now, with that
// object continuing past the edge? Then the blob ends where the oncoming
// object begins, not where this one does: on the far track of the viaduct
// the near train hides whatever is behind it.
bool MultiObjectTracker::edgeCovered(int heading, int side, const Detection & m, int selfId) const {
	if(!cfg.hiddenRear || heading == 0 || side == 0) return false;
	const float x = side > 0 ? m.x1 : m.x0;
	for(const auto & o : tracks){
		const TrackedObject & ob = o.obj;
		if(ob.id == selfId || !ob.confirmed || ob.misses > 0) continue;
		if(o.stoppedSinceMs >= 0 || o.occludedSinceMs >= 0) continue;
		if(signOf(ob.vx, cfg.minSpeedConfirm) != -heading) continue; // oncoming only
		if(lateralOverlapFrac(ob.y0, ob.y1, m.y0, m.y1) < cfg.gateLateral) continue;
		if(x < ob.x0 - cfg.mergeGapFrac || x > ob.x1 + cfg.mergeGapFrac) continue;
		const bool beyond = side > 0 ? ob.x1 > x : ob.x0 < x;
		if(beyond) return true;
	}
	return false;
}

// Is there a confirmed object travelling the same way right next to this
// measurement? Then it is a fragment of that object, not a new one.
bool MultiObjectTracker::nearSameDirection(int heading, const Detection & m, int selfId) const {
	for(const auto & o : tracks){
		const TrackedObject & ob = o.obj;
		if(ob.id == selfId || !ob.confirmed) continue;
		if(signOf(ob.vx, cfg.minSpeedConfirm) != heading) continue;
		if(lateralOverlapFrac(ob.y0, ob.y1, m.y0, m.y1) < cfg.gateLateral) continue;
		if(gapX(ob.x0, ob.x1, m.x0, m.x1) <= cfg.mergeGapFrac) return true;
	}
	return false;
}

// A short blob sitting entirely inside an oncoming confirmed box is a
// piece of that object (or its wake on the other camera), not a second
// object coming in behind it. Used only when deciding to pin a hidden
// rear — not while holding an already-pinned one.
bool MultiObjectTracker::sliverInOncoming(int heading, const Detection & m, int selfId) const {
	if(heading == 0) return false;
	const float mw = m.x1 - m.x0;
	for(const auto & o : tracks){
		const TrackedObject & ob = o.obj;
		if(ob.id == selfId || !ob.confirmed) continue;
		if(signOf(ob.vx, cfg.minSpeedConfirm) != -heading) continue;
		if(lateralOverlapFrac(ob.y0, ob.y1, m.y0, m.y1) < cfg.gateLateral) continue;
		const float ow = std::max(1e-4f, ob.x1 - ob.x0);
		if(m.x0 >= ob.x0 - cfg.mergeGapFrac && m.x1 <= ob.x1 + cfg.mergeGapFrac
		   && mw < 0.25f * ow) return true;
	}
	return false;
}

// Is the object on its way out — front pinned at the end it heads for, or
// within stopEdgeFrac of it? A hidden object that is leaving just coasts
// out; it is never resurrected by whatever appears at that end (the water
// glitter at the lane edge, a car glimpsed behind the trees).
bool MultiObjectTracker::leaving(const Track & t) const {
	const int h = headingOf(t);
	if(h > 0) return t.obj.clippedRight || t.obj.x1 >= 1.0f - cfg.stopEdgeFrac;
	if(h < 0) return t.obj.clippedLeft || t.obj.x0 <= cfg.stopEdgeFrac;
	return false;
}

// Has the trailing edge of a coasting object left the corridor?
bool MultiObjectTracker::pastFarEnd(const Track & t) const {
	const int h = headingOf(t);
	if(h > 0) return t.obj.x0 >= 1.0f;
	if(h < 0) return t.obj.x1 <= 0.0f;
	return false;
}

// ------------------------------------------------------------------ tracks

MultiObjectTracker::Track MultiObjectTracker::makeTrack(const Detection & d, double tMs){
	Track t;
	t.kf.init(kStates, kMeas, 0, CV_32F);
	cv::setIdentity(t.kf.transitionMatrix);
	cv::setIdentity(t.kf.measurementMatrix);
	cv::setIdentity(t.kf.processNoiseCov, cv::Scalar::all(cfg.posProcNoise));
	cv::setIdentity(t.kf.measurementNoiseCov, cv::Scalar::all(cfg.posMeasNoise));
	cv::setIdentity(t.kf.errorCovPost, cv::Scalar::all(1e-3));
	t.kf.errorCovPost.at<float>(VX, VX) = 1e-2f;
	t.kf.errorCovPost.at<float>(VY, VY) = 1e-2f;

	TrackedObject prior;
	const int inherited = reacquireId(d, tMs, &prior);
	const float vx = d.hasVelocity ? d.vx : (inherited ? prior.vx : 0.0f);
	const float vy = cfg.lockAxis ? 0.0f : (d.hasVelocity ? d.vy : (inherited ? prior.vy : 0.0f));
	t.kf.statePost = (cv::Mat_<float>(kStates, 1) << d.x0, d.x1, d.y0, d.y1, vx, vy);

	t.obj.id = inherited ? inherited : nextId++;
	if(inherited){
		t.obj.confirmed = true;
		t.obj.hits = cfg.confirmFrames;
		t.dirSign = signOf(prior.vx, cfg.minSpeedConfirm);
		t.dirVotes = cfg.confirmFrames;
	}else{
		t.obj.hits = 1;
	}
	t.obj.clippedLeft = d.touchesLeft;
	t.obj.clippedRight = d.touchesRight;
	t.lastMatchVx = vx;
	t.lastMatchVy = vy;
	// edge rule bookkeeping: where was this object first seen?
	t.bornByTouch = d.touchesLeft || d.touchesRight;
	const bool nearEnd = d.x0 <= cfg.edgeBirthFrac || d.x1 >= 1.0f - cfg.edgeBirthFrac;
	const bool grace = resetTMs >= 0 && tMs - resetTMs <= cfg.edgeGraceMs;
	t.obj.edgeBorn = inherited != 0 || t.bornByTouch || nearEnd || grace;
	colorForId(t.obj.id, t.obj.colorR, t.obj.colorG, t.obj.colorB);
	refreshDerived(t);
	t.obj.trail.emplace_back(t.obj.x, t.obj.y);
	return t;
}

// Reads the Kalman state into the published object fields.
void MultiObjectTracker::refreshDerived(Track & t){
	cv::Mat & s = t.kf.statePost;
	float x0 = s.at<float>(X0), x1 = s.at<float>(X1);
	float y0 = s.at<float>(Y0), y1 = s.at<float>(Y1);
	if(x1 < x0) std::swap(x0, x1);
	if(y1 < y0) std::swap(y0, y1);
	s.at<float>(X0) = x0; s.at<float>(X1) = x1;
	s.at<float>(Y0) = y0; s.at<float>(Y1) = y1;
	if(cfg.lockAxis) s.at<float>(VY) = 0.0f;
	t.obj.x0 = x0; t.obj.x1 = x1; t.obj.y0 = y0; t.obj.y1 = y1;
	t.obj.x = (x0 + x1) * 0.5f;
	t.obj.y = (y0 + y1) * 0.5f;
	t.obj.w = x1 - x0;
	t.obj.h = y1 - y0;
	t.obj.vx = s.at<float>(VX);
	t.obj.vy = s.at<float>(VY);

	// phase from clipping + travel direction
	if(t.stoppedSinceMs >= 0){
		t.obj.phase = "stopped";
		return;
	}
	if(t.occludedSinceMs >= 0){
		t.obj.phase = "occluded";
		return;
	}
	const int dir = signOf(t.obj.vx, cfg.minSpeedConfirm);
	const bool rearClipped = dir > 0 ? t.obj.clippedLeft : dir < 0 ? t.obj.clippedRight : false;
	const bool frontClipped = dir > 0 ? t.obj.clippedRight : dir < 0 ? t.obj.clippedLeft : false;
	if(t.obj.clippedLeft && t.obj.clippedRight) t.obj.phase = "crossing";
	else if(rearClipped) t.obj.phase = "entering";
	else if(frontClipped) t.obj.phase = "exiting";
	else t.obj.phase = "moving";
}

void MultiObjectTracker::predict(Track & t, double dtSec){
	const float dt = static_cast<float>(dtSec);
	cv::Mat & F = t.kf.transitionMatrix;
	cv::setIdentity(F);
	// A clipped edge sits at the corridor end: it does not travel with v.
	// While an object coasts hidden, its front may be pinned at the exit
	// end (it was leaving): the rear keeps moving, the box shrinks and the
	// object is gone once the rear has passed the end as well.
	F.at<float>(X0, VX) = t.obj.clippedLeft ? 0.0f : dt;
	F.at<float>(X1, VX) = t.obj.clippedRight ? 0.0f : dt;
	F.at<float>(Y0, VY) = dt;
	F.at<float>(Y1, VY) = dt;
	cv::Mat & Q = t.kf.processNoiseCov;
	cv::setIdentity(Q, cv::Scalar::all(cfg.posProcNoise * std::max(dt, 1e-3f)));
	Q.at<float>(VX, VX) = cfg.velProcNoise * std::max(dt, 1e-3f);
	Q.at<float>(VY, VY) = cfg.velProcNoise * std::max(dt, 1e-3f);
	t.kf.predict();
	t.kf.statePost = t.kf.statePre.clone();
	t.kf.errorCovPost = t.kf.errorCovPre.clone();
	refreshDerived(t);
	t.obj.ageFrames++;
}

void MultiObjectTracker::correct(Track & t, const Detection & d, double dtSec, bool holdRear, bool frontHidden){
	(void)dtSec;
	cv::Mat & R = t.kf.measurementNoiseCov;
	cv::setIdentity(R, cv::Scalar::all(cfg.posMeasNoise));
	const cv::Mat & pre = t.kf.statePre;

	// Shape-driven box: a fresh model silhouette trims the blob (the hull,
	// not the wake glued to it by the closing; the roof, not the dilated
	// foreground). The blob stays the outer bound and the anchor: the
	// silhouette is measured in the box's own frame, so left to itself it
	// would only ever reproduce the box it was measured in, and creep
	// inward wherever the mask falls short of the crop border — a corridor
	// long train once lost two thirds of its box that way. A silhouette
	// edge therefore counts only inside the blob and may take at most
	// shapeBoxTrim of the blob's extent off each end. An edge the blob has
	// at the corridor end is pinned there regardless of the silhouette (the
	// padded frame reaches past the end and says nothing about the object).
	const ShapeFeedback & fb = t.fb;
	const bool useShape = cfg.shapeBox && t.obj.confirmed && fb.boxValid
	                      && fb.modelAge <= cfg.shapeBoxMaxAge
	                      && fb.bx1 > fb.bx0 && fb.by1 > fb.by0;
	const bool blobL = d.touchesLeft, blobR = d.touchesRight;
	float mx0 = d.x0, mx1 = d.x1, my0 = d.y0, my1 = d.y1;
	if(useShape){
		const float trimX = cfg.shapeBoxTrim * std::max(0.0f, d.x1 - d.x0);
		const float trimY = cfg.shapeBoxTrim * std::max(0.0f, d.y1 - d.y0);
		if(!blobL) mx0 = std::clamp(fb.bx0, d.x0, d.x0 + trimX);
		if(!blobR) mx1 = std::clamp(fb.bx1, d.x1 - trimX, d.x1);
		my0 = std::clamp(fb.by0, d.y0, d.y0 + trimY);
		my1 = std::clamp(fb.by1, d.y1 - trimY, d.y1);
	}
	const float measW = std::max(1e-4f, mx1 - mx0);

	// A confirmed object does not change length from one frame to the
	// next. A detection much shorter than the predicted box is a fragment
	// (sparkling water / occlusion ate part of the hull), one much longer
	// has picked up a wake or a neighbour: its box edges are weak evidence
	// and must not collapse or inflate the box in one step. Such a
	// fragment can also not release a pinned edge — the object's end has
	// not entered the lane, the detector merely lost sight of it.
	const float predW = std::max(1e-3f, pre.at<float>(X1) - pre.at<float>(X0));
	const float ratio = measW / predW;
	const bool fragment = t.obj.confirmed && ratio < cfg.sizeJumpTrust;
	const bool inflated = t.obj.confirmed && !t.obj.clippedLeft && !t.obj.clippedRight
	                      && !d.touchesLeft && !d.touchesRight && ratio > 1.0f / cfg.sizeJumpTrust;
	if(fragment || inflated){
		for(int i = X0; i <= X1; i++) R.at<float>(i, i) = cfg.posMeasNoise * cfg.sizeJumpNoiseGain;
	}
	// Hidden rear / hidden front (see TrackerConfig::hiddenRear): a pinned
	// rear stays pinned while an oncoming object covers the spot where the
	// blob's rear edge is; a front edge that lies inside an oncoming
	// object's box is where that object begins, not where this one's nose
	// is — the measurement is ignored and the nose coasts at v (unless the
	// blob reaches the lane end there, then it is pinned as usual).
	const int heading = headingOf(t);
	const bool holdL = holdRear && heading > 0 && t.obj.clippedLeft;
	const bool holdR = holdRear && heading < 0 && t.obj.clippedRight;
	if(frontHidden){
		if(heading > 0 && !blobR) R.at<float>(X1, X1) = 1e3f;
		if(heading < 0 && !blobL) R.at<float>(X0, X0) = 1e3f;
	}
	bool touchesLeft = blobL || ((fragment || holdL) && t.obj.clippedLeft);
	bool touchesRight = blobR || ((fragment || holdR) && t.obj.clippedRight);
	// hysteresis: a change of the clipped state must persist for clipFrames
	// frames; until then the current state stands (a pinned edge keeps its
	// place, see zx0/zx1 below)
	const auto settle = [&](bool want, bool current, int & votes){
		if(want == current){ votes = 0; return current; }
		if(++votes >= std::max(1, cfg.clipFrames)){ votes = 0; return want; }
		return current;
	};
	if(t.obj.hits > 1){
		touchesLeft = settle(touchesLeft, t.obj.clippedLeft, t.clipVotesL);
		touchesRight = settle(touchesRight, t.obj.clippedRight, t.clipVotesR);
	}
	// pinned edges are known exactly; the free edge carries the position info
	if(touchesLeft) R.at<float>(X0, X0) = cfg.clippedMeasNoise;
	if(touchesRight) R.at<float>(X1, X1) = cfg.clippedMeasNoise;
	const bool useVel = d.hasVelocity;
	R.at<float>(VX, VX) = useVel ? cfg.velMeasNoise : 1e3f;
	R.at<float>(VY, VY) = useVel ? cfg.velMeasNoise : 1e3f;
	if(cfg.lockAxis) R.at<float>(VY, VY) = cfg.clippedMeasNoise;

	// Edge pin / release transitions are handled outside the Kalman update:
	// the edge state is re-seeded from the measurement and its covariance
	// row/column cleared, so the (large) jump of that edge never leaks into
	// the velocity or the other edges. A released edge (the object's real
	// end entered the corridor) gets a wide prior; a pinned one a tight one.
	cv::Mat & P = t.kf.errorCovPre;
	const auto reseed = [&](int idx, float meas, float var){
		t.kf.statePre.at<float>(idx) = meas;
		t.kf.statePost.at<float>(idx) = meas;
		for(int k = 0; k < kStates; k++){
			P.at<float>(idx, k) = 0.0f;
			P.at<float>(k, idx) = 0.0f;
		}
		P.at<float>(idx, idx) = var;
	};
	if(t.obj.clippedLeft != touchesLeft) reseed(X0, mx0, touchesLeft ? 1e-6f : 1e-2f);
	if(t.obj.clippedRight != touchesRight) reseed(X1, mx1, touchesRight ? 1e-6f : 1e-2f);

	// With a direct flow velocity measurement the velocity is estimated from
	// the flow alone: cut the position<->velocity cross-covariance so a box
	// edge jump (occlusion, merge with a fragment) cannot flip the velocity.
	// Detectors without velocity keep the coupling (velocity from motion).
	if(d.hasVelocity){
		for(int p = X0; p <= Y1; p++){
			for(int v = VX; v <= VY; v++){
				P.at<float>(p, v) = 0.0f;
				P.at<float>(v, p) = 0.0f;
			}
		}
	}

	const float vx = useVel ? d.vx : t.obj.vx;
	const float vy = cfg.lockAxis ? 0.0f : (useVel ? d.vy : t.obj.vy);
	// For the very first frames the velocity prior is weak: seed it hard so
	// the box does not lag behind a fast train while P converges.
	if(t.obj.hits <= 1 && useVel){
		t.kf.statePost.at<float>(VX) = vx;
		t.kf.statePost.at<float>(VY) = vy;
		t.kf.statePre.at<float>(VX) = vx;
		t.kf.statePre.at<float>(VY) = vy;
	}
	// a pinned edge that the measurement could not release stays at the lane end
	const float zx0 = (touchesLeft && !blobL) ? pre.at<float>(X0) : mx0;
	const float zx1 = (touchesRight && !blobR) ? pre.at<float>(X1) : mx1;
	const cv::Mat z = (cv::Mat_<float>(kMeas, 1) << zx0, zx1, my0, my1, vx, vy);
	t.kf.correct(z);
	if(frontHidden){
		// a coasting nose does not leave the corridor
		cv::Mat & s = t.kf.statePost;
		s.at<float>(X0) = std::max(0.0f, s.at<float>(X0));
		s.at<float>(X1) = std::min(1.0f, s.at<float>(X1));
	}
	t.obj.clippedLeft = touchesLeft;
	t.obj.clippedRight = touchesRight;
	t.stoppedSinceMs = -1;  // moving again
	t.occludedSinceMs = -1; // seen again
	refreshDerived(t);
	t.lastMatchVx = t.obj.vx;
	t.lastMatchVy = t.obj.vy;

	// direction consistency (for confirmation and for the axis lock)
	const int s = signOf(t.obj.vx, cfg.minSpeedConfirm);
	if(s != 0){
		if(s == t.dirSign) t.dirVotes++;
		else { t.dirSign = s; t.dirVotes = 1; }
	}
}

void MultiObjectTracker::mergeInto(Track & keep, const Track & gone){
	cv::Mat & s = keep.kf.statePost;
	s.at<float>(X0) = std::min(keep.obj.x0, gone.obj.x0);
	s.at<float>(X1) = std::max(keep.obj.x1, gone.obj.x1);
	s.at<float>(Y0) = std::min(keep.obj.y0, gone.obj.y0);
	s.at<float>(Y1) = std::max(keep.obj.y1, gone.obj.y1);
	const float wa = std::max(keep.obj.w * keep.obj.h, 1e-6f);
	const float wb = std::max(gone.obj.w * gone.obj.h, 1e-6f);
	s.at<float>(VX) = (keep.obj.vx * wa + gone.obj.vx * wb) / (wa + wb);
	s.at<float>(VY) = (keep.obj.vy * wa + gone.obj.vy * wb) / (wa + wb);
	keep.obj.clippedLeft = keep.obj.clippedLeft || gone.obj.clippedLeft;
	keep.obj.clippedRight = keep.obj.clippedRight || gone.obj.clippedRight;
	keep.obj.hits = std::max(keep.obj.hits, gone.obj.hits);
	keep.obj.confirmed = keep.obj.confirmed || gone.obj.confirmed;
	keep.obj.edgeBorn = keep.obj.edgeBorn || gone.obj.edgeBorn;
	// the merged track is as alive as the fresher of the two: a stopped
	// or hidden object that a new track has been riding on is seen again
	if(gone.obj.misses < keep.obj.misses){
		keep.obj.misses = gone.obj.misses;
		// ... with that track's last velocity, if it has one (a track
		// born this frame has none yet — keeping ours beats a zero)
		if(gone.lastMatchVx != 0.0f || gone.lastMatchVy != 0.0f){
			keep.lastMatchVx = gone.lastMatchVx;
			keep.lastMatchVy = gone.lastMatchVy;
		}
	}
	if(keep.obj.misses == 0){
		keep.stoppedSinceMs = -1;
		keep.occludedSinceMs = -1;
	}
	refreshDerived(keep);
}

// A hidden object re-emerges as a fresh tentative track: the coasting
// track takes that track's box and velocity and keeps its id.
void MultiObjectTracker::adoptInto(Track & occ, const Track & fresh){
	cv::Mat & s = occ.kf.statePost;
	s.at<float>(X0) = fresh.obj.x0;
	s.at<float>(X1) = fresh.obj.x1;
	s.at<float>(Y0) = fresh.obj.y0;
	s.at<float>(Y1) = fresh.obj.y1;
	s.at<float>(VX) = fresh.obj.vx;
	s.at<float>(VY) = cfg.lockAxis ? 0.0f : fresh.obj.vy;
	occ.kf.statePre = s.clone();
	cv::setIdentity(occ.kf.errorCovPost, cv::Scalar::all(1e-3));
	occ.kf.errorCovPost.at<float>(VX, VX) = 1e-2f;
	occ.kf.errorCovPost.at<float>(VY, VY) = 1e-2f;
	occ.kf.errorCovPre = occ.kf.errorCovPost.clone();
	occ.obj.clippedLeft = fresh.obj.clippedLeft;
	occ.obj.clippedRight = fresh.obj.clippedRight;
	occ.obj.misses = fresh.obj.misses;
	occ.obj.hits += fresh.obj.hits;
	occ.lastMatchVx = fresh.lastMatchVx;
	occ.lastMatchVy = fresh.lastMatchVy;
	occ.occludedSinceMs = -1;
	occ.stoppedSinceMs = -1;
	occ.clipVotesL = occ.clipVotesR = 0;
	refreshDerived(occ);
}

// A track that died recently and whose predicted position now overlaps the
// new detection (same travel direction) hands its id over.
int MultiObjectTracker::reacquireId(const Detection & d, double tMs, TrackedObject * prior){
	int bestIdx = -1;
	float bestGap = std::numeric_limits<float>::max();
	for(size_t i = 0; i < graveyard.size(); i++){
		const Grave & g = graveyard[i];
		// coast at most 2 s: an object gone for longer reappears where it
		// vanished (occlusion / stop), not where constant velocity says
		const float dt = std::min(2.0f, static_cast<float>((tMs - g.tMs) / 1000.0));
		const float px0 = g.obj.x0 + g.obj.vx * dt;
		const float px1 = g.obj.x1 + g.obj.vx * dt;
		if(lateralOverlapFrac(g.obj.y0, g.obj.y1, d.y0, d.y1) < cfg.gateLateral) continue;
		const float gap = gapX(px0, px1, d.x0, d.x1);
		if(gap > cfg.reacquireGapFrac) continue;
		if(d.hasVelocity && d.speed() >= cfg.minSpeedConfirm){
			if(signOf(d.vx, cfg.minSpeedConfirm) * signOf(g.obj.vx, cfg.minSpeedConfirm) < 0) continue;
		}
		if(gap < bestGap){
			bestGap = gap;
			bestIdx = static_cast<int>(i);
		}
	}
	if(bestIdx < 0) return 0;
	const int id = graveyard[bestIdx].obj.id;
	if(prior) *prior = graveyard[bestIdx].obj;
	graveyard.erase(graveyard.begin() + bestIdx);
	return id;
}

// ------------------------------------------------------------------ update

void MultiObjectTracker::update(const std::vector<Detection> & detections, double tMs,
                                const std::map<int, ShapeFeedback> * feedback){
	double dtSec = (lastTMs < 0) ? 0.0 : (tMs - lastTMs) / 1000.0;
	if(lastTMs < 0 || dtSec < 0 || dtSec > cfg.maxDtSec){
		tracks.clear();
		graveyard.clear();
		mergeCounts.clear();
		dtSec = 0;
		resetTMs = tMs;
	}
	lastTMs = tMs;

	// prune the graveyard
	graveyard.erase(std::remove_if(graveyard.begin(), graveyard.end(), [&](const Grave & g){
		return tMs - g.tMs > cfg.reacquireMs;
	}), graveyard.end());

	// 0. newest shape feedback per track
	if(feedback){
		for(auto & t : tracks){
			const auto it = feedback->find(t.obj.id);
			if(it == feedback->end()) continue;
			t.fb = it->second;
			// when did the model last see anything on this track — its own
			// class or another one (a boat read as "surfboard" is still
			// something, glitter is nothing)
			if(t.fb.modelAvailable){
				if(t.modelSeenMs < 0 || t.fb.modelAge == 0 || t.fb.otherClassHits > t.modelOtherSeen){
					t.modelSeenMs = tMs;
				}
				t.modelOtherSeen = t.fb.otherClassHits;
			}
		}
	}

	// 1. predict
	for(auto & t : tracks) predict(t, dtSec);

	// 2. association: cost-sorted greedy with gates and a direction veto
	struct Pair { float cost; int t, d; };
	std::vector<Pair> pairs;
	for(size_t ti = 0; ti < tracks.size(); ti++){
		const Track & tr = tracks[ti];
		const TrackedObject & o = tr.obj;
		const bool hidden = tr.occludedSinceMs >= 0;
		// a leaving object the model has not seen anything on for a while
		// is no longer fed by the blobs at that lane end (glitter where a
		// boat left): it ages out and coasts away as "occluded"
		if(cfg.modelGate && cfg.modelLostMs > 0 && o.confirmed && tr.fb.modelAvailable
		   && tr.modelSeenMs >= 0 && tMs - tr.modelSeenMs > cfg.modelLostMs && leaving(tr)) continue;
		float gate = cfg.gateGapFrac + 2.0f * std::fabs(o.vx) * static_cast<float>(dtSec);
		// the longer an object has been hidden, the less exact its
		// predicted position
		if(hidden) gate += cfg.occludedGateGrow * static_cast<float>((tMs - tr.occludedSinceMs) / 1000.0);
		for(size_t di = 0; di < detections.size(); di++){
			const Detection & d = detections[di];
			if(lateralOverlapFrac(o.y0, o.y1, d.y0, d.y1) < cfg.gateLateral) continue;
			const float gap = gapX(o.x0, o.x1, d.x0, d.x1);
			if(gap > gate) continue;
			// a stopped object only resumes from a detection that overlaps
			// its frozen box — not from anything passing next to it
			if(tr.stoppedSinceMs >= 0){
				if(iou(o, d) < 0.3f) continue;
				// ... and it must move on in the direction it arrived from;
				// an opposite-direction object passing over the frozen box
				// is a different one
				const int arrived = signOf(tr.lastMatchVx, cfg.minSpeedConfirm);
				const int now = d.hasVelocity ? signOf(d.vx, cfg.minSpeedConfirm) : 0;
				if(arrived != 0 && now != 0 && arrived != now) continue;
			}
			// a hidden object is only ever re-detected moving its own way:
			// the visible train passing in front of it moves the other way
			// and must never drag it along
			if(hidden){
				const int heading = headingOf(tr);
				const int now = d.hasVelocity ? signOf(d.vx, cfg.minSpeedConfirm) : 0;
				if(heading != 0 && now != heading) continue;
			}
			float velCost = 0;
			if(d.hasVelocity && d.speed() >= cfg.minSpeedConfirm
			   && std::hypot(o.vx, o.vy) >= cfg.minSpeedConfirm){
				const float dot = o.vx * d.vx + o.vy * d.vy;
				const float cosang = dot / (std::hypot(o.vx, o.vy) * d.speed());
				const float ang = std::acos(std::clamp(cosang, -1.0f, 1.0f)) * 180.0f / static_cast<float>(M_PI);
				if(ang > cfg.velAngleDeg) continue; // opposite traffic is never the same object
				velCost = std::hypot(o.vx - d.vx, o.vy - d.vy) / cfg.velScale;
			}
			float cost = gap / std::max(gate, 1e-6f) + (1.0f - iou(o, d)) * 0.5f + velCost * 0.3f;
			// a confirmed object outranks a tentative one for the same
			// detection: a fresh track must not steal the re-emerging
			// hidden object or a confirmed object's own blob
			if(!o.confirmed) cost += 1.0f;
			pairs.push_back({cost, static_cast<int>(ti), static_cast<int>(di)});
		}
	}
	std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b){ return a.cost < b.cost; });

	std::vector<int> trackMatch(tracks.size(), -1);
	std::vector<bool> detUsed(detections.size(), false);
	for(const auto & p : pairs){
		if(trackMatch[p.t] >= 0 || detUsed[p.d]) continue;
		trackMatch[p.t] = p.d;
		detUsed[p.d] = true;
	}

	// 3. one-to-many absorption: leftover fragments that sit inside the
	//    matched track's expanded box and move with it join its measurement.
	std::vector<Detection> measurements(tracks.size());
	GroupingParams gp;
	gp.gapFrac = cfg.absorbGapFrac;
	gp.angleDeg = cfg.velAngleDeg;
	gp.speedRatio = 3.0f;
	gp.lateralOverlap = cfg.absorbLateral;
	gp.minSpeed = cfg.minSpeedConfirm;
	for(size_t ti = 0; ti < tracks.size(); ti++){
		if(trackMatch[ti] < 0) continue;
		measurements[ti] = detections[trackMatch[ti]];
	}
	bool absorbed = true;
	while(absorbed){
		absorbed = false;
		for(size_t di = 0; di < detections.size(); di++){
			if(detUsed[di]) continue;
			const Detection & d = detections[di];
			int best = -1;
			float bestGap = std::numeric_limits<float>::max();
			for(size_t ti = 0; ti < tracks.size(); ti++){
				if(trackMatch[ti] < 0) continue;
				const Detection & m = measurements[ti];
				// expanded predicted box + current measurement union
				const float bx0 = std::min(m.x0, tracks[ti].obj.x0);
				const float bx1 = std::max(m.x1, tracks[ti].obj.x1);
				const float by0 = std::min(m.y0, tracks[ti].obj.y0);
				const float by1 = std::max(m.y1, tracks[ti].obj.y1);
				if(lateralOverlapFrac(by0, by1, d.y0, d.y1) < gp.lateralOverlap) continue;
				const float gap = gapX(bx0, bx1, d.x0, d.x1);
				if(gap > gp.gapFrac) continue;
				if(!velocitiesAgree(asDetection(tracks[ti].obj), d, gp)) continue;
				if(gap < bestGap){
					bestGap = gap;
					best = static_cast<int>(ti);
				}
			}
			if(best >= 0){
				measurements[best] = unionDetections(measurements[best], d);
				detUsed[di] = true;
				absorbed = true;
			}
		}
	}

	// 4. correct matched, age unmatched
	for(size_t ti = 0; ti < tracks.size(); ti++){
		Track & t = tracks[ti];
		if(trackMatch[ti] >= 0){
			const int h = headingOf(t);
			const bool holdRear = edgeCovered(h, h > 0 ? -1 : +1, measurements[ti], t.obj.id);
			const bool frontHidden = edgeCovered(h, h > 0 ? +1 : -1, measurements[ti], t.obj.id);
			correct(t, measurements[ti], dtSec, holdRear, frontHidden);
			t.obj.hits++;
			t.obj.misses = 0;
			// Hidden rear (see TrackerConfig): once a tentative track's
			// direction is settled, a free rear edge that sits inside an
			// oncoming object's box means the object came in behind that
			// one — pin the rear at the entry end it must have come
			// through. (Decided only now, not at birth: a wake fragment's
			// first velocity sample can point the wrong way.)
			if(!t.obj.confirmed && !t.hiddenRearChecked && t.dirVotes >= std::min(cfg.confirmFrames, 3)){
				t.hiddenRearChecked = true;
				const Detection & m = measurements[ti];
				const int hd = t.dirSign;
				const bool rearFree = hd > 0 ? !t.obj.clippedLeft : hd < 0 ? !t.obj.clippedRight : false;
				const float blobW = std::max(0.0f, m.x1 - m.x0);
				if(rearFree && (cfg.minConfirmLen <= 0.0f || blobW >= cfg.minConfirmLen)
				   && !nearSameDirection(hd, m, t.obj.id)
				   && !sliverInOncoming(hd, m, t.obj.id)
				   && edgeCovered(hd, hd > 0 ? -1 : +1, m, t.obj.id)){
					cv::Mat & s = t.kf.statePost;
					if(hd > 0){
						s.at<float>(X0) = 0.0f;
						t.obj.clippedLeft = true;
					}else{
						s.at<float>(X1) = 1.0f;
						t.obj.clippedRight = true;
					}
					t.kf.statePre = s.clone();
					t.clipVotesL = t.clipVotesR = 0;
					t.bornByTouch = true;
					t.obj.edgeBorn = true;
					refreshDerived(t);
				}
			}
		}else{
			t.obj.misses++;
			if(t.obj.confirmed && t.stoppedSinceMs < 0 && t.occludedSinceMs < 0
			   && t.obj.misses > cfg.maxMisses){
				// A confirmed object that vanished at low speed inside the
				// corridor has halted: freeze it instead of coasting away.
				// One that vanished while moving is hidden, not gone: it
				// keeps coasting at its last velocity as "occluded" until
				// it is seen again or its trailing edge has left the
				// corridor. (Fading out close to the end it heads for is
				// the same thing — trees / a building swallow the last
				// wagons before they touch the lane edge.)
				const int heading = headingOf(t);
				const bool nearEnd = (heading <= 0 && t.obj.x0 < cfg.stopEdgeFrac)
				                     || (heading >= 0 && t.obj.x1 > 1.0f - cfg.stopEdgeFrac);
				const bool slow = std::hypot(t.lastMatchVx, t.lastMatchVy) < cfg.stopSpeed;
				if(slow && !t.obj.clippedLeft && !t.obj.clippedRight && !nearEnd
				   && t.obj.w >= cfg.stopMinWidthFrac){
					t.stoppedSinceMs = tMs;
					// undo the coasting of the missed frames: back to where it
					// was last seen
					const float back = static_cast<float>(cfg.maxMisses) * static_cast<float>(dtSec);
					cv::Mat & s = t.kf.statePost;
					s.at<float>(X0) -= t.obj.vx * back;
					s.at<float>(X1) -= t.obj.vx * back;
					s.at<float>(Y0) -= t.obj.vy * back;
					s.at<float>(Y1) -= t.obj.vy * back;
					s.at<float>(VX) = 0.0f;
					s.at<float>(VY) = 0.0f;
					t.kf.statePre = s.clone();
					refreshDerived(t);
				}else if(cfg.coastOccluded && heading != 0){
					t.occludedSinceMs = tMs;
					// coast at the velocity it was last seen with, dead
					// straight (rails and rivers do not turn here). The
					// rear edge is released: a rear still pinned at the
					// entry end would make the box grow without end.
					if(heading > 0) t.obj.clippedLeft = false;
					else t.obj.clippedRight = false;
					cv::Mat & s = t.kf.statePost;
					s.at<float>(VX) = t.lastMatchVx;
					s.at<float>(VY) = cfg.lockAxis ? 0.0f : t.lastMatchVy;
					t.kf.statePre = s.clone();
					refreshDerived(t);
				}
			}
		}
		const float speed = std::hypot(t.obj.vx, t.obj.vy);
		if(!t.obj.confirmed && t.obj.hits >= cfg.confirmFrames
		   && speed >= cfg.minSpeedConfirm && t.dirVotes >= std::min(cfg.confirmFrames, 3)){
			bool ok = true;
			// edge rule: it must have come in through a corridor end
			if(cfg.edgeRule && !t.obj.edgeBorn) ok = false;
			// length gate: a car behind the viaduct never gets this long
			if(cfg.minConfirmLen > 0 && t.obj.w < cfg.minConfirmLen) ok = false;
			// model gate: the segmentation model must have found it, and
			// not mostly called it something else. (Re-evaluated every
			// frame: the nose of a train at the lane end is "truck" for the
			// first few calls, so an early verdict must not be final.)
			if(cfg.modelGate && t.fb.modelAvailable){
				const bool found = t.fb.modelHits >= 1;
				const bool mostlyOther = t.fb.otherClassHits >= cfg.modelGateOtherHits
				                         && t.fb.otherClassHits > 2 * t.fb.modelHits;
				if(!found || mostlyOther) ok = false;
			}
			// shadow suppression: a fragment / wake travelling right next
			// to a confirmed object in the same direction is not an object
			bool shadow = t.isShadow;
			if(ok && cfg.shadowSuppress && !shadow){
				for(size_t oi = 0; oi < tracks.size() && !shadow; oi++){
					if(oi == ti || !tracks[oi].obj.confirmed || tracks[oi].stoppedSinceMs >= 0) continue;
					const TrackedObject & o = tracks[oi].obj;
					const int so = signOf(o.vx, cfg.minSpeedConfirm), st = signOf(t.obj.vx, cfg.minSpeedConfirm);
					if(so == 0 || st == 0 || so != st) continue; // only moving, same direction
					const float gapY = std::max(0.0f, std::max(o.y0, t.obj.y0) - std::min(o.y1, t.obj.y1));
					if(gapY > cfg.shadowLateral * std::max(1e-3f, o.y1 - o.y0)) continue;
					if(gapX(o.x0, o.x1, t.obj.x0, t.obj.x1) <= cfg.shadowGapFrac) shadow = true;
				}
				// once a wake, always a wake: the object moving away must
				// not turn its trailing water into a second object
				t.isShadow = shadow;
			}
			if(ok && !shadow) t.obj.confirmed = true;
		}
		if(cfg.lockAxis && t.dirVotes >= 3){
			// rails do not turn: kill lateral drift
			t.kf.statePost.at<float>(VY) = 0.0f;
			refreshDerived(t);
		}
		t.obj.trail.emplace_back(t.obj.x, t.obj.y);
		if(static_cast<int>(t.obj.trail.size()) > cfg.trailLen){
			t.obj.trail.erase(t.obj.trail.begin(),
			                  t.obj.trail.begin() + (t.obj.trail.size() - cfg.trailLen));
		}
	}

	// 5. bury dead tracks (confirmed ones can be reacquired for a while);
	//    stopped ones are held until stoppedHoldMs runs out, hidden ones
	//    until their trailing edge has left the corridor
	for(auto it = tracks.begin(); it != tracks.end();){
		const int limit = it->obj.confirmed ? cfg.maxMisses : cfg.tentativeMaxMisses;
		const bool heldStopped = it->stoppedSinceMs >= 0 && tMs - it->stoppedSinceMs <= cfg.stoppedHoldMs;
		const bool heldOccluded = it->occludedSinceMs >= 0
			&& tMs - it->occludedSinceMs <= cfg.occludedMaxMs && !pastFarEnd(*it);
		if(it->obj.misses > limit && !heldStopped && !heldOccluded){
			// an object that coasted out past the far end has left the
			// corridor: nothing at that lane end can be it again (glitter
			// where a boat left resurrected it for 25 s otherwise)
			if(it->obj.confirmed && !pastFarEnd(*it)) graveyard.push_back({it->obj, tMs});
			it = tracks.erase(it);
		}else{
			++it;
		}
	}

	// 6. spawn from leftovers (may inherit an id from the graveyard)
	for(size_t di = 0; di < detections.size(); di++){
		if(!detUsed[di]) tracks.push_back(makeTrack(detections[di], tMs));
	}

	// 6b. a hidden object re-emerging as a fresh tentative track: the
	//     coasting track adopts it (box + velocity) and keeps its id
	{
		std::vector<bool> dead(tracks.size(), false);
		for(size_t oi = 0; oi < tracks.size(); oi++){
			Track & occ = tracks[oi];
			if(occ.occludedSinceMs < 0 || dead[oi]) continue;
			if(leaving(occ)) continue; // on its way out: coasts out, no comeback
			const int heading = headingOf(occ);
			const float grow = cfg.occludedGateGrow * static_cast<float>((tMs - occ.occludedSinceMs) / 1000.0);
			int best = -1;
			float bestGap = std::numeric_limits<float>::max();
			for(size_t fi = 0; fi < tracks.size(); fi++){
				if(fi == oi || dead[fi]) continue;
				const Track & fr = tracks[fi];
				if(fr.obj.confirmed || fr.obj.misses > 0) continue;
				if(signOf(fr.obj.vx, cfg.minSpeedConfirm) != heading) continue;
				if(lateralOverlapFrac(occ.obj.y0, occ.obj.y1, fr.obj.y0, fr.obj.y1) < cfg.gateLateral) continue;
				const float gap = gapX(occ.obj.x0, occ.obj.x1, fr.obj.x0, fr.obj.x1);
				if(gap > cfg.reacquireGapFrac + grow) continue;
				if(gap < bestGap){ bestGap = gap; best = static_cast<int>(fi); }
			}
			if(best >= 0){
				adoptInto(occ, tracks[best]);
				dead[best] = true;
			}
		}
		for(size_t i = tracks.size(); i-- > 0;){
			if(dead[i]) tracks.erase(tracks.begin() + i);
		}
	}

	// 7. track-track merge: two tracks travelling together for mergeFrames
	//    are one object (a head and a tail that got separate ids before the
	//    absorption could join them).
	std::map<std::pair<int, int>, int> nextCounts;
	std::vector<bool> dead(tracks.size(), false);
	for(size_t i = 0; i < tracks.size(); i++){
		for(size_t j = i + 1; j < tracks.size(); j++){
			if(dead[i] || dead[j]) continue;
			const TrackedObject & a = tracks[i].obj;
			const TrackedObject & b = tracks[j].obj;
			if(!a.confirmed && !b.confirmed) continue;
			if(lateralOverlapFrac(a.y0, a.y1, b.y0, b.y1) < cfg.gateLateral) continue;
			if(gapX(a.x0, a.x1, b.x0, b.x1) > cfg.mergeGapFrac) continue;
			if(std::hypot(a.vx - b.vx, a.vy - b.vy) > cfg.mergeVelTol) continue;
			if(signOf(a.vx, cfg.minSpeedConfirm) * signOf(b.vx, cfg.minSpeedConfirm) < 0) continue;
			// a stopped / hidden object only merges with something leaving
			// in the direction it arrived from (its own departure), never
			// with an opposite-direction object passing over it
			const bool aHeld = tracks[i].stoppedSinceMs >= 0 || tracks[i].occludedSinceMs >= 0;
			const bool bHeld = tracks[j].stoppedSinceMs >= 0 || tracks[j].occludedSinceMs >= 0;
			if(aHeld || bHeld){
				const Track & st = aHeld ? tracks[i] : tracks[j];
				const TrackedObject & mv = aHeld ? b : a;
				const int arrived = signOf(st.lastMatchVx, cfg.minSpeedConfirm);
				const int now = signOf(mv.vx, cfg.minSpeedConfirm);
				if(arrived != 0 && now != 0 && arrived != now) continue;
				// a hidden object on its way out is not brought back by
				// a fragment at the lane end it is leaving through
				if(st.occludedSinceMs >= 0 && leaving(st) && !mv.confirmed) continue;
			}
			const auto key = std::make_pair(std::min(a.id, b.id), std::max(a.id, b.id));
			const int count = mergeCounts.count(key) ? mergeCounts[key] + 1 : 1;
			if(count >= cfg.mergeFrames){
				// keep the one with a silhouette (the model found an object
				// there, not on the other), else the confirmed / older one
				size_t keep = i, gone = j;
				const bool aShape = tracks[i].fb.outlineValid, bShape = tracks[j].fb.outlineValid;
				if(aShape != bShape){
					if(bShape){ keep = j; gone = i; }
				}else if((b.confirmed && !a.confirmed) || (a.confirmed == b.confirmed && b.id < a.id)){
					keep = j; gone = i;
				}
				mergeInto(tracks[keep], tracks[gone]);
				dead[gone] = true;
			}else{
				nextCounts[key] = count;
			}
		}
	}
	mergeCounts.swap(nextCounts);
	for(size_t i = tracks.size(); i-- > 0;){
		if(dead[i]) tracks.erase(tracks.begin() + i);
	}
}

std::vector<TrackedObject> MultiObjectTracker::getObjects() const {
	std::vector<TrackedObject> out;
	out.reserve(tracks.size());
	for(const auto & t : tracks) out.push_back(t.obj);
	return out;
}

} // namespace tracking
