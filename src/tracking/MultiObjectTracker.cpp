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

void MultiObjectTracker::correct(Track & t, const Detection & d, double){
	cv::Mat & R = t.kf.measurementNoiseCov;
	cv::setIdentity(R, cv::Scalar::all(cfg.posMeasNoise));

	// A confirmed object does not change length from one frame to the
	// next. A detection much shorter than the predicted box is a fragment
	// (sparkling water / occlusion ate part of the hull), one much longer
	// has picked up a wake or a neighbour: its box edges are weak evidence
	// and must not collapse or inflate the box in one step. Such a
	// fragment can also not release a pinned edge — the object's end has
	// not entered the lane, the detector merely lost sight of it.
	const cv::Mat & pre = t.kf.statePre;
	const float predW = std::max(1e-3f, pre.at<float>(X1) - pre.at<float>(X0));
	const float ratio = d.w() / predW;
	const bool fragment = t.obj.confirmed && ratio < cfg.sizeJumpTrust;
	const bool inflated = t.obj.confirmed && !t.obj.clippedLeft && !t.obj.clippedRight
	                      && !d.touchesLeft && !d.touchesRight && ratio > 1.0f / cfg.sizeJumpTrust;
	if(fragment || inflated){
		for(int i = X0; i <= X1; i++) R.at<float>(i, i) = cfg.posMeasNoise * cfg.sizeJumpNoiseGain;
	}
	bool touchesLeft = d.touchesLeft || (fragment && t.obj.clippedLeft);
	bool touchesRight = d.touchesRight || (fragment && t.obj.clippedRight);
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
	if(t.obj.clippedLeft != touchesLeft) reseed(X0, d.x0, touchesLeft ? 1e-6f : 1e-2f);
	if(t.obj.clippedRight != touchesRight) reseed(X1, d.x1, touchesRight ? 1e-6f : 1e-2f);

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
	// a pinned edge that the fragment could not release stays at the lane end
	const float zx0 = (touchesLeft && !d.touchesLeft) ? pre.at<float>(X0) : d.x0;
	const float zx1 = (touchesRight && !d.touchesRight) ? pre.at<float>(X1) : d.x1;
	const cv::Mat z = (cv::Mat_<float>(kMeas, 1) << zx0, zx1, d.y0, d.y1, vx, vy);
	t.kf.correct(z);
	t.obj.clippedLeft = touchesLeft;
	t.obj.clippedRight = touchesRight;
	t.stoppedSinceMs = -1; // moving again
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
	// the merged track is as alive as the fresher of the two: a stopped
	// object that a new track has been riding on is moving again
	if(gone.obj.misses < keep.obj.misses){
		keep.obj.misses = gone.obj.misses;
		keep.lastMatchVx = gone.lastMatchVx;
		keep.lastMatchVy = gone.lastMatchVy;
	}
	if(keep.obj.misses == 0) keep.stoppedSinceMs = -1;
	refreshDerived(keep);
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

void MultiObjectTracker::update(const std::vector<Detection> & detections, double tMs){
	double dtSec = (lastTMs < 0) ? 0.0 : (tMs - lastTMs) / 1000.0;
	if(dtSec < 0 || dtSec > cfg.maxDtSec){
		tracks.clear();
		graveyard.clear();
		mergeCounts.clear();
		dtSec = 0;
	}
	lastTMs = tMs;

	// prune the graveyard
	graveyard.erase(std::remove_if(graveyard.begin(), graveyard.end(), [&](const Grave & g){
		return tMs - g.tMs > cfg.reacquireMs;
	}), graveyard.end());

	// 1. predict
	for(auto & t : tracks) predict(t, dtSec);

	// 2. association: cost-sorted greedy with gates and a direction veto
	struct Pair { float cost; int t, d; };
	std::vector<Pair> pairs;
	for(size_t ti = 0; ti < tracks.size(); ti++){
		const TrackedObject & o = tracks[ti].obj;
		const float gate = cfg.gateGapFrac + 2.0f * std::fabs(o.vx) * static_cast<float>(dtSec);
		for(size_t di = 0; di < detections.size(); di++){
			const Detection & d = detections[di];
			if(lateralOverlapFrac(o.y0, o.y1, d.y0, d.y1) < cfg.gateLateral) continue;
			const float gap = gapX(o.x0, o.x1, d.x0, d.x1);
			if(gap > gate) continue;
			// a stopped object only resumes from a detection that overlaps
			// its frozen box — not from anything passing next to it
			if(tracks[ti].stoppedSinceMs >= 0){
				if(iou(o, d) < 0.3f) continue;
				// ... and it must move on in the direction it arrived from;
				// an opposite-direction object passing over the frozen box
				// is a different one
				const int arrived = signOf(tracks[ti].lastMatchVx, cfg.minSpeedConfirm);
				const int now = d.hasVelocity ? signOf(d.vx, cfg.minSpeedConfirm) : 0;
				if(arrived != 0 && now != 0 && arrived != now) continue;
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
			const float cost = gap / std::max(gate, 1e-6f) + (1.0f - iou(o, d)) * 0.5f + velCost * 0.3f;
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
			correct(t, measurements[ti], dtSec);
			t.obj.hits++;
			t.obj.misses = 0;
		}else{
			t.obj.misses++;
			// A confirmed object that vanished at low speed inside the
			// corridor has halted: freeze it instead of coasting away.
			// An object that fades out close to the corridor end it is
			// heading for has left (trees / a building swallow the last
			// wagons before they touch the lane edge) — not stopped.
			const int heading = signOf(t.lastMatchVx, cfg.minSpeedConfirm);
			const bool nearEnd = (heading <= 0 && t.obj.x0 < cfg.stopEdgeFrac)
			                     || (heading >= 0 && t.obj.x1 > 1.0f - cfg.stopEdgeFrac);
			if(t.obj.confirmed && t.stoppedSinceMs < 0 && t.obj.misses > cfg.maxMisses
			   && !t.obj.clippedLeft && !t.obj.clippedRight && !nearEnd
			   && t.obj.w >= cfg.stopMinWidthFrac
			   && std::hypot(t.lastMatchVx, t.lastMatchVy) < cfg.stopSpeed){
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
			}
		}
		const float speed = std::hypot(t.obj.vx, t.obj.vy);
		if(!t.obj.confirmed && t.obj.hits >= cfg.confirmFrames
		   && speed >= cfg.minSpeedConfirm && t.dirVotes >= std::min(cfg.confirmFrames, 3)){
			// shadow suppression: a fragment / wake travelling right next
			// to a confirmed object in the same direction is not an object
			bool shadow = t.isShadow;
			if(cfg.shadowSuppress && !shadow){
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
			if(!shadow) t.obj.confirmed = true;
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
	//    stopped ones are held until stoppedHoldMs runs out
	for(auto it = tracks.begin(); it != tracks.end();){
		const int limit = it->obj.confirmed ? cfg.maxMisses : cfg.tentativeMaxMisses;
		const bool heldStopped = it->stoppedSinceMs >= 0 && tMs - it->stoppedSinceMs <= cfg.stoppedHoldMs;
		if(it->obj.misses > limit && !heldStopped){
			if(it->obj.confirmed) graveyard.push_back({it->obj, tMs});
			it = tracks.erase(it);
		}else{
			++it;
		}
	}

	// 6. spawn from leftovers (may inherit an id from the graveyard)
	for(size_t di = 0; di < detections.size(); di++){
		if(!detUsed[di]) tracks.push_back(makeTrack(detections[di], tMs));
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
			// a stopped object only merges with something leaving in the
			// direction it arrived from (its own departure), never with an
			// opposite-direction object passing over it
			const bool aStopped = tracks[i].stoppedSinceMs >= 0, bStopped = tracks[j].stoppedSinceMs >= 0;
			if(aStopped || bStopped){
				const Track & st = aStopped ? tracks[i] : tracks[j];
				const TrackedObject & mv = aStopped ? b : a;
				const int arrived = signOf(st.lastMatchVx, cfg.minSpeedConfirm);
				const int now = signOf(mv.vx, cfg.minSpeedConfirm);
				if(arrived != 0 && now != 0 && arrived != now) continue;
			}
			const auto key = std::make_pair(std::min(a.id, b.id), std::max(a.id, b.id));
			const int count = mergeCounts.count(key) ? mergeCounts[key] + 1 : 1;
			if(count >= cfg.mergeFrames){
				// keep the older (smaller id) / confirmed one
				size_t keep = i, gone = j;
				if((b.confirmed && !a.confirmed) || (a.confirmed == b.confirmed && b.id < a.id)){
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
