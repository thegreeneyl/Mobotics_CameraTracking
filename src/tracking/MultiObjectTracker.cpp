#include "MultiObjectTracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tracking {

namespace {
constexpr float kPosMeasurementVar = 2.5e-5f; // (0.5% width)^2
constexpr float kPosProcessVar = 1e-5f;
constexpr float kVelProcessVar = 1e-3f;
} // namespace

MultiObjectTracker::MultiObjectTracker(const TrackerConfig & config) : cfg(config){}

void MultiObjectTracker::reset(){
	tracks.clear();
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

float MultiObjectTracker::iou(float ax, float ay, float aw, float ah,
                              float bx, float by, float bw, float bh){
	const float ax0 = ax - aw / 2, ay0 = ay - ah / 2, ax1 = ax + aw / 2, ay1 = ay + ah / 2;
	const float bx0 = bx - bw / 2, by0 = by - bh / 2, bx1 = bx + bw / 2, by1 = by + bh / 2;
	const float ix = std::max(0.0f, std::min(ax1, bx1) - std::max(ax0, bx0));
	const float iy = std::max(0.0f, std::min(ay1, by1) - std::max(ay0, by0));
	const float inter = ix * iy;
	const float uni = aw * ah + bw * bh - inter;
	return uni <= 0 ? 0 : inter / uni;
}

MultiObjectTracker::Track MultiObjectTracker::makeTrack(const Detection & d){
	Track t;
	t.kf.init(4, 2, 0, CV_32F);
	// state: [x, y, vx, vy]; transition dt is filled per frame in update().
	t.kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
	t.kf.measurementMatrix = (cv::Mat_<float>(2, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0);
	cv::setIdentity(t.kf.processNoiseCov, cv::Scalar::all(kPosProcessVar));
	t.kf.processNoiseCov.at<float>(2, 2) = kVelProcessVar;
	t.kf.processNoiseCov.at<float>(3, 3) = kVelProcessVar;
	cv::setIdentity(t.kf.measurementNoiseCov, cv::Scalar::all(kPosMeasurementVar));
	cv::setIdentity(t.kf.errorCovPost, cv::Scalar::all(0.1));
	t.kf.statePost = (cv::Mat_<float>(4, 1) << d.x, d.y, 0, 0);

	t.obj.id = nextId++;
	t.obj.x = d.x;
	t.obj.y = d.y;
	t.obj.w = d.w;
	t.obj.h = d.h;
	t.obj.hits = 1;
	colorForId(t.obj.id, t.obj.colorR, t.obj.colorG, t.obj.colorB);
	if(!d.label.empty()){
		t.labelVotes[d.label] = 1;
		t.obj.label = d.label;
	}
	t.obj.trail.emplace_back(d.x, d.y);
	return t;
}

void MultiObjectTracker::update(const std::vector<Detection> & detections, double tMs){
	double dtSec = (lastTMs < 0) ? 0.0 : (tMs - lastTMs) / 1000.0;
	if(dtSec < 0 || dtSec > cfg.maxDtSec){
		// Source switch / recording wrap — start fresh from these detections.
		tracks.clear();
		dtSec = 0;
	}
	lastTMs = tMs;

	// 1. Predict all tracks forward by dt.
	for(auto & t : tracks){
		t.kf.transitionMatrix.at<float>(0, 2) = static_cast<float>(dtSec);
		t.kf.transitionMatrix.at<float>(1, 3) = static_cast<float>(dtSec);
		const cv::Mat pred = t.kf.predict();
		t.obj.x = pred.at<float>(0);
		t.obj.y = pred.at<float>(1);
		t.obj.vx = pred.at<float>(2);
		t.obj.vy = pred.at<float>(3);
		t.obj.ageFrames++;
	}

	std::vector<int> trackMatch(tracks.size(), -1); // matched detection index
	std::vector<bool> detMatched(detections.size(), false);

	// 2a. Greedy IoU association (best pair first).
	while(true){
		float bestIoU = cfg.minIoU;
		int bestT = -1, bestD = -1;
		for(size_t ti = 0; ti < tracks.size(); ti++){
			if(trackMatch[ti] >= 0) continue;
			const auto & o = tracks[ti].obj;
			for(size_t di = 0; di < detections.size(); di++){
				if(detMatched[di]) continue;
				const auto & d = detections[di];
				const float v = iou(o.x, o.y, o.w, o.h, d.x, d.y, d.w, d.h);
				if(v > bestIoU){
					bestIoU = v;
					bestT = static_cast<int>(ti);
					bestD = static_cast<int>(di);
				}
			}
		}
		if(bestT < 0) break;
		trackMatch[bestT] = bestD;
		detMatched[bestD] = true;
	}

	// 2b. Centroid-distance fallback for small/fast objects whose boxes
	// barely overlap between frames.
	while(true){
		float bestDist = cfg.maxDist;
		int bestT = -1, bestD = -1;
		for(size_t ti = 0; ti < tracks.size(); ti++){
			if(trackMatch[ti] >= 0) continue;
			const auto & o = tracks[ti].obj;
			for(size_t di = 0; di < detections.size(); di++){
				if(detMatched[di]) continue;
				const auto & d = detections[di];
				const float dist = std::hypot(o.x - d.x, o.y - d.y);
				if(dist < bestDist){
					bestDist = dist;
					bestT = static_cast<int>(ti);
					bestD = static_cast<int>(di);
				}
			}
		}
		if(bestT < 0) break;
		trackMatch[bestT] = bestD;
		detMatched[bestD] = true;
	}

	// 3. Update matched tracks, age unmatched ones.
	for(size_t ti = 0; ti < tracks.size(); ti++){
		auto & t = tracks[ti];
		if(trackMatch[ti] >= 0){
			const Detection & d = detections[trackMatch[ti]];
			const cv::Mat meas = (cv::Mat_<float>(2, 1) << d.x, d.y);
			const cv::Mat post = t.kf.correct(meas);
			t.obj.x = post.at<float>(0);
			t.obj.y = post.at<float>(1);
			t.obj.vx = post.at<float>(2);
			t.obj.vy = post.at<float>(3);
			const float a = cfg.bboxSmoothing;
			t.obj.w = (1 - a) * t.obj.w + a * d.w;
			t.obj.h = (1 - a) * t.obj.h + a * d.h;
			t.obj.hits++;
			t.obj.misses = 0;
			if(!d.label.empty()){
				const int votes = ++t.labelVotes[d.label];
				if(t.obj.label.empty() || votes >= t.labelVotes[t.obj.label]){
					t.obj.label = d.label;
				}
			}
		}else{
			t.obj.misses++;
		}
		if(!t.obj.confirmed && t.obj.hits >= cfg.confirmFrames){
			t.obj.confirmed = true;
		}
		t.obj.trail.emplace_back(t.obj.x, t.obj.y);
		if(static_cast<int>(t.obj.trail.size()) > cfg.trailLen){
			t.obj.trail.erase(t.obj.trail.begin(),
			                  t.obj.trail.begin() + (t.obj.trail.size() - cfg.trailLen));
		}
	}

	// 4. Kill dead tracks: confirmed ones after maxMisses, tentative ones
	// quickly (that is what suppresses water-glint noise).
	tracks.erase(std::remove_if(tracks.begin(), tracks.end(), [this](const Track & t){
		const int limit = t.obj.confirmed ? cfg.maxMisses : cfg.tentativeMaxMisses;
		return t.obj.misses > limit;
	}), tracks.end());

	// 5. Spawn tentative tracks from unmatched detections.
	for(size_t di = 0; di < detections.size(); di++){
		if(!detMatched[di]) tracks.push_back(makeTrack(detections[di]));
	}
}

std::vector<TrackedObject> MultiObjectTracker::getObjects() const {
	std::vector<TrackedObject> out;
	out.reserve(tracks.size());
	for(const auto & t : tracks) out.push_back(t.obj);
	return out;
}

} // namespace tracking
