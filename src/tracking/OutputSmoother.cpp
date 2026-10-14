#include "OutputSmoother.h"

#include <algorithm>
#include <cmath>

namespace tracking {

namespace {
using P = std::pair<float, float>;

constexpr double kResetGapMs = 5000.0; // same rule as the tracker

float dist(const P & a, const P & b){
	return std::hypot(a.first - b.first, a.second - b.second);
}

// Moves `cur` toward `target`, at most `maxStep` away from `cur`.
float approach(float cur, float target, float maxStep){
	const float d = target - cur;
	if(std::fabs(d) <= maxStep) return target;
	return cur + (d > 0 ? maxStep : -maxStep);
}
} // namespace

// Resample rule: the polygon is walked clockwise (y down), starting at the
// point where its boundary crosses the horizontal line through the centre
// of its bounding box on the right-hand side (the "nose" of the object).
// With start and orientation fixed, vertex k of one frame's outline lies
// on the same part of the object as vertex k of the next frame's, so the
// per-vertex low-pass in OutputSmoother::blendOutline averages shape, not
// garbage.
std::vector<P> resampleClosed(const std::vector<P> & in, int n){
	std::vector<P> out;
	if(in.size() < 3 || n < 3) return out;
	std::vector<P> poly = in;
	// orientation: positive signed area = clockwise with y down
	double area = 0;
	for(size_t i = 0; i < poly.size(); i++){
		const P & a = poly[i];
		const P & b = poly[(i + 1) % poly.size()];
		area += static_cast<double>(a.first) * b.second - static_cast<double>(b.first) * a.second;
	}
	if(area < 0) std::reverse(poly.begin(), poly.end());

	float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
	for(const P & p : poly){
		minX = std::min(minX, p.first); maxX = std::max(maxX, p.first);
		minY = std::min(minY, p.second); maxY = std::max(maxY, p.second);
	}
	const float midY = (minY + maxY) * 0.5f;

	// start point: rightmost crossing of y = midY
	int startEdge = -1;
	P start{-1e9f, midY};
	for(size_t i = 0; i < poly.size(); i++){
		const P & a = poly[i];
		const P & b = poly[(i + 1) % poly.size()];
		if((a.second <= midY) == (b.second <= midY)) continue; // no crossing
		const float t = (midY - a.second) / (b.second - a.second);
		const float x = a.first + t * (b.first - a.first);
		if(x > start.first){
			start = {x, midY};
			startEdge = static_cast<int>(i);
		}
	}
	if(startEdge < 0){
		// degenerate (flat): start at the rightmost vertex
		size_t best = 0;
		for(size_t i = 1; i < poly.size(); i++) if(poly[i].first > poly[best].first) best = i;
		std::rotate(poly.begin(), poly.begin() + best, poly.end());
		start = poly.front();
		startEdge = static_cast<int>(poly.size()) - 1; // the walk below begins at vertex 0
	}

	// boundary as a point sequence from the start point around and back
	std::vector<P> seq;
	seq.reserve(poly.size() + 2);
	seq.push_back(start);
	for(size_t k = 1; k <= poly.size(); k++) seq.push_back(poly[(startEdge + k) % poly.size()]);
	seq.push_back(start);
	std::vector<float> cum(seq.size(), 0.0f);
	for(size_t i = 1; i < seq.size(); i++) cum[i] = cum[i - 1] + dist(seq[i - 1], seq[i]);
	const float L = cum.back();
	if(L <= 1e-9f){
		out.assign(n, start);
		return out;
	}
	out.reserve(n);
	size_t seg = 1;
	for(int k = 0; k < n; k++){
		const float s = L * k / n;
		while(seg < seq.size() - 1 && cum[seg] < s) seg++;
		const float segLen = cum[seg] - cum[seg - 1];
		const float t = segLen <= 1e-9f ? 0.0f : (s - cum[seg - 1]) / segLen;
		out.emplace_back(seq[seg - 1].first + t * (seq[seg].first - seq[seg - 1].first),
		                 seq[seg - 1].second + t * (seq[seg].second - seq[seg - 1].second));
	}
	return out;
}

// ------------------------------------------------------------- smoother

void OutputSmoother::initOutline(State & st) const {
	// the box itself, as a polygon (box-normalized)
	const std::vector<P> rect = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
	st.outline = resampleClosed(rect, std::max(3, cfg.outlineVertices));
}

void OutputSmoother::blendOutline(State & st, const TrackedObject & o) const {
	if(!o.shape.valid || o.shape.outline.size() < 3 || o.w <= 1e-6f || o.h <= 1e-6f) return;
	std::vector<P> norm;
	norm.reserve(o.shape.outline.size());
	for(const P & p : o.shape.outline){
		norm.emplace_back((p.first - o.x0) / o.w, (p.second - o.y0) / o.h);
	}
	const int n = std::max(3, cfg.outlineVertices);
	const std::vector<P> rs = resampleClosed(norm, n);
	if(static_cast<int>(rs.size()) != n) return;
	if(static_cast<int>(st.outline.size()) != n){
		st.outline = rs;
		return;
	}
	const float a = std::clamp(cfg.outlineAlpha, 0.01f, 1.0f);
	for(int i = 0; i < n; i++){
		st.outline[i].first += (rs[i].first - st.outline[i].first) * a;
		st.outline[i].second += (rs[i].second - st.outline[i].second) * a;
	}
}

void OutputSmoother::publish(const State & st, TrackedObject & o) const {
	o.x0 = st.x0; o.x1 = st.x1; o.y0 = st.y0; o.y1 = st.y1;
	o.x = (o.x0 + o.x1) * 0.5f;
	o.y = (o.y0 + o.y1) * 0.5f;
	o.w = o.x1 - o.x0;
	o.h = o.y1 - o.y0;
	o.presence = st.presence;
	o.shape.valid = true;
	o.shape.outline.clear();
	o.shape.outline.reserve(st.outline.size());
	for(const P & uv : st.outline){
		o.shape.outline.emplace_back(o.x0 + uv.first * o.w, o.y0 + uv.second * o.h);
	}
}

void OutputSmoother::apply(std::vector<TrackedObject> & objects, double tMs){
	if(!cfg.enabled){
		for(auto & o : objects) o.presence = 1.0f;
		return;
	}
	double dtMs = lastTMs < 0 ? 0.0 : tMs - lastTMs;
	if(lastTMs < 0 || dtMs < 0 || dtMs > kResetGapMs){
		states.clear();
		dtMs = 0;
	}
	lastTMs = tMs;
	const float dt = static_cast<float>(dtMs / 1000.0);
	const float maxStep = cfg.boxRate * dt;

	for(auto & kv : states) kv.second.seenThisFrame = false;

	for(auto & o : objects){
		if(!o.confirmed) continue;
		auto it = states.find(o.id);
		if(it == states.end()){
			State st;
			st.x0 = o.x0; st.x1 = o.x1; st.y0 = o.y0; st.y1 = o.y1;
			st.presence = 0.0f;
			initOutline(st);
			it = states.emplace(o.id, std::move(st)).first;
		}
		State & st = it->second;
		st.seenThisFrame = true;
		// the box travels with the object, every edge slides toward the
		// tracker's edge at a bounded rate. A pinned edge does not travel
		// (it sits at the corridor end), but it is rate-limited like the
		// others: when the tracker snaps an edge to the end the published
		// edge glides there within a few frames instead of jumping.
		const bool pinL = o.clippedLeft && o.x0 <= 0.02f;
		const bool pinR = o.clippedRight && o.x1 >= 0.98f;
		st.x0 = approach(pinL ? st.x0 : st.x0 + o.vx * dt, o.x0, maxStep);
		st.x1 = approach(pinR ? st.x1 : st.x1 + o.vx * dt, o.x1, maxStep);
		st.y0 = approach(st.y0 + o.vy * dt, o.y0, maxStep);
		st.y1 = approach(st.y1 + o.vy * dt, o.y1, maxStep);
		if(st.x1 < st.x0) std::swap(st.x0, st.x1);
		if(st.y1 < st.y0) std::swap(st.y0, st.y1);
		blendOutline(st, o);
		st.presence = cfg.fadeInMs <= 0 ? 1.0f
			: std::min(1.0f, st.presence + static_cast<float>(dtMs / cfg.fadeInMs));
		st.lastTMs = tMs;
		st.last = o;
		publish(st, o);
	}

	// objects the tracker dropped: coast and fade out as "gone"
	for(auto it = states.begin(); it != states.end();){
		State & st = it->second;
		if(st.seenThisFrame){ ++it; continue; }
		st.presence -= cfg.fadeOutMs <= 0 ? 1.0f : static_cast<float>(dtMs / cfg.fadeOutMs);
		if(st.presence <= 0.0f){
			it = states.erase(it);
			continue;
		}
		st.x0 += st.last.vx * dt;
		st.x1 += st.last.vx * dt;
		st.y0 += st.last.vy * dt;
		st.y1 += st.last.vy * dt;
		TrackedObject o = st.last;
		o.phase = "gone";
		o.misses++;
		o.ageFrames++;
		publish(st, o);
		objects.push_back(std::move(o));
		++it;
	}
}

} // namespace tracking
