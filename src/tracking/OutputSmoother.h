#pragma once

#include <map>
#include <utility>
#include <vector>

#include "TrackingTypes.h"

namespace tracking {

// Publisher-side smoothing (tracking.smoothing in the config).
struct SmoothConfig {
	bool enabled = true;
	// A box edge may move at most this much (corridor fractions per
	// second) on top of the object's own travel. Merges, pin / release
	// transitions and a switch between blob and silhouette box become
	// slides instead of jumps. An edge the tracker pins at a corridor end
	// is exempt (it sits at the end, by definition).
	float boxRate = 0.25f;
	// The outline is resampled to this many vertices by arc length (from
	// a canonical start point) and each vertex is low-passed in
	// box-normalized coordinates.
	int outlineVertices = 32;
	float outlineAlpha = 0.3f; // per-frame blend of a new outline (1 = none)
	// presence ramps 0 -> 1 over fadeInMs after the first published frame
	// and 1 -> 0 over fadeOutMs after the tracker dropped the object; the
	// object keeps travelling at its last velocity while it fades out.
	double fadeInMs = 300;
	double fadeOutMs = 400;
};

// Sits between the tracker and the publisher. The tracker's job is to be
// right; this stage's job is that whatever it publishes never jumps: no
// box edge moves faster than boxRate, the outline is a fixed-size,
// low-passed polygon that is never empty (the box rectangle until the
// first silhouette arrives, the last silhouette while the model is
// silent), and objects appear and disappear through a presence ramp
// instead of popping. Operates on confirmed objects only; tentative ones
// pass through untouched (they are not published anyway).
//
// The output is a *presentation* of the tracker state. Nothing feeds back
// into the tracker.
class OutputSmoother {
public:
	void setConfig(const SmoothConfig & config){ cfg = config; }
	void reset(){ states.clear(); }

	// objects: this corridor's objects (corridor coordinates), modified in
	// place; dropped objects that are still fading out are appended with
	// phase "gone". tMs: frame timestamp.
	void apply(std::vector<TrackedObject> & objects, double tMs);

private:
	struct State {
		TrackedObject last;  // newest tracker state (for the fade-out coast)
		float x0 = 0, x1 = 0, y0 = 0, y1 = 0; // smoothed box
		std::vector<std::pair<float, float>> outline; // box-normalized u,v
		float presence = 0;
		double lastTMs = -1;
		bool seenThisFrame = false;
	};

	void initOutline(State & st) const;
	void blendOutline(State & st, const TrackedObject & o) const;
	void publish(const State & st, TrackedObject & o) const;

	SmoothConfig cfg;
	std::map<int, State> states;
	double lastTMs = -1;
};

// Resamples a closed polygon to n vertices by arc length. Exposed for the
// evaluation tooling / tests; see OutputSmoother.cpp for the start rule.
std::vector<std::pair<float, float>> resampleClosed(const std::vector<std::pair<float, float>> & poly, int n);

} // namespace tracking
