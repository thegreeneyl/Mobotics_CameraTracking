#pragma once

#include <opencv2/core.hpp>

// Fisheye undistortion applied to a camera frame before it is shown and
// before any lane warp or tracking. One set of coefficients describes the
// lens; a side-by-side BOTH frame (width >= 2x height) is two modules and
// each half is corrected on its own optical center.
//
// The model is OpenCV's equidistant fisheye: fovDeg is the horizontal field
// of view of one pane and is the strength of the straightening. k1..k4 are
// the deviation from equidistant (0 = pure equidistant of that field of
// view). balance 0 fills the output (invalid corners cropped), 1 keeps
// every source pixel (black corners).
struct LensCorrection {
	bool enabled = true;
	float fovDeg = 170.0f;
	float k1 = 0.0f;
	float k2 = 0.0f;
	float k3 = 0.0f;
	float k4 = 0.0f;
	float balance = 0.0f;
	float centerX = 0.5f; // optical center, fraction of the pane
	float centerY = 0.5f;

	bool operator==(const LensCorrection & o) const {
		return enabled == o.enabled && fovDeg == o.fovDeg && k1 == o.k1 && k2 == o.k2
			&& k3 == o.k3 && k4 == o.k4 && balance == o.balance
			&& centerX == o.centerX && centerY == o.centerY;
	}
	bool operator!=(const LensCorrection & o) const { return !(*this == o); }
};

// Not thread-safe: one instance per thread (the display thread and the
// tracking worker each keep their own). Maps are cached for the pane size.
class LensCorrector {
public:
	void set(const LensCorrection & correction){ params = correction; }

	const LensCorrection & get() const { return params; }

	// Writes the corrected frame into dst (same size and type as src).
	// A disabled correction copies src through.
	void apply(const cv::Mat & src, cv::Mat & dst);

private:
	void ensureMaps(int paneW, int paneH);

	LensCorrection params;
	LensCorrection built;
	int builtW = 0;
	int builtH = 0;
	cv::Mat map1, map2;
};
