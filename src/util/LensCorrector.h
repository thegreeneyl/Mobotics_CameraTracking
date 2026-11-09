#pragma once

#include <opencv2/core.hpp>

// Fisheye undistortion applied to a camera frame before it is shown and
// before any lane warp or tracking. One set of coefficients describes the
// lens; a side-by-side BOTH frame (width >= 2x height) is two modules and
// each half is corrected on its own optical center.
//
// The model is OpenCV's equidistant fisheye. fovDeg is the horizontal field
// of view of one pane — the lens, not a zoom. k1..k4 are how the lens
// differs from equidistant (0 = pure equidistant of that field of view).
//
// The filled frame is not the sensor's aspect ratio. It spans the full
// width of the center row and the full height of the center column, in
// square pixels, which is the largest filled rectangle inside the
// undistorted sensor. frameFovDeg limits that width (0 = no limit, the
// whole sensor). An older `balance` key is ignored.
struct LensCorrection {
	bool enabled = true;
	float fovDeg = 92.55f;
	float k1 = 0.146f;
	float k2 = -0.143f;
	float k3 = 0.0f;
	float k4 = 0.0f;
	float frameFovDeg = 0.0f; // 0 = full sensor width
	float centerX = 0.5f;     // optical center, fraction of the pane
	float centerY = 0.5f;

	bool operator==(const LensCorrection & o) const {
		return enabled == o.enabled && fovDeg == o.fovDeg && k1 == o.k1 && k2 == o.k2
			&& k3 == o.k3 && k4 == o.k4 && frameFovDeg == o.frameFovDeg
			&& centerX == o.centerX && centerY == o.centerY;
	}
	bool operator!=(const LensCorrection & o) const { return !(*this == o); }
};

// One corrected pane. Lane UV 0..1 is the filled frame: innerX/innerY is
// that frame's top-left inside the output pane, and innerW/innerH is its
// size. The output matches the filled frame. A disabled correction is the
// source pane.
struct LensGeometry {
	int panes = 1;
	int outPaneW = 0;
	int outPaneH = 0;
	int innerX = 0;
	int innerY = 0;
	int innerW = 0;
	int innerH = 0;

	bool hasMargin() const {
		return innerW > 0 && innerH > 0 && outPaneW > 0 && outPaneH > 0
			&& (innerX > 0 || innerY > 0 || innerX + innerW < outPaneW || innerY + innerH < outPaneH);
	}

	// Image UV that still lands inside the output. 0..1 is the filled frame.
	void uvLimits(float & u0, float & v0, float & u1, float & v1) const {
		if(innerW <= 0 || innerH <= 0){
			u0 = v0 = 0.0f;
			u1 = v1 = 1.0f;
			return;
		}
		u0 = -static_cast<float>(innerX) / static_cast<float>(innerW);
		v0 = -static_cast<float>(innerY) / static_cast<float>(innerH);
		u1 = static_cast<float>(outPaneW - innerX) / static_cast<float>(innerW);
		v1 = static_cast<float>(outPaneH - innerY) / static_cast<float>(innerH);
	}
};

// Not thread-safe: one instance per thread (the display thread and the
// tracking worker each keep their own). Maps are cached for the pane size.
// apply() writes an image of panes * geometry().outPaneW by outPaneH.
class LensCorrector {
public:
	void set(const LensCorrection & correction){ params = correction; }

	const LensCorrection & get() const { return params; }

	// Valid after apply(). Identity (no border) until the first successful
	// correction, and whenever correction is disabled or the map fails.
	const LensGeometry & geometry() const { return geom; }

	// Writes the corrected frame into dst. The filled frame is the whole
	// pane (black only where the sensor has no pixel). A disabled
	// correction copies src through.
	void apply(const cv::Mat & src, cv::Mat & dst);

private:
	void ensureMaps(int paneW, int paneH);
	static LensGeometry identity(int panes, int paneW, int paneH);

	LensCorrection params;
	LensCorrection built;
	LensGeometry builtGeom; // matches map1/map2
	LensGeometry geom;      // published by the last apply()
	int builtW = 0;
	int builtH = 0;
	cv::Mat map1, map2;
};

// Moves a lane UV from the old balance-0 crop (the source aspect ratio)
// into the filled frame of the same lens. The pane size cancels out.
// False leaves the point unchanged.
bool migrateBalanceUv(const LensCorrection & lens, float & u, float & v);
