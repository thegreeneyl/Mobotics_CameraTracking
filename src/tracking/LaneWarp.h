#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "TrackingTypes.h"

namespace tracking {

// Piecewise perspective map of a lane strip onto one upright rectangle.
// Station 0 is the left end (lane x = 0). Along-track position is the
// centreline arc length, so a bend occupies its share of the rectangle.
// Two stations reproduce the single homography of the four-corner quad.
struct LaneMap {
	struct Seg {
		float u0 = 0, u1 = 1;     // lane x of the slice ends
		cv::Mat unitToPane;       // lane (u, v) -> pane px, 3x3 CV_64F
		cv::Mat paneToLane;       // pane px -> lane px, filled by prepare()
	};

	std::vector<Seg> segs;
	cv::Mat paneToLane;           // the one homography, only when segs.size()==1
	int laneW = 0, laneH = 0;
	float srcW = 0, srcH = 0;     // centreline / mean rib, pane px
	bool ok = false;

	// Geometry only (toPane). An unwarpable strip falls back to the full pane.
	// paneW/paneH are the pixel size of lane UV 0..1 (the filled frame).
	// originX/Y is that frame's top-left in the mat warp() samples, so a
	// station past 0..1 lands in the recovered border. Both default to a
	// mat that is the filled frame itself.
	static LaneMap build(const LaneConfig & lane, float paneW, float paneH,
	                     float originX = 0, float originY = 0);

	// Rectified pixel size, same rules as the old quad (cap width, floor height).
	void rectifiedSize(int maxW, int minH, int paneLimitH, int & w, int & h) const;

	// Fills per-slice pane->lane homographies for an output of dstW x dstH.
	void prepare(int dstW, int dstH);

	void warp(const cv::Mat & pane, cv::Mat & dst, int interpolation) const;

	// Lane-normalized (u, v) -> pane px (origin at the pane's top-left).
	bool toPane(float u, float v, float & x, float & y) const;

	// Axis-aligned pane-px bounds of a lane-normalized rectangle. Samples
	// every station the rectangle crosses, so a curve is enclosed.
	cv::Rect2f paneBounds(float u0, float u1, float v0, float v1) const;
};

} // namespace tracking
