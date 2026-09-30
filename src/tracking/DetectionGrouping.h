#pragma once

#include <vector>

#include "TrackingTypes.h"

namespace tracking {

// Rules for deciding that two motion blobs belong to the same object.
struct GroupingParams {
	float gapFrac = 0.15f;         // max along-axis gap between the boxes
	float angleDeg = 40.0f;        // max velocity direction difference
	float speedRatio = 3.0f;       // max ratio of the two speeds
	float lateralOverlap = 0.25f;  // min y overlap / min(h_a, h_b)
	float minSpeed = 0.005f;       // below this a blob has "no direction"
};

// Do a and b agree in direction (both directionless counts as agreement)?
bool velocitiesAgree(const Detection & a, const Detection & b, const GroupingParams & p);

// Along-axis gap between two boxes (0 when they overlap in x).
float gapX(float ax0, float ax1, float bx0, float bx1);

// Lateral (y) overlap as a fraction of the smaller height.
float lateralOverlapFrac(float ay0, float ay1, float by0, float by1);

// Union-find grouping of detections that move together along the axis
// (same direction, similar speed, laterally overlapping, small gap). A train
// split at wagon gaps / masts, or the same train seen by two overlapping
// lanes, collapses into one detection: bbox = union, velocity = area-weighted
// mean, touches flags OR-ed. Output sorted by area, largest first.
std::vector<Detection> groupDetections(const std::vector<Detection> & in, const GroupingParams & p);

// Merge helper used by the tracker's one-to-many absorption as well.
Detection unionDetections(const Detection & a, const Detection & b);

} // namespace tracking
