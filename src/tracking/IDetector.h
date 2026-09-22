#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "TrackingTypes.h"

namespace tracking {

// Per-frame object detector, swappable behind this interface (user decision:
// BGS is the default runtime path, YOLO an optional experiment mode). One
// instance per optical module pane.
class IDetector {
public:
	virtual ~IDetector() = default;

	// pane: analysis-resolution image of one module. Grayscale 8U when
	// wantsColor() is false, BGR 8UC3 otherwise. Returned detections are
	// width-normalized (see TrackingTypes.h).
	virtual std::vector<Detection> detect(const cv::Mat & pane) = 0;

	virtual void reset() {}
	virtual bool wantsColor() const { return false; }
	virtual const char * name() const = 0;
};

} // namespace tracking
