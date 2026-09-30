#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "TrackingTypes.h"

namespace tracking {

// Per-frame object detector, swappable behind this interface. One instance
// per lane (each lane has its own background / flow state).
class IDetector {
public:
	virtual ~IDetector() = default;

	// lane: rectified lane image at analysis resolution. Grayscale 8U when
	// wantsColor() is false, BGR 8UC3 otherwise. dtSec: time since the
	// previous frame handed to this detector (0 for the first). Returned
	// detections are lane-normalized (x,y in 0..1 of lane width/height,
	// velocities in those fractions per second).
	virtual std::vector<Detection> detect(const cv::Mat & lane, double dtSec) = 0;

	virtual void reset() {}
	virtual bool wantsColor() const { return false; }
	virtual const char * name() const = 0;

	// Optional debug view: the binary motion/foreground mask of the last
	// detect() call at lane resolution (empty if the detector has none).
	virtual const cv::Mat & debugMask() const { return emptyMask; }

private:
	cv::Mat emptyMask;
};

} // namespace tracking
