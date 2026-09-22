#pragma once

#include <string>
#include <vector>

#include <opencv2/video/background_segm.hpp>

#include "IDetector.h"

namespace tracking {

// Class-tag zone on a fixed camera: any detection whose centroid falls
// inside the band gets the zone's label (train corridor -> "train", river
// band -> "boat", road -> "car"). Coordinates are fractions of the pane:
// y0/y1 of pane HEIGHT, x0/x1 of pane WIDTH.
struct Zone {
	std::string label;
	float y0 = 0, y1 = 1;
	float x0 = 0, x1 = 1;
};

struct BgsConfig {
	int history = 500;         // MOG2 background model length (frames)
	double varThreshold = 16;  // MOG2 pixel variance threshold
	double learningRate = -1;  // -1 = automatic
	float minAreaNorm = 0.0005f; // min blob area as fraction of pane area
	float maxAreaNorm = 0.5f;    // max blob area (rejects lighting flips)
	int morphOpenPx = 3;   // opening kernel (kills speckle / water glint)
	int morphClosePx = 9;  // closing kernel (merges broken object parts)
};

// Background-subtraction detector: MOG2 -> shadow removal -> morphology ->
// contours -> area filter -> zone-based labels. Runs on a grayscale
// analysis pane at full frame rate; no ML model needed.
class BgsDetector : public IDetector {
public:
	BgsDetector(const BgsConfig & config, std::vector<Zone> zones);

	std::vector<Detection> detect(const cv::Mat & pane) override;
	void reset() override;
	bool wantsColor() const override { return false; }
	const char * name() const override { return "bgs"; }

private:
	BgsConfig cfg;
	std::vector<Zone> zones;
	cv::Ptr<cv::BackgroundSubtractorMOG2> mog2;
	cv::Mat fgMask;
	cv::Mat openKernel, closeKernel;
};

} // namespace tracking
