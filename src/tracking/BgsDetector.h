#pragma once

#include <string>
#include <vector>

#include <opencv2/video/background_segm.hpp>

#include "IDetector.h"

namespace tracking {

struct BgsConfig {
	int history = 500;         // MOG2 background model length (frames)
	double varThreshold = 16;  // MOG2 pixel variance threshold
	double learningRate = -1;  // -1 = automatic
	float minAreaNorm = 0.0005f; // min blob area as fraction of lane area
	float maxAreaNorm = 0.5f;    // max blob area (rejects lighting flips)
	int morphOpenPx = 3;   // opening kernel (kills speckle / water glint)
	int morphClosePx = 9;  // closing kernel (merges broken object parts)
};

// Legacy background-subtraction detector: MOG2 -> shadow removal ->
// morphology -> contours -> area filter. No velocity measurement (the
// tracker then derives velocity from box motion only). Kept as a fallback
// and for A/B comparison against FlowMotionDetector.
class BgsDetector : public IDetector {
public:
	explicit BgsDetector(const BgsConfig & config);

	// Live tuning: area/learning-rate changes apply immediately; the MOG2
	// model is recreated (background relearns) only when history or
	// varThreshold change, kernels rebuild only when their sizes change.
	void setConfig(const BgsConfig & config);

	std::vector<Detection> detect(const cv::Mat & lane, double dtSec) override;
	void reset() override;
	bool wantsColor() const override { return false; }
	const char * name() const override { return "bgs"; }
	const cv::Mat & debugMask() const override { return fgMask; }

private:
	void rebuildKernels();

	BgsConfig cfg;
	cv::Ptr<cv::BackgroundSubtractorMOG2> mog2;
	cv::Mat fgMask;
	cv::Mat openKernel, closeKernel;
};

} // namespace tracking
