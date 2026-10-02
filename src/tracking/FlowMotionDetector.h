#pragma once

#include <vector>

#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/tracking.hpp>

#include "DetectionGrouping.h"
#include "IDetector.h"

namespace tracking {

struct FlowDetectorConfig {
	// --- motion evidence
	int preset = 1;             // DIS preset: 0 ultrafast, 1 fast, 2 medium
	float flowEma = 0.5f;       // temporal EMA of the flow field (1 = no smoothing)
	float minFlowPx = 0.6f;     // moving if |flow| > this (px per 1/30 s on the lane image)
	float sparkleRatio = 0.85f; // sparkle = instantaneous > thr but smoothed < thr*ratio
	float busyAlpha = 0.04f;    // busy-map rate (persistent sparkle => suppressed)
	float busyThresh = 0.35f;
	bool useMog2 = true;        // AND the flow mask with the MOG2 foreground
	int mogHistory = 300;
	float mogVarThreshold = 24.0f;
	int mogDilatePx = 7;        // foreground grown before the AND so edges survive
	// flow-seeded fill: MOG2 foreground within this reach (lane fractions) of
	// a moving pixel joins the mask even where the flow itself is below
	// threshold. Flat, textureless parts of an object (a train roof, a hull in
	// shadow) give no usable flow but are foreground — the flow blob is the
	// evidence that the object moves, the foreground draws its outline.
	// 0 = off (plain flow AND foreground).
	float fillWFrac = 0.0f;
	float fillHFrac = 0.0f;
	// --- mask cleanup
	int morphOpenPx = 3;
	float closeWFrac = 0.04f;   // anisotropic closing kernel width (of lane width)
	float closeHFrac = 0.30f;   // and height (of lane height)
	// --- blob filters
	float minAreaFrac = 0.003f;  // of lane area
	float minWidthFrac = 0.02f;  // of lane width
	float minHeightFrac = 0.10f; // of lane height
	float minCoherence = 0.45f;  // |median flow| / mean |flow|
	float minSpeed = 0.01f;      // lane widths per second
	float maxSpeed = 1.5f;       // clamp (lane widths per second)
	// --- grouping (blobs of one object)
	float mergeGapFrac = 0.15f;
	float mergeAngleDeg = 40.0f;
	float mergeSpeedRatio = 3.0f;
	float mergeLateralOverlap = 0.25f;
	// --- axis lock (trains): velocity is along the lane, vy is noise
	bool lockAxis = false;
	int maxDetections = 8;
	int edgeMarginPx = 3;        // blob within this of a lane end => touches
};

// Optical-flow motion detector: DIS dense flow (temporally smoothed) gives a
// per-pixel motion field; pixels moving faster than a threshold, not marked
// as persistently sparkling water, and (optionally) confirmed by MOG2
// foreground form the motion mask. Anisotropic closing bridges wagon gaps,
// connected components become blobs with a median flow vector, and blobs
// moving together are grouped into one detection per object.
//
// Output is lane-normalized: x,y in 0..1, vx in lane widths/s, vy in lane
// heights/s. Detections carry hasVelocity = true.
class FlowMotionDetector : public IDetector {
public:
	explicit FlowMotionDetector(const FlowDetectorConfig & config);

	void setConfig(const FlowDetectorConfig & config);
	const FlowDetectorConfig & config() const { return cfg; }

	std::vector<Detection> detect(const cv::Mat & lane, double dtSec) override;
	void reset() override;
	bool wantsColor() const override { return false; }
	const char * name() const override { return "flow"; }
	const cv::Mat & debugMask() const override { return mask; }
	// The undilated MOG2 foreground: inside a tracked box it is the better
	// silhouette (a flat roof is foreground even where it has no flow).
	const cv::Mat & shapeMask() const override { return cfg.useMog2 && !fgRaw.empty() ? fgRaw : mask; }

private:
	void rebuild();

	FlowDetectorConfig cfg;
	cv::Ptr<cv::DISOpticalFlow> dis;
	cv::Ptr<cv::BackgroundSubtractorMOG2> mog2;
	cv::Mat prev;      // previous lane image (8U)
	cv::Mat flow;      // instantaneous flow (32FC2, px/frame)
	cv::Mat flowEma;   // smoothed flow (32FC2, px per 1/30 s)
	cv::Mat busy;      // persistent-sparkle accumulator (32F)
	cv::Mat mask;      // final motion mask (8U)
	cv::Mat seedMask;  // flow-only mask when the fill is on (velocity source), else empty
	cv::Mat fg;        // MOG2 foreground (thresholded + dilated, ANDed with the flow)
	cv::Mat fgRaw;     // MOG2 foreground, thresholded only (shape source)
	cv::Mat openKernel;
};

} // namespace tracking
