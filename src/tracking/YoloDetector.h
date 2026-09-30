#pragma once

#include <string>
#include <vector>

#include <opencv2/dnn.hpp>

#include "IDetector.h"

namespace tracking {

struct YoloConfig {
	std::string modelPath; // absolute path to a YOLOv8/YOLO11 ONNX file
	int inputSize = 640;
	float confThreshold = 0.35f;
	float nmsThreshold = 0.45f;
	std::vector<std::string> classFilter; // COCO names to keep; empty = all
};

// Appearance-based detector: YOLOv8/YOLO11 ONNX via cv::dnn (CPU).
// Experiment mode — expensive (~100+ ms per pane), so the tracking thread's
// drop-stale input keeps the UI unaffected. Expects the v8-family output
// layout [1, 4+numClasses, numAnchors].
class YoloDetector : public IDetector {
public:
	explicit YoloDetector(const YoloConfig & config);

	bool isLoaded() const { return loaded; }

	// Live tuning: thresholds are read on the next detect().
	void setThresholds(float confThreshold, float nmsThreshold){
		cfg.confThreshold = confThreshold;
		cfg.nmsThreshold = nmsThreshold;
	}

	std::vector<Detection> detect(const cv::Mat & pane, double dtSec) override; // BGR lane image
	bool wantsColor() const override { return true; }
	const char * name() const override { return "yolo"; }

private:
	YoloConfig cfg;
	cv::dnn::Net net;
	bool loaded = false;
	std::vector<bool> classAllowed; // indexed by COCO class id
};

} // namespace tracking
