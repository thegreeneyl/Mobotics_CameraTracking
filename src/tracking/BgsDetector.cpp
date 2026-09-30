#include "BgsDetector.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace tracking {

BgsDetector::BgsDetector(const BgsConfig & config) : cfg(config){
	// detectShadows=true marks shadows as 127 in the mask so we can drop
	// them with a threshold instead of tracking them as objects.
	mog2 = cv::createBackgroundSubtractorMOG2(cfg.history, cfg.varThreshold, true);
	rebuildKernels();
}

void BgsDetector::rebuildKernels(){
	const int openPx = std::max(1, cfg.morphOpenPx);
	const int closePx = std::max(1, cfg.morphClosePx);
	openKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(openPx, openPx));
	closeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(closePx, closePx));
}

void BgsDetector::setConfig(const BgsConfig & config){
	const bool modelChanged = config.history != cfg.history
		|| config.varThreshold != cfg.varThreshold;
	const bool kernelsChanged = config.morphOpenPx != cfg.morphOpenPx
		|| config.morphClosePx != cfg.morphClosePx;
	cfg = config;
	if(modelChanged){
		mog2 = cv::createBackgroundSubtractorMOG2(cfg.history, cfg.varThreshold, true);
	}
	if(kernelsChanged) rebuildKernels();
}

void BgsDetector::reset(){
	mog2 = cv::createBackgroundSubtractorMOG2(cfg.history, cfg.varThreshold, true);
}

std::vector<Detection> BgsDetector::detect(const cv::Mat & lane, double){
	std::vector<Detection> detections;
	if(lane.empty()) return detections;

	mog2->apply(lane, fgMask, cfg.learningRate);

	// Foreground is 255, shadows are 127 — keep only real foreground.
	cv::threshold(fgMask, fgMask, 200, 255, cv::THRESH_BINARY);
	cv::morphologyEx(fgMask, fgMask, cv::MORPH_OPEN, openKernel);
	cv::morphologyEx(fgMask, fgMask, cv::MORPH_CLOSE, closeKernel);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(fgMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	const float laneW = static_cast<float>(lane.cols);
	const float laneH = static_cast<float>(lane.rows);
	const float laneArea = laneW * laneH;
	const float minArea = cfg.minAreaNorm * laneArea;
	const float maxArea = cfg.maxAreaNorm * laneArea;

	for(const auto & contour : contours){
		const cv::Rect box = cv::boundingRect(contour);
		const float area = static_cast<float>(box.area());
		if(area < minArea || area > maxArea) continue;

		Detection d;
		d.x0 = box.x / laneW;
		d.x1 = (box.x + box.width) / laneW;
		d.y0 = box.y / laneH;
		d.y1 = (box.y + box.height) / laneH;
		d.area = area / laneArea;
		d.touchesLeft = box.x <= 2;
		d.touchesRight = box.x + box.width >= lane.cols - 2;
		d.hasVelocity = false;
		detections.push_back(std::move(d));
	}
	return detections;
}

} // namespace tracking
