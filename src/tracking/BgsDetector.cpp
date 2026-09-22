#include "BgsDetector.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace tracking {

BgsDetector::BgsDetector(const BgsConfig & config, std::vector<Zone> zoneList)
	: cfg(config), zones(std::move(zoneList)){
	// detectShadows=true marks shadows as 127 in the mask so we can drop
	// them with a threshold instead of tracking them as objects.
	mog2 = cv::createBackgroundSubtractorMOG2(cfg.history, cfg.varThreshold, true);
	const int openPx = std::max(1, cfg.morphOpenPx);
	const int closePx = std::max(1, cfg.morphClosePx);
	openKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(openPx, openPx));
	closeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(closePx, closePx));
}

void BgsDetector::reset(){
	mog2 = cv::createBackgroundSubtractorMOG2(cfg.history, cfg.varThreshold, true);
}

std::vector<Detection> BgsDetector::detect(const cv::Mat & pane){
	std::vector<Detection> detections;
	if(pane.empty()) return detections;

	mog2->apply(pane, fgMask, cfg.learningRate);

	// Foreground is 255, shadows are 127 — keep only real foreground.
	cv::threshold(fgMask, fgMask, 200, 255, cv::THRESH_BINARY);
	// Opening kills isolated speckle (water glint, sensor noise), closing
	// merges fragments of the same object (a train split by windows).
	cv::morphologyEx(fgMask, fgMask, cv::MORPH_OPEN, openKernel);
	cv::morphologyEx(fgMask, fgMask, cv::MORPH_CLOSE, closeKernel);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	const float paneW = static_cast<float>(pane.cols);
	const float paneH = static_cast<float>(pane.rows);
	const float paneArea = paneW * paneH;
	const float minArea = cfg.minAreaNorm * paneArea;
	const float maxArea = cfg.maxAreaNorm * paneArea;

	for(const auto & contour : contours){
		const cv::Rect box = cv::boundingRect(contour);
		const float area = static_cast<float>(box.area());
		if(area < minArea || area > maxArea) continue;

		const float cx = box.x + box.width * 0.5f;
		const float cy = box.y + box.height * 0.5f;

		Detection d;
		d.x = cx / paneW;
		d.y = cy / paneW; // width-normalized on purpose (see TrackingTypes.h)
		d.w = box.width / paneW;
		d.h = box.height / paneW;
		d.confidence = 1.0f;

		// Zone label lookup uses the intuitive per-axis fractions.
		const float xFrac = cx / paneW;
		const float yFrac = cy / paneH;
		for(const auto & zone : zones){
			if(yFrac >= zone.y0 && yFrac <= zone.y1 && xFrac >= zone.x0 && xFrac <= zone.x1){
				d.label = zone.label;
				break;
			}
		}
		detections.push_back(std::move(d));
	}
	return detections;
}

} // namespace tracking
