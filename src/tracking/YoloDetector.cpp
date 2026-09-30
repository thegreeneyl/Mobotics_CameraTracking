#include "YoloDetector.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "ofLog.h"

namespace tracking {

namespace {
// COCO class names, index = class id (YOLOv8/YOLO11 pretrained models).
const std::vector<std::string> kCocoNames = {
	"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
	"truck", "boat", "traffic light", "fire hydrant", "stop sign",
	"parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
	"elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
	"tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
	"baseball bat", "baseball glove", "skateboard", "surfboard",
	"tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
	"bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
	"hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
	"bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
	"keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
	"refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
	"hair drier", "toothbrush"
};
} // namespace

YoloDetector::YoloDetector(const YoloConfig & config) : cfg(config){
	try {
		net = cv::dnn::readNetFromONNX(cfg.modelPath);
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
		loaded = true;
	} catch(const cv::Exception & e){
		ofLogWarning("YoloDetector") << "failed to load " << cfg.modelPath << ": " << e.what();
		loaded = false;
		return;
	}

	classAllowed.assign(kCocoNames.size(), cfg.classFilter.empty());
	for(const auto & name : cfg.classFilter){
		const auto it = std::find(kCocoNames.begin(), kCocoNames.end(), name);
		if(it != kCocoNames.end()){
			classAllowed[static_cast<size_t>(it - kCocoNames.begin())] = true;
		}else{
			ofLogWarning("YoloDetector") << "unknown COCO class in filter: " << name;
		}
	}
}

std::vector<Detection> YoloDetector::detect(const cv::Mat & pane, double){
	std::vector<Detection> detections;
	if(!loaded || pane.empty()) return detections;

	const int inputSize = cfg.inputSize;
	const float paneW = static_cast<float>(pane.cols);
	const float paneH = static_cast<float>(pane.rows);

	// Letterbox to a square inputSize x inputSize canvas (pad value 114,
	// same as Ultralytics preprocessing).
	const float r = std::min(inputSize / paneW, inputSize / paneH);
	const int scaledW = static_cast<int>(std::round(paneW * r));
	const int scaledH = static_cast<int>(std::round(paneH * r));
	const int padX = (inputSize - scaledW) / 2;
	const int padY = (inputSize - scaledH) / 2;

	cv::Mat resized;
	cv::resize(pane, resized, cv::Size(scaledW, scaledH));
	cv::Mat canvas(inputSize, inputSize, CV_8UC3, cv::Scalar(114, 114, 114));
	resized.copyTo(canvas(cv::Rect(padX, padY, scaledW, scaledH)));

	cv::Mat blob = cv::dnn::blobFromImage(canvas, 1.0 / 255.0,
	                                      cv::Size(inputSize, inputSize),
	                                      cv::Scalar(), true, false);
	net.setInput(blob);

	cv::Mat out;
	try {
		out = net.forward();
	} catch(const cv::Exception & e){
		ofLogWarning("YoloDetector") << "forward failed: " << e.what();
		return detections;
	}

	// v8-family layout: [1, 4+numClasses, numAnchors] -> rows = anchors.
	if(out.dims != 3 || out.size[1] < 5) return detections;
	const int numAttrs = out.size[1];
	const int numClasses = numAttrs - 4;
	cv::Mat m = out.reshape(1, numAttrs); // numAttrs x numAnchors
	cv::Mat rows;
	cv::transpose(m, rows); // numAnchors x numAttrs

	std::vector<cv::Rect> boxes;
	std::vector<float> scores;
	std::vector<int> classIds;

	for(int i = 0; i < rows.rows; i++){
		const float * row = rows.ptr<float>(i);
		int bestClass = -1;
		float bestScore = 0;
		for(int c = 0; c < numClasses; c++){
			const float score = row[4 + c];
			if(score > bestScore){
				bestScore = score;
				bestClass = c;
			}
		}
		if(bestScore < cfg.confThreshold || bestClass < 0) continue;
		if(bestClass < static_cast<int>(classAllowed.size()) && !classAllowed[bestClass]) continue;

		// Undo the letterbox: back to pane pixel coordinates.
		const float cx = (row[0] - padX) / r;
		const float cy = (row[1] - padY) / r;
		const float w = row[2] / r;
		const float h = row[3] / r;
		boxes.emplace_back(static_cast<int>(cx - w / 2), static_cast<int>(cy - h / 2),
		                   static_cast<int>(w), static_cast<int>(h));
		scores.push_back(bestScore);
		classIds.push_back(bestClass);
	}

	std::vector<int> keep;
	cv::dnn::NMSBoxes(boxes, scores, cfg.confThreshold, cfg.nmsThreshold, keep);

	for(const int idx : keep){
		const cv::Rect & box = boxes[idx];
		Detection d;
		d.x0 = box.x / paneW;
		d.x1 = (box.x + box.width) / paneW;
		d.y0 = box.y / paneH;
		d.y1 = (box.y + box.height) / paneH;
		d.area = d.w() * d.h();
		d.touchesLeft = box.x <= 2;
		d.touchesRight = box.x + box.width >= static_cast<int>(paneW) - 2;
		d.hasVelocity = false;
		d.confidence = scores[idx];
		if(classIds[idx] >= 0 && classIds[idx] < static_cast<int>(kCocoNames.size())){
			d.label = kCocoNames[classIds[idx]];
		}
		detections.push_back(std::move(d));
	}
	return detections;
}

} // namespace tracking
