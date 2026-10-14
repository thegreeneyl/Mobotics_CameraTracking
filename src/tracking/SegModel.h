#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

namespace tracking {

// Instance segmentation (YOLOv8/YOLO11 *-seg ONNX via cv::dnn, CPU) on a
// crop around one tracked object. Used by the ShapeEstimator as its
// appearance-based silhouette source: the tracker decides where and when to
// look, the model only has to say which pixels of the crop are the object.
//
// Expects the Ultralytics v8-family segmentation export: output0
// [1, 4 + numClasses + 32, numAnchors] (box, class scores, mask
// coefficients) and output1 [1, 32, H/4, W/4] (prototype masks). Export
// with a fixed input (e.g. imgsz=[192, 640]) — the input size is read from
// the config, not from the model, and must match the export. A wide input
// suits the objects here: a whole train fits one crop without shrinking to
// a sliver, as it would in a square letterbox; the network is fully
// convolutional, so a rectangular input is legitimate.
class SegModel {
public:
	struct Result {
		cv::Mat mask;     // 8U, crop size, 0/255
		cv::Rect2f box;   // instance box in crop px (union of the used instances)
		float score = 0;  // class score
		int classId = -1; // COCO id
		float iou = 0;    // overlap with the requested target box
		// best instance on the target that is NOT of an allowed class (a car
		// where a train was expected); -1 when none. Set even when the call
		// returns false, so the caller can tell "nothing there" from
		// "something else there".
		int otherClass = -1;
		float otherScore = 0;
	};

	bool load(const std::string & path, int inputW, int inputH);
	bool isLoaded() const { return loaded; }
	const std::string & path() const { return modelPath; }

	// Segments the instance(s) that cover `target` (crop px). `allow` lists
	// the acceptable COCO class ids (empty = any class); candidates of other
	// classes are only reported via Result::otherClass. Candidates with
	// class score < conf or IoU(target) < minIou are dropped. Mask pixels
	// >= maskThresh (0..1) are set. Returns false when nothing qualifies.
	bool segment(const cv::Mat & bgrCrop, const cv::Rect2f & target, float conf, float minIou,
	             float maskThresh, const std::vector<int> & allow, Result & out);

	double lastForwardMs() const { return forwardMs; }

private:
	cv::dnn::Net net;
	std::vector<std::string> outNames;
	std::string modelPath;
	int inW = 640, inH = 192;
	bool loaded = false;
	double forwardMs = 0;
};

} // namespace tracking
