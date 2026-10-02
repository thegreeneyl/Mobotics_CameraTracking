#include "SegModel.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "ofLog.h"

namespace tracking {

namespace {
constexpr int kMaskCoeffs = 32;

double nowMs(){
	using namespace std::chrono;
	return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

float rectIou(const cv::Rect2f & a, const cv::Rect2f & b){
	const float inter = (a & b).area();
	const float uni = a.area() + b.area() - inter;
	return uni <= 0 ? 0.0f : inter / uni;
}
} // namespace

bool SegModel::load(const std::string & path, int inputW, int inputH){
	loaded = false;
	modelPath = path;
	inW = std::max(64, inputW);
	inH = std::max(64, inputH);
	try {
		net = cv::dnn::readNetFromONNX(path);
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
		outNames = net.getUnconnectedOutLayersNames();
		loaded = !outNames.empty();
	} catch(const cv::Exception & e){
		ofLogWarning("SegModel") << "failed to load " << path << ": " << e.what();
		return false;
	}
	if(!loaded) ofLogWarning("SegModel") << path << ": no outputs";
	return loaded;
}

bool SegModel::segment(const cv::Mat & crop, const cv::Rect2f & target, float conf, float minIou,
                       float maskThresh, Result & out){
	if(!loaded || crop.empty() || crop.type() != CV_8UC3) return false;

	// letterbox (Ultralytics preprocessing: pad 114, RGB, 0..1)
	const float cropW = static_cast<float>(crop.cols), cropH = static_cast<float>(crop.rows);
	const float r = std::min(inW / cropW, inH / cropH);
	const int scaledW = std::max(1, static_cast<int>(std::round(cropW * r)));
	const int scaledH = std::max(1, static_cast<int>(std::round(cropH * r)));
	const int padX = (inW - scaledW) / 2;
	const int padY = (inH - scaledH) / 2;
	cv::Mat resized;
	cv::resize(crop, resized, cv::Size(scaledW, scaledH));
	cv::Mat canvas(inH, inW, CV_8UC3, cv::Scalar(114, 114, 114));
	resized.copyTo(canvas(cv::Rect(padX, padY, scaledW, scaledH)));
	cv::Mat blob = cv::dnn::blobFromImage(canvas, 1.0 / 255.0, cv::Size(inW, inH),
	                                      cv::Scalar(), true, false);

	std::vector<cv::Mat> outs;
	const double t0 = nowMs();
	try {
		net.setInput(blob);
		net.forward(outs, outNames);
	} catch(const cv::Exception & e){
		ofLogWarning("SegModel") << "forward failed: " << e.what();
		return false;
	}
	forwardMs = nowMs() - t0;

	// identify detections [1, attrs, anchors] and prototypes [1, 32, ph, pw]
	const cv::Mat * det = nullptr;
	const cv::Mat * proto = nullptr;
	for(const auto & o : outs){
		if(o.dims == 3 && o.size[1] > 4 + kMaskCoeffs) det = &o;
		else if(o.dims == 4 && o.size[1] == kMaskCoeffs) proto = &o;
	}
	if(!det || !proto) return false;
	const int numAttrs = det->size[1];
	const int numAnchors = det->size[2];
	const int numClasses = numAttrs - 4 - kMaskCoeffs;
	if(numClasses <= 0) return false;
	cv::Mat rows;
	cv::transpose(det->reshape(1, numAttrs), rows); // anchors x attrs

	// Candidates: every instance that overlaps the target enough (IoU) or
	// lies mostly inside it (a long train is often cut into two or three
	// instances; the tracked box says they are one object). Their masks
	// are unioned, so no NMS is needed.
	struct Cand { int row; cv::Rect2f box; float score; int cls; float iou; float rank; };
	std::vector<Cand> cands;
	for(int i = 0; i < numAnchors; i++){
		const float * row = rows.ptr<float>(i);
		int cls = -1;
		float score = 0;
		for(int c = 0; c < numClasses; c++){
			if(row[4 + c] > score){ score = row[4 + c]; cls = c; }
		}
		if(score < conf) continue;
		const float cx = (row[0] - padX) / r, cy = (row[1] - padY) / r;
		const float w = row[2] / r, h = row[3] / r;
		const cv::Rect2f box(cx - w / 2, cy - h / 2, w, h);
		if(box.area() <= 0) continue;
		const float iou = rectIou(box, target);
		const float inside = (box & target).area() / box.area();
		// an "inside" instance must be a real part of the object, not a
		// speck (a person on deck, a reflection) — at least 3 % of the box
		const bool part = inside >= 0.6f && box.area() >= 0.03f * target.area();
		if(iou < minIou && !part) continue;
		cands.push_back({i, box, score, cls, iou, std::max(iou, inside * 0.5f) * (0.5f + 0.5f * score)});
	}
	if(cands.empty()) return false;
	std::sort(cands.begin(), cands.end(), [](const Cand & a, const Cand & b){ return a.rank > b.rank; });
	constexpr size_t kMaxInstances = 6;
	if(cands.size() > kMaxInstances) cands.resize(kMaxInstances);

	// mask = sigmoid(coeffs . protos) on the stride-4 grid of the input,
	// back to the letterboxed input, then to the crop; each instance is
	// kept inside its own box (+ margin) because the prototype mask bleeds
	const int ph = proto->size[2], pw = proto->size[3];
	const cv::Mat protos(kMaskCoeffs, ph * pw, CV_32F, const_cast<float *>(proto->ptr<float>()));
	const cv::Rect inner(padX, padY, scaledW, scaledH);
	cv::Mat mask = cv::Mat::zeros(crop.size(), CV_8U);
	cv::Rect2f unionBox;
	for(size_t k = 0; k < cands.size(); k++){
		const Cand & c = cands[k];
		const cv::Mat coeffs(1, kMaskCoeffs, CV_32F, const_cast<float *>(rows.ptr<float>(c.row) + 4 + numClasses));
		cv::Mat m = coeffs * protos; // 1 x (ph*pw)
		m = m.reshape(1, ph);
		cv::exp(-m, m);
		m = 1.0f / (1.0f + m);
		cv::Mat full, cropProb;
		cv::resize(m, full, cv::Size(inW, inH), 0, 0, cv::INTER_LINEAR);
		cv::resize(full(inner), cropProb, crop.size(), 0, 0, cv::INTER_LINEAR);
		cv::Mat inst = cropProb >= maskThresh;
		const int mx = std::max(0, static_cast<int>(std::floor(c.box.x - 2)));
		const int my = std::max(0, static_cast<int>(std::floor(c.box.y - 2)));
		const int mx1 = std::min(crop.cols, static_cast<int>(std::ceil(c.box.x + c.box.width + 2)));
		const int my1 = std::min(crop.rows, static_cast<int>(std::ceil(c.box.y + c.box.height + 2)));
		if(mx1 <= mx || my1 <= my) continue;
		const cv::Rect keep(mx, my, mx1 - mx, my1 - my);
		mask(keep) |= inst(keep);
		unionBox = k == 0 ? c.box : (unionBox | c.box);
	}

	out.mask = mask;
	out.box = unionBox;
	out.score = cands.front().score;
	out.classId = cands.front().cls;
	out.iou = cands.front().iou;
	return true;
}

} // namespace tracking
