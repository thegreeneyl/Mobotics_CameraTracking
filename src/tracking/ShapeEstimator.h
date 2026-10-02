#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "SegModel.h"
#include "TrackingTypes.h"

namespace tracking {

struct ShapeConfig {
	bool enabled = true;
	// Silhouette source per frame:
	//   0 mask        the detector's fused motion mask (what the boxes come from)
	//   1 foreground  the undilated MOG2 foreground inside the tracked box —
	//                 flat parts of the object without flow are still foreground
	//   2 model       instance segmentation (SegModel) on a crop around the
	//                 tracked box; falls back to `mask` while the model is
	//                 unavailable or has been silent for modelHoldFrames
	int source = 2;
	int gridW = 96, gridH = 32;  // occupancy grid pinned to the padded box
	float ema = 0.25f;           // per-frame blend of a new observation (1 = no smoothing)
	float threshold = 0.5f;      // occupancy that counts as object for the outline
	float padX = 0.04f;          // grid frame margin around the box, fraction of box w / h
	float padY = 0.15f;
	float simplify = 0.5f;       // polygon simplification tolerance (grid cells)
	int closeCells = 1;          // closing radius (grid cells) before the contour
	bool confirmedOnly = true;   // tentative tracks get no shape
	// --- model source
	int modelInputW = 640;       // network input (must match the ONNX export)
	int modelInputH = 192;
	float modelConf = 0.25f;     // class score floor
	float modelMinIou = 0.3f;    // instance box must overlap the tracked box this much
	float modelMaskThresh = 0.5f;
	float modelPad = 0.25f;      // crop margin around the box (fraction of its size)
	// A crop wider than modelMaxAspect (w/h) is cut into overlapping tiles,
	// each letterboxed on its own: a whole train squeezed into the 640x192
	// input is a 57 px ribbon the model does not see. One tile per model
	// call, round-robin per object; cells outside the tile stay unobserved.
	float modelMaxAspect = 6.5f;
	float modelTileOverlap = 0.15f;
	int modelEveryN = 2;         // run every Nth frame per object (staggered by id)
	int modelHoldFrames = 45;    // coast this long on a silent model before the fallback
	int modelMaxPerFrame = 2;    // crops per frame (largest objects first)
	// A model-derived shape is not overwritten by the fallback mask at the
	// full `ema` rate (that turned a train into a blob within three frames
	// when the model went silent); it drifts toward the fallback at this rate.
	float fallbackEma = 0.06f;
	std::string modelPath;       // resolved ONNX path (not a flat parameter)
	std::string modelFile;       // the path as written in the config
};

// Per-track silhouette estimation (see ObjectShape in TrackingTypes.h).
//
// Each tracked object owns a small occupancy grid pinned to its (padded)
// box. Every frame the chosen source mask is sampled into that grid —
// through every lane of the corridor that sees the object, so a dual-pane
// train is one shape — and blended into the grid with an EMA. Because the
// box itself is Kalman-smooth and travels with the object, the same grid
// cell keeps looking at the same part of the object from frame to frame:
// a roof that the mask only catches in half of the frames ends up solidly
// inside the shape instead of flickering, while wake pixels that come and
// go stay below the threshold. The outline is the simplified contour of
// the thresholded grid, published in corridor coordinates.
//
// Stopped objects keep their shape (a motion mask would fade on them).
// Lives on the tracking worker; no UI dependencies.
class ShapeEstimator {
public:
	struct LaneInput {
		const LaneConfig * lane = nullptr;
		const cv::Mat * mask = nullptr;       // fused motion mask, lane px, 8U
		const cv::Mat * foreground = nullptr; // detector's shape mask, lane px, 8U
		const cv::Mat * homography = nullptr; // pane px -> lane px, 3x3 CV_64F
		int laneW = 0, laneH = 0;
	};

	// Global settings (model path / input). Per-corridor overrides arrive
	// with every updateCorridor() call.
	void setConfig(const ShapeConfig & config);
	void reset();

	// True when the model source is wanted and the model could be loaded:
	// the caller then has to hand a BGR frame to beginFrame().
	bool wantsColor(const ShapeConfig & corridorCfg) const;

	// bgr: lens-corrected full frame (all panes side by side), BGR 8UC3, or
	// empty when no corridor wants the model.
	void beginFrame(const cv::Mat & bgr, int paneW, int paneH, uint64_t frameIndex);
	void updateCorridor(CorridorResult & cr, const ShapeConfig & cfg, const std::vector<LaneInput> & lanes);
	void endFrame(); // drops the state of objects that vanished

	double lastModelMs() const { return modelMsThisFrame; }
	bool modelLoaded() const { return model && model->isLoaded(); }

private:
	struct State {
		cv::Mat ema;              // 32F gridH x gridW, 0..1
		std::vector<uint8_t> obs; // last observation (0 / 255 / 128 unobserved)
		std::string lastSource;
		int modelMiss = 0;        // consecutive model frames without an instance
		int tileCursor = 0;       // next crop tile to run (round-robin)
		uint64_t lastSeen = 0;
		bool fresh = true;        // no observation yet: the first one seeds the grid
	};

	void ensureModel();
	// seen: optional 8U lane mask of the area actually looked at this call;
	// cells mostly outside it stay unobserved (kUnobserved) instead of empty.
	void sampleLane(const cv::Mat & laneMask, const LaneInput & li, const ObjectShape & frame, cv::Mat & obs,
	                const cv::Mat * seen = nullptr) const;
	bool runModel(const TrackedObject & o, const ShapeConfig & cfg, const LaneInput & li,
	              const ObjectShape & frame, State & st, cv::Mat & laneMask, cv::Mat & laneSeen);
	static void extractOutline(const cv::Mat & ema, const ShapeConfig & cfg, ObjectShape & shape);

	ShapeConfig globalCfg;
	std::unique_ptr<SegModel> model;
	bool modelFailed = false;
	std::map<int, State> states;

	// per-frame inputs
	cv::Mat frameBgr;
	int paneW = 0, paneH = 0;
	uint64_t frameIndex = 0;
	int modelRunsThisFrame = 0;
	double modelMsThisFrame = 0;
};

} // namespace tracking
