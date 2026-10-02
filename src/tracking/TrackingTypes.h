#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

// Plain data types shared by the tracking pipeline.
//
// Geometry model
// --------------
// A *lane* is a four-corner quad on one module pane (the viaduct band, the
// river band). The pane is perspective-warped so the quad becomes an upright
// rectangle; only pixels inside reach the detector. Detections come out in
// lane-normalized coordinates (x 0..1 across the lane, y 0..1 down the lane).
//
// A *corridor* is the physical object path (the rail line, the river). One or
// more lanes feed a corridor; each lane maps its x range onto the corridor
// axis s in [s0, s1] (s0 > s1 flips the direction). In the single-module
// view a corridor has one lane; in the dual "BOTH" view the train corridor
// has one lane per pane with overlapping s ranges, so one train seen by both
// modules becomes ONE object.
//
// Tracking happens in corridor coordinates: x = s along the corridor (0 =
// corridor start, 1 = end), y = lateral 0..1 inside the lane, sizes are
// fractions of those, velocities fractions per second.
//
// Published coordinates are *canvas* coordinates: every corridor is placed
// on one output canvas (the proportions of the facade's Lines area) by a
// per-corridor placement rectangle; what falls off the canvas is cut.
//
// No UI / OF dependencies (same rule as src/mobotix/), so
// YOUniverse_Steuerung can reuse this later.
namespace tracking {

// Four corners in pane UV (fractions of pane width/height), order TL, TR,
// BR, BL. Default = the full pane.
struct FrameQuad {
	std::array<std::array<float, 2>, 4> pts{{{{0, 0}}, {{1, 0}}, {{1, 1}}, {{0, 1}}}};

	bool operator==(const FrameQuad & o) const { return pts == o.pts; }
	bool operator!=(const FrameQuad & o) const { return !(*this == o); }

	bool isFullPane(float eps = 1e-4f) const {
		const FrameQuad def{};
		for(int i = 0; i < 4; i++){
			if(std::fabs(pts[i][0] - def.pts[i][0]) > eps
			   || std::fabs(pts[i][1] - def.pts[i][1]) > eps) return false;
		}
		return true;
	}

	// Strictly convex in TL->TR->BR->BL (clockwise, y down) order with
	// non-trivial area. A crossed or collapsed quad fails and must not be
	// used for warping.
	bool isConvex() const {
		for(int i = 0; i < 4; i++){
			const auto & a = pts[i];
			const auto & b = pts[(i + 1) % 4];
			const auto & c = pts[(i + 2) % 4];
			const float cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
			if(cross <= 1e-6f) return false;
		}
		return true;
	}

	// Average edge lengths in pane pixels -> rectified aspect (w/h).
	float aspect(float panePxW, float panePxH) const {
		const auto len = [&](int a, int b){
			return std::hypot((pts[b][0] - pts[a][0]) * panePxW, (pts[b][1] - pts[a][1]) * panePxH);
		};
		const float w = (len(0, 1) + len(3, 2)) * 0.5f;
		const float h = (len(0, 3) + len(1, 2)) * 0.5f;
		if(w <= 0 || h <= 0) return panePxH > 0 ? panePxW / panePxH : 1.0f;
		return w / h;
	}
};

// One lane of a layout (see header comment).
struct LaneConfig {
	std::string id;        // "train_M1"
	int pane = 0;          // module pane index (0 = M1, 1 = M2)
	std::string corridor;  // corridor id; the corridor's label is the class
	FrameQuad quad;
	float s0 = 0, s1 = 1;  // corridor axis range covered by lane x 0..1

	bool operator==(const LaneConfig & o) const {
		return id == o.id && pane == o.pane && corridor == o.corridor && quad == o.quad
			&& s0 == o.s0 && s1 == o.s1;
	}
	bool operator!=(const LaneConfig & o) const { return !(*this == o); }

	// lane x (0..1) -> corridor s
	float toCorridor(float x) const { return s0 + x * (s1 - s0); }
	// corridor s -> lane x (may fall outside 0..1)
	float toLane(float s) const {
		const float d = s1 - s0;
		return std::fabs(d) < 1e-6f ? 0.0f : (s - s0) / d;
	}
	bool reversed() const { return s1 < s0; }
	// Corridor range [sa, sb] -> visible lane x range clipped to 0..1.
	// Returns false when the range does not intersect this lane.
	bool corridorRangeToLane(float sa, float sb, float & x0, float & x1) const {
		float a = toLane(sa), b = toLane(sb);
		if(a > b) std::swap(a, b);
		x0 = std::max(0.0f, a);
		x1 = std::min(1.0f, b);
		return x1 > x0;
	}
	// does the lane's left / right end coincide with a corridor end?
	bool leftIsCorridorEnd() const { return reversed() ? s0 >= 0.999f : s0 <= 0.001f; }
	bool rightIsCorridorEnd() const { return reversed() ? s1 <= 0.001f : s1 >= 0.999f; }
};

// Where a corridor lands on the output canvas, in canvas-normalized units
// (0,0 = canvas top-left, 1,1 = bottom-right). Corridor x 0..1 maps onto
// canvas X x0..x1, lane y 0..1 onto canvas Y y0..y1. Values outside 0..1
// are allowed and useful: with x0 = -0.2 the first fifth of the corridor
// lies off the canvas and is cut; x1 < x0 mirrors the direction of travel.
struct CorridorPlacement {
	float x0 = 0, x1 = 1, y0 = 0, y1 = 1;

	bool operator==(const CorridorPlacement & o) const {
		return x0 == o.x0 && x1 == o.x1 && y0 == o.y0 && y1 == o.y1;
	}
	bool operator!=(const CorridorPlacement & o) const { return !(*this == o); }

	float toCanvasX(float s) const { return x0 + s * (x1 - x0); }
	float toCanvasY(float y) const { return y0 + y * (y1 - y0); }
	float spanX() const { return x1 - x0; }
	float spanY() const { return y1 - y0; }
	bool mirrored() const { return x1 < x0; }

	// Corridor box -> canvas box (edges sorted, NOT clipped).
	void mapBox(float sx0, float sx1, float sy0, float sy1,
	            float & cx0, float & cx1, float & cy0, float & cy1) const {
		cx0 = toCanvasX(sx0); cx1 = toCanvasX(sx1);
		cy0 = toCanvasY(sy0); cy1 = toCanvasY(sy1);
		if(cx0 > cx1) std::swap(cx0, cx1);
		if(cy0 > cy1) std::swap(cy0, cy1);
	}
	// Clip a canvas box to the canvas; false when nothing remains.
	static bool clipToCanvas(float & cx0, float & cx1, float & cy0, float & cy1){
		cx0 = std::max(0.0f, cx0); cx1 = std::min(1.0f, cx1);
		cy0 = std::max(0.0f, cy0); cy1 = std::min(1.0f, cy1);
		return cx1 > cx0 && cy1 > cy0;
	}
};

// Pixel proportions of the output canvas (only the aspect matters to
// consumers). Default: the facade's Lines area.
struct CanvasConfig {
	int width = 832;
	int height = 442;
	bool operator==(const CanvasConfig & o) const { return width == o.width && height == o.height; }
	bool operator!=(const CanvasConfig & o) const { return !(*this == o); }
};

// A named set of lanes for one camera layout (pane count + optional
// recording names it applies to).
struct LayoutConfig {
	std::string name;                     // "single", "dual", ...
	int panes = 1;                        // 1 or 2
	std::vector<std::string> recordings;  // optional: only for these sources
	std::vector<LaneConfig> lanes;
};

// One detector hit. Coordinates are lane-normalized when produced by a
// detector and corridor-normalized after TrackingManager mapped them.
struct Detection {
	float x0 = 0, x1 = 0, y0 = 0, y1 = 0; // bbox edges
	float vx = 0, vy = 0;      // measured velocity (units per second)
	bool hasVelocity = false;  // false for detectors without flow (bgs/yolo)
	float coherence = 1.0f;    // 0..1 flow agreement inside the blob
	float area = 0;            // moving-pixel area (normalized units^2), merge weight
	float confidence = 1.0f;
	bool touchesLeft = false, touchesRight = false; // clipped at a lane/corridor end
	int lane = -1;
	std::string label; // empty = unknown (bgs/flow); COCO name for yolo

	float cx() const { return (x0 + x1) * 0.5f; }
	float cy() const { return (y0 + y1) * 0.5f; }
	float w() const { return x1 - x0; }
	float h() const { return y1 - y0; }
	float speed() const { return std::hypot(vx, vy); }
};

// The silhouette of a tracked object. Accumulated per track in a small
// occupancy grid pinned to the tracked box (plus a padding margin): grid
// u 0..1 runs along the box from x0 to x1, v 0..1 from y0 to y1, so the
// shape of a rigid object stays put in grid coordinates while the box
// travels — a temporal EMA of the grid then averages out the per-frame
// mask flicker without smearing the shape. See ShapeEstimator.
struct ObjectShape {
	bool valid = false;
	int gridW = 0, gridH = 0;
	// grid frame in corridor coordinates (the tracked box plus padding)
	float fx0 = 0, fx1 = 0, fy0 = 0, fy1 = 0;
	std::vector<uint8_t> ema; // accumulated occupancy 0..255, row-major
	// the newest observation that fed the grid: 0 background, 255 object,
	// 128 = not observed (outside every lane / no source this frame)
	std::vector<uint8_t> obs;
	// closed polygon in corridor coordinates, derived from the thresholded
	// grid; empty when the grid holds nothing yet
	std::vector<std::pair<float, float>> outline;
	std::string source; // "mask" | "foreground" | "model" — what fed obs

	float frameW() const { return fx1 - fx0; }
	float frameH() const { return fy1 - fy0; }
	// grid cell (gu, gv in cell units, may be fractional) -> corridor coords
	float toS(float gu) const { return fx0 + gu / std::max(1, gridW) * frameW(); }
	float toY(float gv) const { return fy0 + gv / std::max(1, gridH) * frameH(); }
};

// One tracked object with a stable unique id (corridor coordinates).
struct TrackedObject {
	int id = 0;
	std::string label;    // corridor label (class): train | boat | ...
	std::string corridor; // corridor id
	float x0 = 0, x1 = 0, y0 = 0, y1 = 0; // filtered bbox edges
	float x = 0, y = 0;   // center
	float w = 0, h = 0;   // size
	float vx = 0, vy = 0; // velocity (fractions per second)
	bool clippedLeft = false, clippedRight = false; // edge pinned at a corridor end
	std::string phase = "moving"; // entering | crossing | exiting | moving
	int hits = 0;         // matched detections so far
	int misses = 0;       // consecutive frames without a match
	int ageFrames = 0;
	bool confirmed = false;
	float colorR = 1, colorG = 1, colorB = 1; // stable per-id debug color
	std::vector<std::pair<float, float>> trail; // recent centers, newest last
	ObjectShape shape; // silhouette (filled by the ShapeEstimator, not the tracker)
};

struct CorridorResult {
	std::string id;
	std::string label;
	std::vector<int> lanes; // indices into TrackingResults::lanes
	CorridorPlacement placement; // where this corridor sits on the canvas
	std::vector<TrackedObject> objects; // corridor coordinates
};

// Per-lane debug info for overlays: geometry + the detector's fused motion
// mask at lane resolution + this frame's raw detections (lane-normalized).
struct LaneDebug {
	LaneConfig lane;
	int maskW = 0, maskH = 0;
	std::vector<uint8_t> mask; // 0/255, row-major
	std::vector<Detection> detections;
	double detectMs = 0;
};

// Snapshot published by the tracking thread after each analysed frame.
struct TrackingResults {
	std::vector<CorridorResult> corridors;
	std::vector<LaneDebug> lanes;
	std::string layoutName;
	int paneCount = 1;
	CanvasConfig canvas;  // output canvas proportions
	double frameTMs = 0;  // timestamp of the analysed frame
	double decodeMs = 0;  // analysis decode cost
	double detectMs = 0;  // detector cost (all lanes)
	double trackMs = 0;   // tracker cost (all corridors)
	double shapeMs = 0;   // shape estimator cost (incl. segmentation model)
	int analysisW = 0;    // per-pane analysis resolution (px)
	int analysisH = 0;
	std::string detectorName; // "flow" | "bgs" | "yolo"
	uint64_t revision = 0;    // increments per published frame
};

} // namespace tracking
