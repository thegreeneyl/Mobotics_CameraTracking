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
// A *lane* is a strip on one module pane (the viaduct band, the river band):
// a top and a bottom border, each a polyline of the same length, left to
// right. Two points per border is the old four-corner quad. Extra stations
// let the strip follow a curve. Each slice between stations is
// perspective-warped into the next section of one upright rectangle, so
// detections still come out in lane-normalized coordinates (x 0..1 across
// the lane, y 0..1 down the lane) and the published corridor stays
// rectangular.
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
	static constexpr int kMaxStations = 12; // points on each border, including the ends

	std::string id;        // "train_M1"
	int pane = 0;          // module pane index (0 = M1, 1 = M2)
	std::string corridor;  // corridor id; the corridor's label is the class
	FrameQuad quad;        // the four ends: TL, TR, BR, BL. Kept in sync with the borders.
	// Border polylines in pane UV, left to right, same count. Empty means
	// "just the quad" (top = TL->TR, bottom = BL->BR).
	std::vector<std::array<float, 2>> top;
	std::vector<std::array<float, 2>> bottom;
	float s0 = 0, s1 = 1;  // corridor axis range covered by lane x 0..1
	// Optional lateral band (lane y 0..1): detections whose vertical centre
	// lies outside it are dropped before tracking. Lets a quad stay generous
	// (lane height floor) while the road band behind the viaduct is ignored.
	float yBand0 = 0, yBand1 = 1;

	// Effective borders: the stored polylines, or the quad ends when unset.
	void borders(std::vector<std::array<float, 2>> & topOut,
	             std::vector<std::array<float, 2>> & bottomOut) const {
		if(top.size() >= 2 && top.size() == bottom.size()){
			topOut = top;
			bottomOut = bottom;
			return;
		}
		topOut = {quad.pts[0], quad.pts[1]};
		bottomOut = {quad.pts[3], quad.pts[2]};
	}
	int stationCount() const {
		return (top.size() >= 2 && top.size() == bottom.size()) ? static_cast<int>(top.size()) : 2;
	}
	bool hasYBand() const { return yBand0 > 0.0f || yBand1 < 1.0f; }

	bool operator==(const LaneConfig & o) const {
		if(id != o.id || pane != o.pane || corridor != o.corridor) return false;
		if(s0 != o.s0 || s1 != o.s1 || yBand0 != o.yBand0 || yBand1 != o.yBand1) return false;
		std::vector<std::array<float, 2>> ta, ba, tb, bb;
		borders(ta, ba);
		o.borders(tb, bb);
		return ta == tb && ba == bb;
	}
	bool operator!=(const LaneConfig & o) const { return !(*this == o); }

	void materializeBorders(){
		if(top.size() >= 2 && top.size() == bottom.size()) return;
		top = {quad.pts[0], quad.pts[1]};
		bottom = {quad.pts[3], quad.pts[2]};
	}
	void syncQuadEnds(){
		if(top.size() < 2 || top.size() != bottom.size()) return;
		quad.pts[0] = top.front();
		quad.pts[1] = top.back();
		quad.pts[2] = bottom.back();
		quad.pts[3] = bottom.front();
	}

	// Every slice TL,TR,BR,BL strictly convex and not collapsed, so each one
	// has a homography. A crossed strip must not be warped.
	bool isWarpable() const {
		std::vector<std::array<float, 2>> t, b;
		borders(t, b);
		if(t.size() < 2) return false;
		for(size_t i = 0; i + 1 < t.size(); i++){
			const std::array<float, 2> p[4] = {t[i], t[i + 1], b[i + 1], b[i]};
			const float span = std::hypot(t[i + 1][0] - t[i][0], t[i + 1][1] - t[i][1])
			                 + std::hypot(b[i + 1][0] - b[i][0], b[i + 1][1] - b[i][1]);
			if(span < 1e-3f) return false;
			for(int k = 0; k < 4; k++){
				const auto & a = p[k];
				const auto & c = p[(k + 1) % 4];
				const auto & d = p[(k + 2) % 4];
				const float cross = (c[0] - a[0]) * (d[1] - c[1]) - (c[1] - a[1]) * (d[0] - c[0]);
				if(cross <= 1e-6f) return false;
			}
		}
		return true;
	}

	// Rectified aspect (width / height) from the border lengths in pane pixels.
	// Two stations: identical to FrameQuad::aspect.
	float aspect(float panePxW, float panePxH) const {
		std::vector<std::array<float, 2>> t, b;
		borders(t, b);
		const auto len = [&](const std::array<float, 2> & p, const std::array<float, 2> & q){
			return std::hypot((q[0] - p[0]) * panePxW, (q[1] - p[1]) * panePxH);
		};
		float w = 0, h = 0;
		for(size_t i = 0; i + 1 < t.size(); i++){
			w += 0.5f * (len(t[i], t[i + 1]) + len(b[i], b[i + 1]));
		}
		for(size_t i = 0; i < t.size(); i++) h += len(t[i], b[i]);
		h /= static_cast<float>(std::max<size_t>(1, t.size()));
		if(w <= 0 || h <= 0) return panePxH > 0 ? panePxW / panePxH : 1.0f;
		return w / h;
	}

	// Centerline length and mean rib length, in pane pixels (the rectified size).
	void stripSize(float panePxW, float panePxH, float & widthPx, float & heightPx) const {
		std::vector<std::array<float, 2>> t, b;
		borders(t, b);
		const auto len = [&](const std::array<float, 2> & p, const std::array<float, 2> & q){
			return std::hypot((q[0] - p[0]) * panePxW, (q[1] - p[1]) * panePxH);
		};
		widthPx = 0;
		heightPx = 0;
		for(size_t i = 0; i + 1 < t.size(); i++){
			widthPx += 0.5f * (len(t[i], t[i + 1]) + len(b[i], b[i + 1]));
		}
		for(size_t i = 0; i < t.size(); i++) heightPx += len(t[i], b[i]);
		heightPx /= static_cast<float>(std::max<size_t>(1, t.size()));
	}

	void setStation(int edge, int index, float x, float y){
		materializeBorders();
		if(index < 0 || index >= static_cast<int>(top.size())) return;
		// 0..1 is the filled frame. Outside it is the recovered lens border.
		// The editor clamps to the real border; this only stops a runaway value.
		x = std::min(5.0f, std::max(-4.0f, x));
		y = std::min(5.0f, std::max(-4.0f, y));
		(edge == 0 ? top : bottom)[static_cast<size_t>(index)] = {x, y};
		syncQuadEnds();
	}

	// Split slice `seg` at fraction t. The clicked border gets (x, y); the
	// other border gets the point at the same fraction, so the new rib
	// matches. Returns the new station index, or -1.
	int insertStation(int edge, int seg, float t, float x, float y){
		materializeBorders();
		if(static_cast<int>(top.size()) >= kMaxStations) return -1;
		if(seg < 0 || seg + 1 >= static_cast<int>(top.size())) return -1;
		t = std::min(0.98f, std::max(0.02f, t));
		const auto lerp = [](const std::array<float, 2> & a, const std::array<float, 2> & b, float u){
			return std::array<float, 2>{a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u};
		};
		auto nt = lerp(top[static_cast<size_t>(seg)], top[static_cast<size_t>(seg + 1)], t);
		auto nb = lerp(bottom[static_cast<size_t>(seg)], bottom[static_cast<size_t>(seg + 1)], t);
		x = std::min(5.0f, std::max(-4.0f, x));
		y = std::min(5.0f, std::max(-4.0f, y));
		if(edge == 0) nt = {x, y};
		else nb = {x, y};
		const int at = seg + 1;
		top.insert(top.begin() + at, nt);
		bottom.insert(bottom.begin() + at, nb);
		syncQuadEnds();
		return at;
	}

	// Drops one station from both borders. The two ends stay.
	bool removeStation(int index){
		materializeBorders();
		if(index <= 0 || index >= static_cast<int>(top.size()) - 1) return false;
		top.erase(top.begin() + index);
		bottom.erase(bottom.begin() + index);
		syncQuadEnds();
		return true;
	}

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
	bool hasVelocity = false;  // false for a detector without flow
	float coherence = 1.0f;    // 0..1 flow agreement inside the blob
	float area = 0;            // moving-pixel area (normalized units^2), merge weight
	float confidence = 1.0f;
	bool touchesLeft = false, touchesRight = false; // clipped at a lane/corridor end
	int lane = -1;
	std::string label; // empty = unknown; a COCO class name when a model named it

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
	// bounding box of the outline (corridor coordinates); boxValid when the
	// outline exists
	bool boxValid = false;
	float bx0 = 0, bx1 = 0, by0 = 0, by1 = 0;
	// segmentation-model bookkeeping for this track (see ShapeFeedback)
	int modelCalls = 0;      // model runs on this track
	int modelHits = 0;       // runs that returned an instance of an allowed class
	int otherClassHits = 0;  // runs that saw only instances of other classes
	int modelAge = 1 << 20;  // frames since the last hit
	int modelClass = -1;     // COCO class of the last hit

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
	float x0 = 0, x1 = 0, y0 = 0, y1 = 0; // published bbox (see hullBox)
	float x = 0, y = 0;   // published center
	float w = 0, h = 0;   // published size
	float vx = 0, vy = 0; // velocity (fractions per second); always the flow velocity
	// Tracker box after output smoothing, before the result step. For a
	// boat whose outline is a real silhouette the published box above is
	// that outline's bounds and the center is the center of those bounds.
	// These stay the flow box, which the raw view draws. Nothing here is
	// fed back into the tracker. Trains are not replaced: the model
	// outline covers less of a train than the flow box does.
	float rawX0 = 0, rawX1 = 0, rawY0 = 0, rawY1 = 0;
	float rawX = 0, rawY = 0, rawW = 0, rawH = 0;
	std::vector<std::pair<float, float>> rawTrail; // flow-box centers
	bool hullBox = false; // published box came from the outline
	bool clippedLeft = false, clippedRight = false; // edge pinned at a corridor end
	// entering | crossing | exiting | moving | stopped | occluded | gone
	// (gone only from the output smoother, while the object fades out)
	std::string phase = "moving";
	int hits = 0;         // matched detections so far
	int misses = 0;       // consecutive frames without a match
	int ageFrames = 0;
	bool confirmed = false;
	// born clipped at / within edgeBirthFrac of a corridor end (or inherited
	// an id, or born right after a reset) — the edge rule's precondition
	bool edgeBorn = true;
	// multiplier the tracker applies to this object's flow velocity (see
	// TrackerConfig::flowGain); 1 when learning is off. Diagnostic only.
	float flowGain = 1.0f;
	// 0..1 visibility ramp of the output smoother (fade in at birth, fade
	// out after the tracker dropped the object); 1 straight from the tracker
	float presence = 1.0f;
	float colorR = 1, colorG = 1, colorB = 1; // stable per-id debug color
	std::vector<std::pair<float, float>> trail; // published centers, newest last
	ObjectShape shape; // silhouette (filled by the ShapeEstimator, not the tracker)
};

// What the ShapeEstimator learned about a track, handed back to the tracker
// one frame later. The tracker never reads the silhouette itself; this is
// the deliberate, narrow coupling: (a) the model gate — a tentative track the
// segmentation model never finds (or only labels as another class) does not
// confirm; (b) the shape-driven box — while a fresh model silhouette exists
// its bounding box is the tracker's position measurement, so the published
// box follows the hull and not the wake; (c) merge priority — a track with a
// silhouette is never folded into one without.
struct ShapeFeedback {
	bool modelAvailable = false; // a segmentation model runs for this corridor
	int modelCalls = 0;
	int modelHits = 0;
	int otherClassHits = 0;
	int modelAge = 1 << 20;      // frames since the last hit
	bool boxValid = false;       // silhouette bbox usable (model source, outline exists)
	float bx0 = 0, bx1 = 0, by0 = 0, by1 = 0; // corridor coordinates
	bool outlineValid = false;   // a model-derived outline exists
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
	int analysisW = 0;    // extended pane size the warp samples (px)
	int analysisH = 0;
	// Filled frame inside that pane. Lane UV 0..1 is this rect; a station
	// in the recovered border is outside 0..1. Zero inner size means the
	// pane is the filled frame (no border).
	int lensInnerX = 0, lensInnerY = 0, lensInnerW = 0, lensInnerH = 0;
	std::string detectorName; // IDetector::name() of the lane detectors ("flow")
	uint64_t revision = 0;    // increments per published frame
};

} // namespace tracking
