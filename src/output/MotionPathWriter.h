#pragma once

#include <string>
#include <utility>
#include <vector>

#include "ofJson.h"

#include "../tracking/TrackingTypes.h"

// motion-path.v1 payload builder — the wire format the B3S FacadeEditor
// (packages/frontend/src/types/cameraTracking.ts) and, later, the Steuerung
// renderer consume. One JSON object per analysed frame; a recording is the
// ordered list of those objects under "frames".
//
// Coordinates are normalized 0..1 on the output canvas (the corridor's
// placement rectangle maps corridor x / lane y onto it; the box is cut at
// the canvas edges and objects entirely off the canvas are not published).
// vx/vy are canvas fractions per second, heading 0 deg = +x (right),
// 90 deg = +y (down). The uncut corridor-space box travels along in
// `corridor` for consumers that want the tracker's own view.
//
// No UI / OF-graphics dependencies (ofJson is the core nlohmann alias), so
// this can move into YOUniverse_Steuerung unchanged.
namespace output {

struct MotionObject {
	int id = 0;
	std::string view;      // corridor id ("train", "boat") or pane ("M1")
	std::string label;     // class: train | boat | ""
	float x = 0, y = 0;    // center
	float w = 0, h = 0;    // bbox size
	float vx = 0, vy = 0;  // fractions per second
	std::string phase = "moving"; // entering | crossing | exiting | moving
	bool confirmed = true;
	bool clippedLeft = false, clippedRight = false; // cut at a corridor end or the canvas edge
	std::vector<std::pair<float, float>> path; // recent centers, oldest first
	// The object in corridor space (x along the corridor, y lateral in the
	// lane), uncut — extra field for debugging / consumers that re-place.
	struct CorridorBox { float x = 0, y = 0, w = 0, h = 0, vx = 0, vy = 0; };
	CorridorBox corridor;
	// Per-pane back-projection [pane, x, y, w, h] in pane UV (0..1) for
	// overlay/debug — extra field, ignored by the motion-path.v1 parser.
	struct PaneBox { int pane = 0; float x = 0, y = 0, w = 0, h = 0; };
	std::vector<PaneBox> panes;
	// Silhouette as a closed polygon on the canvas (same space as bbox,
	// cut at the canvas edge like the box). Empty while the shape
	// estimator has nothing yet. Extra field, ignored by older parsers.
	std::vector<std::pair<float, float>> outline;
};

// Sutherland-Hodgman clip of a closed polygon to the unit canvas. Returns
// an empty polygon when nothing remains.
std::vector<std::pair<float, float>> clipPolygonToCanvas(const std::vector<std::pair<float, float>> & poly);

// Flattens a tracking snapshot: one MotionObject per tracked object of every
// corridor that still shows on the canvas after the corridor's placement,
// with per-lane back-projections in `panes` (lane quad -> pane UV via the
// quad's homography; y is the lane's lateral coordinate).
std::vector<MotionObject> fromTrackingResults(const tracking::TrackingResults & res);

// Builds a single frame datagram. tSec: recording-relative seconds; frame:
// 1-based index; fps: nominal rate; camera: source label; sizeW/H: the
// published space's pixel size (aspect only matters to consumers).
ofJson buildFrame(const std::vector<MotionObject> & objects, double tSec, int frame,
                  double fps, const std::string & camera, int sizeW, int sizeH,
                  double wallClockSec);

// Accumulates frames and writes the {"kind":"recording", ...} wrapper.
class MotionPathRecording {
public:
	MotionPathRecording(std::string source, double fps, int sizeW, int sizeH);

	void addFrame(const std::vector<MotionObject> & objects, double tSec, double wallClockSec);
	size_t frameCount() const { return frames.size(); }
	bool save(const std::string & path, const std::string & note = "") const;

private:
	std::string source;
	double fps;
	int sizeW, sizeH;
	std::vector<ofJson> frames;
	double lastTSec = 0;
};

// Coarse 8-way label for a heading in degrees (0 = right, 90 = down).
std::string headingLabel(double degrees);

} // namespace output
