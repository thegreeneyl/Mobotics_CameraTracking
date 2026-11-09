#include "ofApp.h"

#include <algorithm>
#include <cmath>

#include "output/MotionPathWriter.h"
#include "tracking/LaneWarp.h"

using Mode = CameraTrackingSession::Mode;

namespace {
constexpr float kFrameHandleRadius = 6;     // drawn size
constexpr float kFrameHandleHitRadius = 12; // mouse pick radius
constexpr float kMaxArrow = 0.25f;          // velocity arrow clamp (lane widths)
constexpr float kBarHeight = 64;            // stats bar at the bottom

// Lane-normalized (u, v) -> screen, through the strip's piecewise homography.
// The map was built in the pane's own pixel space, so pane-local px plus the
// pane origin is the screen position.
glm::vec2 projectLane(const tracking::LaneMap & map, const ofRectangle & pane, float u, float v){
	float x, y;
	if(!map.toPane(u, v, x, y)) return {pane.x, pane.y};
	return {pane.x + x, pane.y + y};
}

// Closest point on either border polyline. t is the fraction along that slice.
bool nearestBorder(const tracking::LaneConfig & lane, const ofRectangle & pane,
                   float x, float y, int & edge, int & seg, float & t, float & dist){
	std::vector<std::array<float, 2>> top, bottom;
	lane.borders(top, bottom);
	dist = 1e9f;
	bool hit = false;
	const auto consider = [&](int e, const std::vector<std::array<float, 2>> & pts){
		for(size_t i = 0; i + 1 < pts.size(); i++){
			const float x0 = pane.x + pts[i][0] * pane.width;
			const float y0 = pane.y + pts[i][1] * pane.height;
			const float x1 = pane.x + pts[i + 1][0] * pane.width;
			const float y1 = pane.y + pts[i + 1][1] * pane.height;
			const float dx = x1 - x0, dy = y1 - y0;
			const float len2 = dx * dx + dy * dy;
			if(len2 < 1.0f) continue;
			float u = ((x - x0) * dx + (y - y0) * dy) / len2;
			u = std::min(1.0f, std::max(0.0f, u));
			const float px = x0 + u * dx, py = y0 + u * dy;
			const float d = std::hypot(x - px, y - py);
			if(d < dist){
				dist = d;
				edge = e;
				seg = static_cast<int>(i);
				t = u;
				hit = true;
			}
		}
	};
	consider(0, top);
	consider(1, bottom);
	return hit;
}

// Stable colour per corridor id for the canvas view.
ofColor corridorColor(const std::string & id){
	uint32_t h = 2166136261u;
	for(unsigned char c : id) h = (h ^ c) * 16777619u;
	ofColor col;
	col.setHsb((h % 256), 150, 240);
	return col;
}

// Clip all drawing to a screen rectangle (OF has no scissor helper).
void beginScissor(const ofRectangle & r){
	glEnable(GL_SCISSOR_TEST);
	const float s = ofGetWindowWidth() > 0 ? static_cast<float>(ofGetViewportWidth()) / ofGetWindowWidth() : 1.0f;
	glScissor(static_cast<GLint>(r.x * s), static_cast<GLint>((ofGetWindowHeight() - r.getBottom()) * s),
	          static_cast<GLsizei>(r.width * s), static_cast<GLsizei>(r.height * s));
}
void endScissor(){
	glDisable(GL_SCISSOR_TEST);
}

std::string formatClock(double ms){
	const int totalSec = static_cast<int>(ms / 1000.0);
	const int m = totalSec / 60;
	const int s = totalSec % 60;
	return ofToString(m, 2, '0') + ":" + ofToString(s, 2, '0');
}

// ofTexture::loadData only reallocates when the new frame is LARGER than the
// current allocation; a smaller frame is uploaded into a sub-region and the
// texture keeps its old size (and stale pixels outside the region). Frame
// sizes change when playback cycles between dual- and single-camera
// recordings — so reallocate whenever the dimensions differ.
void loadFrameTexture(ofTexture & tex, const ofPixels & pixels){
	if(tex.isAllocated()
	   && (static_cast<size_t>(tex.getWidth()) != pixels.getWidth()
	       || static_cast<size_t>(tex.getHeight()) != pixels.getHeight())){
		tex.clear();
	}
	tex.loadData(pixels);
}

// Draws one corrected pane. paneOut is the bright filled frame (lane UV
// 0..1). The recovered border around it is veiled, so the pixels the crop
// had thrown away stay visible without taking over the picture.
void drawCorrectedPane(const ofTexture & tex, float texX, float texW, float texH,
                       float x, float y, float w, float h,
                       int innerX, int innerY, int innerW, int innerH,
                       ofRectangle & paneOut){
	ofSetColor(255);
	tex.drawSubsection(x, y, w, h, texX, 0, texW, texH);
	const bool ghost = innerW > 0 && innerH > 0 && texW > 0 && texH > 0
		&& (innerX > 0 || innerY > 0 || innerX + innerW < texW - 0.5f || innerY + innerH < texH - 0.5f);
	if(!ghost){
		paneOut.set(x, y, w, h);
		return;
	}
	const float sx = w / texW;
	const float sy = h / texH;
	paneOut.set(x + innerX * sx, y + innerY * sy, innerW * sx, innerH * sy);
	ofEnableAlphaBlending();
	ofSetColor(0, 0, 0, 150);
	ofFill();
	const float left = paneOut.x - x;
	const float right = (x + w) - paneOut.getRight();
	const float top = paneOut.y - y;
	const float bottom = (y + h) - paneOut.getBottom();
	if(left > 0.5f) ofDrawRectangle(x, y, left, h);
	if(right > 0.5f) ofDrawRectangle(paneOut.getRight(), y, right, h);
	if(top > 0.5f) ofDrawRectangle(paneOut.x, y, paneOut.width, top);
	if(bottom > 0.5f) ofDrawRectangle(paneOut.x, paneOut.getBottom(), paneOut.width, bottom);
	ofSetColor(255);
}

// Stations stay inside the recovered picture (the bright frame plus the
// dim border). Past that there is no sensor pixel.
void clampLaneDrag(const LensGeometry & g, const ofRectangle & pane, int x, int y, float & u, float & v){
	u = pane.width > 0 ? (x - pane.x) / pane.width : 0.0f;
	v = pane.height > 0 ? (y - pane.y) / pane.height : 0.0f;
	float u0, v0, u1, v1;
	g.uvLimits(u0, v0, u1, v1);
	u = std::min(u1, std::max(u0, u));
	v = std::min(v1, std::max(v0, v));
}

ofColor objectColor(const tracking::TrackedObject & obj){
	return ofColor(obj.colorR * 255, obj.colorG * 255, obj.colorB * 255, obj.confirmed ? 255 : 90);
}

// Maps a corridor-space point (s along the corridor, y lateral) to the
// screen; returns false where the view does not show that point.
using CorridorToScreen = std::function<bool(float s, float y, glm::vec2 & out)>;

// The object's current shape pixels (the newest observation that fed its
// grid) as filled cells, drawn through an arbitrary corridor->screen map so
// the same code serves the canvas, the lane strips and the photo overlay.
void drawShapeCells(const tracking::ObjectShape & sh, const CorridorToScreen & map, const ofColor & col){
	if(!sh.valid || sh.gridW <= 0 || sh.obs.size() != static_cast<size_t>(sh.gridW) * sh.gridH) return;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
	for(int gv = 0; gv < sh.gridH; gv++){
		for(int gu = 0; gu < sh.gridW; gu++){
			if(sh.obs[static_cast<size_t>(gv) * sh.gridW + gu] != 255) continue;
			glm::vec2 p[4];
			if(!map(sh.toS(gu), sh.toY(gv), p[0]) || !map(sh.toS(gu + 1), sh.toY(gv), p[1])
			   || !map(sh.toS(gu + 1), sh.toY(gv + 1), p[2]) || !map(sh.toS(gu), sh.toY(gv + 1), p[3])) continue;
			const ofIndexType base = mesh.getNumVertices();
			for(int c = 0; c < 4; c++){
				mesh.addVertex(glm::vec3(p[c], 0));
				mesh.addColor(col);
			}
			mesh.addIndex(base); mesh.addIndex(base + 1); mesh.addIndex(base + 2);
			mesh.addIndex(base); mesh.addIndex(base + 2); mesh.addIndex(base + 3);
		}
	}
	if(mesh.getNumVertices() == 0) return;
	ofPushStyle();
	ofFill();
	ofEnableAlphaBlending();
	mesh.draw();
	ofPopStyle();
}

// Outline polygon (corridor coordinates) through the same kind of map:
// faint fill plus a stroke. Vertices the view does not show are dropped.
void drawShapeOutline(const std::vector<std::pair<float, float>> & outline, const CorridorToScreen & map,
                      const ofColor & col, float lineWidth, int fillAlpha){
	if(outline.size() < 3) return;
	std::vector<glm::vec2> pts;
	pts.reserve(outline.size());
	for(const auto & o : outline){
		glm::vec2 p;
		if(map(o.first, o.second, p)) pts.push_back(p);
	}
	if(pts.size() < 3) return;
	ofPushStyle();
	if(fillAlpha > 0){
		ofFill();
		ofSetColor(col, fillAlpha);
		ofBeginShape();
		for(const auto & p : pts) ofVertex(p.x, p.y);
		ofEndShape(true);
	}
	ofNoFill();
	ofSetColor(col);
	ofSetLineWidth(lineWidth);
	ofPolyline line;
	for(const auto & p : pts) line.addVertex(glm::vec3(p, 0));
	line.close();
	line.draw();
	ofSetLineWidth(1);
	ofPopStyle();
}

// Which box and center the views draw. Result is what is published (a
// boat's outline bounds); raw is the flow box kept beside it.
struct ShownGeom {
	float x0 = 0, x1 = 0, y0 = 0, y1 = 0, x = 0, y = 0, w = 0, h = 0;
	const std::vector<std::pair<float, float>> * trail = nullptr;
};

ShownGeom shownGeom(const tracking::TrackedObject & o, bool result){
	ShownGeom g;
	if(result){
		g.x0 = o.x0; g.x1 = o.x1; g.y0 = o.y0; g.y1 = o.y1;
		g.x = o.x; g.y = o.y; g.w = o.w; g.h = o.h;
		g.trail = &o.trail;
	}else{
		g.x0 = o.rawX0; g.x1 = o.rawX1; g.y0 = o.rawY0; g.y1 = o.rawY1;
		g.x = o.rawX; g.y = o.rawY; g.w = o.rawW; g.h = o.rawH;
		g.trail = &o.rawTrail;
	}
	return g;
}
} // namespace

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetFrameRate(60);
	ofSetVerticalSync(true);
	ofBackground(12);

	session.load("config/config.json");

	// Headless runs: CAMTRACK_AUTOPLAYBACK=1 starts in playback mode
	// (tracking against a recording, no camera); CAMTRACK_RECORDINGS
	// overrides the recordings folder, so a single recording can be played
	// by pointing it at a folder holding just that one. CAMTRACK_AUTOEXPORT=1
	// starts a motion export right away.
	if(std::getenv("CAMTRACK_AUTOPLAYBACK")){
		if(const char * dir = std::getenv("CAMTRACK_RECORDINGS")) session.setRecordingsDir(ofToDataPath(dir, true));
	}

	gui.setup("camera tracking", "gui.xml");
	gui.add(playbackParam);
	gui.add(recordParam);
	gui.add(exportParam);
	gui.add(trackingParam);
	gui.add(overlayParam);
	gui.add(imageScaleParam);
	gui.add(showMaskParam);
	gui.add(showDetectionsParam);
	gui.add(showPixelsParam);
	gui.add(showOutlineParam);
	gui.add(showBoxParam);
	gui.add(showResultParam);
	gui.add(resultViewParam);
	reconnectButton.setup("reconnect (r)");
	gui.add(&reconnectButton);
	gui.setPosition(10, 10);

	// playback transport. Listeners are attached right away: the panel is
	// only drawn (and hit) in playback mode, and the keys route through the
	// same parameters.
	playbackGui.setup("playback", "playback_gui.xml");
	playbackGui.add(playParam);
	playbackGui.add(playheadParam);
	prevFrameButton.setup("prev frame");
	nextFrameButton.setup("next frame");
	prevRecordingButton.setup("prev recording (LEFT)");
	nextRecordingButton.setup("next recording (RIGHT)");
	playbackGui.add(&prevFrameButton);
	playbackGui.add(&nextFrameButton);
	playbackGui.add(&prevRecordingButton);
	playbackGui.add(&nextRecordingButton);
	playbackGui.unregisterMouseEvents(); // hidden in live mode; layoutGuiPanels() wires it up
	playParam.addListener(this, &ofApp::onPlayChanged);
	playheadParam.addListener(this, &ofApp::onPlayheadChanged);
	prevFrameButton.addListener(this, &ofApp::onPrevFramePressed);
	nextFrameButton.addListener(this, &ofApp::onNextFramePressed);
	prevRecordingButton.addListener(this, &ofApp::onPrevRecordingPressed);
	nextRecordingButton.addListener(this, &ofApp::onNextRecordingPressed);

	// edit panel: the only values that leave the window are the lane
	// anchors and the corridor placements, written by the save button.
	editGui.setup("edit", "edit_gui.xml");
	editGui.add(laneSelectParam);
	editGui.add(laneS0Param);
	editGui.add(laneS1Param);
	editGui.add(corridorSelectParam);
	editGui.add(placeX0Param);
	editGui.add(placeX1Param);
	editGui.add(placeY0Param);
	editGui.add(placeY1Param);
	saveButton.setup("save config.json");
	editGui.add(&saveButton);
	saveButton.addListener(this, &ofApp::onSavePressed);
	layoutGuiPanels();

	// never restore a stale state from gui.xml
	recordParam = false;
	exportParam = false;
	playbackParam = false;
	trackingParam = session.isTrackingEnabled();
	recordParam.addListener(this, &ofApp::onRecordChanged);
	exportParam.addListener(this, &ofApp::onExportChanged);
	playbackParam.addListener(this, &ofApp::onPlaybackChanged);
	trackingParam.addListener(this, &ofApp::onTrackingChanged);
	reconnectButton.addListener(this, &ofApp::onReconnectPressed);

	session.start();

	if(std::getenv("CAMTRACK_AUTOPLAYBACK")){
		ofLogNotice("ofApp") << "autoplayback enabled";
		playbackParam = true;
	}
	if(std::getenv("CAMTRACK_AUTOEXPORT")){
		ofLogNotice("ofApp") << "autoexport enabled";
		exportParam = true;
	}
}

//--------------------------------------------------------------
// Lane editing state follows the session's resolved layout (source and
// pane count): that layout becomes the editable copy.
void ofApp::syncEditLayout(){
	if(session.layoutRevision() == seenLayoutRev) return;
	seenLayoutRev = session.layoutRevision();
	editLayout = session.currentLayout();
	dragLane = -1;
	dragEdge = dragStation = -1;
	anchorLane = anchorEdge = anchorStation = -1;
	laneSelectParam.setMax(std::max(0, static_cast<int>(editLayout.lanes.size()) - 1));
	laneSelectParam = std::min(laneSelectParam.get(), laneSelectParam.getMax());
	lastLaneSelect = -1; // re-sync the s0/s1 sliders
	laneMaskTex.assign(editLayout.lanes.size(), ofTexture());
}

void ofApp::commitLayout(){
	// a synthesized fallback layout comes back with its real name
	editLayout.name = session.setLayout(editLayout).name;
}

//--------------------------------------------------------------
void ofApp::update(){
	session.update();
	const bool playback = session.getMode() == Mode::Playback;

	// newest corrected frame -> texture (an empty preview clears it)
	ofPixels pixels;
	if(session.takePreview(pixels, previewGeometry, previewRev)){
		if(pixels.isAllocated()) loadFrameTexture(texture, pixels);
		else texture.clear();
	}
	syncEditLayout();

	// playhead slider mirrors the player, except while the mouse is
	// dragging it — then the slider drives the player (onPlayheadChanged)
	if(playback){
		const bool scrubbing = ofGetMousePressed()
			&& playbackGui.getShape().inside(ofGetMouseX(), ofGetMouseY());
		if(!scrubbing){
			const float durationSec = static_cast<float>(session.durationSec());
			if(durationSec > 0 && playheadParam.getMax() != durationSec) playheadParam.setMax(durationSec);
			playheadSync = true;
			playheadParam = static_cast<float>(session.positionSec());
			playheadSync = false;
		}
	}

	// newest tracking results (published by the tracking thread)
	if(session.takeResults(trackResults, trackResultsRev) && showMaskParam){
		// lane masks -> textures (only when shown)
		if(laneMaskTex.size() < trackResults.lanes.size()) laneMaskTex.resize(trackResults.lanes.size());
		for(size_t i = 0; i < trackResults.lanes.size(); i++){
			const auto & dbg = trackResults.lanes[i];
			if(dbg.maskW <= 0 || dbg.maskH <= 0 || dbg.mask.empty()) continue;
			ofPixels p;
			p.setFromPixels(dbg.mask.data(), dbg.maskW, dbg.maskH, OF_PIXELS_GRAY);
			loadFrameTexture(laneMaskTex[i], p);
		}
	}

	// lane selector -> s0/s1 sliders (and back)
	if(!editLayout.lanes.empty()){
		const int sel = ofClamp(laneSelectParam.get(), 0, static_cast<int>(editLayout.lanes.size()) - 1);
		if(sel != lastLaneSelect){
			lastLaneSelect = sel;
			laneS0Param = editLayout.lanes[sel].s0;
			laneS1Param = editLayout.lanes[sel].s1;
		}else{
			tracking::LaneConfig & lane = editLayout.lanes[sel];
			if(lane.s0 != laneS0Param.get() || lane.s1 != laneS1Param.get()){
				lane.s0 = laneS0Param;
				lane.s1 = laneS1Param;
				if(dragLane < 0) commitLayout();
			}
		}
	}

	// corridor selector -> placement sliders (and back). While the mouse
	// drags a placement the sliders follow it; otherwise they drive it.
	{
		const std::vector<std::string> ids = session.corridorIds();
		corridorSelectParam.setMax(std::max(0, static_cast<int>(ids.size()) - 1));
		if(!ids.empty()){
			const int sel = ofClamp(corridorSelectParam.get(), 0, static_cast<int>(ids.size()) - 1);
			if(sel != corridorSelectParam.get()) corridorSelectParam = sel;
			const tracking::CorridorPlacement current = session.getConfig().placementFor(ids[sel]);
			if(sel != lastCorridorSelect || dragCorridor >= 0){
				lastCorridorSelect = sel;
				placeX0Param = current.x0;
				placeX1Param = current.x1;
				placeY0Param = current.y0;
				placeY1Param = current.y1;
			}else if(current.x0 != placeX0Param.get() || current.x1 != placeX1Param.get()
			         || current.y0 != placeY0Param.get() || current.y1 != placeY1Param.get()){
				tracking::CorridorPlacement p;
				p.x0 = placeX0Param;
				p.x1 = placeX1Param;
				p.y0 = placeY0Param;
				p.y1 = placeY1Param;
				session.setPlacement(ids[sel], p);
			}
		}
	}

	// periodic stats log (useful when running from the terminal)
	if(ofGetElapsedTimeMillis() - lastStatsLogMs > 5000){
		lastStatsLogMs = ofGetElapsedTimeMillis();
		const StreamStats stats = session.streamStats();
		const SensorSnapshot & sensors = session.getSensors();
		ofLogNotice("ofApp")
			<< "[" << stats.state << "] decoded " << ofToString(stats.decodedFps, 1)
			<< " fps, " << ofToString(stats.bytesPerSecond / (1024.0 * 1024.0), 2)
			<< " MB/s, frames " << stats.framesDecoded
			<< ", MultiSense " << ofToString(sensors.multiSenseLux(), 1) << " lx"
			<< ", published " << session.resultRevision()
			<< (stats.lastError.empty() ? "" : ", error: " + stats.lastError);
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	drawVideo();
	drawRecordingIndicator();
	drawStatsBar();
	gui.draw();
	if(session.getMode() == Mode::Playback) playbackGui.draw();
	editGui.draw();
}

//--------------------------------------------------------------
void ofApp::drawVideo(){
	const float splitGap = 8;
	// keep the image clear of the GUI panels on the left
	const float guiWidth = std::max(gui.getShape().getRight(), editGui.getShape().getRight()) + splitGap;
	const ofRectangle full(guiWidth, 0, ofGetWidth() - guiWidth, ofGetHeight() - kBarHeight);
	// top: the lens-corrected image(s) with lane editor + overlay; bottom: the
	// rectified lane views (motion mask + tracking, no camera pixels)
	const ofRectangle viewport(full.x, full.y, full.width, full.height * 0.6f);
	const ofRectangle resultView(full.x, viewport.getBottom() + splitGap,
	                             full.width, full.height * 0.4f - splitGap);

	const bool playback = session.getMode() == Mode::Playback;
	const ofTexture & tex = texture;

	paneRects.clear();
	laneResultRects.clear();

	if(!tex.isAllocated()){
		ofSetColor(160);
		const std::string msg = playback
			? "waiting for playback frame (" + session.recordingName() + ")"
			: "waiting for stream: " + session.streamUrl();
		ofDrawBitmapString(msg, viewport.getCenter().x - 240, viewport.getCenter().y);
		ofSetColor(255);
		return;
	}

	ofSetColor(255);
	const LensGeometry & lensGeo = previewGeometry;
	const bool geo = lensGeo.outPaneW > 0 && lensGeo.outPaneH > 0 && lensGeo.panes > 0
		&& std::abs(tex.getWidth() - lensGeo.panes * lensGeo.outPaneW) < 1.5
		&& std::abs(tex.getHeight() - lensGeo.outPaneH) < 1.5;
	const bool splitBoth = geo ? lensGeo.panes >= 2 : tex.getWidth() >= tex.getHeight() * 2;
	const float extW = geo ? static_cast<float>(lensGeo.outPaneW)
	                       : tex.getWidth() / (splitBoth ? 2.0f : 1.0f);
	const float extH = geo ? static_cast<float>(lensGeo.outPaneH) : tex.getHeight();
	const int innerX = geo ? lensGeo.innerX : 0;
	const int innerY = geo ? lensGeo.innerY : 0;
	const int innerW = geo ? lensGeo.innerW : static_cast<int>(extW);
	const int innerH = geo ? lensGeo.innerH : static_cast<int>(extH);

	// 1 fits the whole corrected picture, border included. Higher values
	// enlarge it around the center and clip to the video area.
	const float zoom = std::max(1.0f, imageScaleParam.get());
	std::vector<float> paneFitW;
	beginScissor(viewport);
	if(splitBoth){
		// one BOTH frame = M1 | M2 side by side; draw as two panes with a gap
		const float gap = 4;
		const float paneW = (viewport.width - gap) / 2.0f;
		const float fit = std::min(paneW / extW, viewport.height / extH);
		const float drawW = extW * fit * zoom;
		const float drawH = extH * fit * zoom;
		const float y = viewport.y + (viewport.height - drawH) / 2.0f;
		const float x0 = viewport.x + (paneW - drawW) / 2.0f;
		const float x1 = viewport.x + paneW + gap + (paneW - drawW) / 2.0f;
		ofRectangle r0, r1;
		drawCorrectedPane(tex, 0, extW, extH, x0, y, drawW, drawH, innerX, innerY, innerW, innerH, r0);
		drawCorrectedPane(tex, extW, extW, extH, x1, y, drawW, drawH, innerX, innerY, innerW, innerH, r1);
		paneRects.push_back(r0); // module 0 (M1)
		paneRects.push_back(r1); // module 1 (M2)
		paneFitW.push_back(extW * fit);
		paneFitW.push_back(extW * fit);
		ofSetColor(200);
		ofDrawBitmapString("M1 (left)", r0.x + 6, r0.y + 16);
		ofDrawBitmapString("M2 (right)", r1.x + 6, r1.y + 16);
		ofSetColor(255);
	}else{
		const float fit = std::min(viewport.width / extW, viewport.height / extH);
		const float drawW = extW * fit * zoom;
		const float drawH = extH * fit * zoom;
		const float x = viewport.x + (viewport.width - drawW) / 2.0f;
		const float y = viewport.y + (viewport.height - drawH) / 2.0f;
		ofRectangle r;
		drawCorrectedPane(tex, 0, extW, extH, x, y, drawW, drawH, innerX, innerY, innerW, innerH, r);
		paneRects.push_back(r); // single module
		paneFitW.push_back(extW * fit);
	}

	// lane result rects: one column per pane, the pane's lanes stacked in
	// rows; each cell keeps the rectified lane's aspect
	// Lane aspect is measured on the filled frame, not the extended mat,
	// so a lane that stays inside 0..1 keeps the size it had before.
	const float texPaneW = innerW > 0 ? static_cast<float>(innerW) : tex.getWidth() / paneRects.size();
	const float texPaneH = innerH > 0 ? static_cast<float>(innerH) : tex.getHeight();
	std::vector<int> rowsInPane(paneRects.size(), 0);
	for(const auto & lane : editLayout.lanes){
		if(lane.pane >= 0 && lane.pane < static_cast<int>(rowsInPane.size())) rowsInPane[lane.pane]++;
	}
	std::vector<int> rowCursor(paneRects.size(), 0);
	laneResultRects.assign(editLayout.lanes.size(), ofRectangle());
	for(size_t i = 0; i < editLayout.lanes.size(); i++){
		const tracking::LaneConfig & lane = editLayout.lanes[i];
		if(lane.pane < 0 || lane.pane >= static_cast<int>(paneRects.size())) continue;
		const int rows = std::max(1, rowsInPane[lane.pane]);
		const float cellH = (resultView.height - splitGap * (rows - 1)) / rows;
		const float cellY = resultView.y + rowCursor[lane.pane]++ * (cellH + splitGap);
		const float aspect = lane.aspect(texPaneW, texPaneH);
		float w = paneFitW[lane.pane];
		float h = w / aspect;
		if(h > cellH){ h = cellH; w = h * aspect; }
		if(h < 40){ h = std::min(40.0f, cellH); }
		const float x = paneRects[lane.pane].getCenter().x - w / 2.0f;
		const float y = cellY + (cellH - h) / 2.0f;
		laneResultRects[i] = ofRectangle(x, y, w, h);
	}

	// output canvas (Lines-area proportions) centred in the result view
	// with room around it so off-canvas placement parts stay visible
	{
		const tracking::CanvasConfig & canvas = session.getConfig().canvas;
		const float aspect = canvas.height > 0 ? static_cast<float>(canvas.width) / canvas.height : 1.0f;
		float w = resultView.width * 0.62f;
		float h = w / aspect;
		if(h > resultView.height * 0.72f){ h = resultView.height * 0.72f; w = h * aspect; }
		canvasViewRect = resultView;
		canvasRect = ofRectangle(resultView.getCenter().x - w / 2.0f, resultView.getCenter().y - h / 2.0f, w, h);
	}

	drawTrackingOverlay();
	drawLaneEditor();
	endScissor();
	if(resultViewParam == 0) drawCanvasView();
	else drawTrackingResults();
}

//--------------------------------------------------------------
// The output canvas: what a consumer receives. Every corridor's placement
// rectangle is drawn (the part outside the canvas dimmed — that part of the
// corridor is cut), the tracked objects are mapped through it and clipped
// like the published data. The selected corridor has corner handles and can
// be dragged by its body.
void ofApp::drawCanvasView(){
	if(canvasRect.width <= 0) return;
	const ofRectangle & c = canvasRect;
	const auto toScreen = [&](float cx, float cy){
		return glm::vec2(c.x + cx * c.width, c.y + cy * c.height);
	};
	const tracking::TrackingConfig & cfg = session.getConfig();
	const float arrowScale = session.arrowScale();
	const std::vector<std::string> ids = session.corridorIds();
	const int sel = ofClamp(corridorSelectParam.get(), 0, std::max(0, static_cast<int>(ids.size()) - 1));

	ofPushStyle();
	beginScissor(canvasViewRect);
	ofFill();
	ofSetColor(12);
	ofDrawRectangle(canvasViewRect);
	ofSetColor(34);
	ofDrawRectangle(c);
	ofSetColor(48);
	for(int i = 1; i < 10; i++){
		ofDrawLine(c.x + c.width * i / 10.0f, c.y, c.x + c.width * i / 10.0f, c.getBottom());
		ofDrawLine(c.x, c.y + c.height * i / 10.0f, c.getRight(), c.y + c.height * i / 10.0f);
	}
	ofNoFill();
	ofSetColor(150);
	ofDrawRectangle(c);
	ofDrawBitmapStringHighlight("canvas " + ofToString(cfg.canvas.width) + "x"
		+ ofToString(cfg.canvas.height) + (showResultParam ? "  result" : "  raw")
		+ "  (0,0 top-left, 1,1 bottom-right)",
		c.x + 6, c.y - 6, ofColor(0, 140), ofColor(200));

	// placements
	for(size_t i = 0; i < ids.size(); i++){
		const tracking::CorridorPlacement p = cfg.placementFor(ids[i]);
		const bool selected = static_cast<int>(i) == sel;
		const ofColor col = corridorColor(ids[i]);
		const glm::vec2 a = toScreen(std::min(p.x0, p.x1), std::min(p.y0, p.y1));
		const glm::vec2 b = toScreen(std::max(p.x0, p.x1), std::max(p.y0, p.y1));
		const ofRectangle full(a.x, a.y, b.x - a.x, b.y - a.y);
		ofRectangle vis = full.getIntersection(c);
		ofNoFill();
		ofSetColor(col, selected ? 110 : 60);
		ofSetLineWidth(1);
		ofDrawRectangle(full);
		ofSetColor(col, selected ? 255 : 150);
		ofSetLineWidth(selected ? 2 : 1);
		if(vis.width > 0 && vis.height > 0) ofDrawRectangle(vis);
		ofSetLineWidth(1);
		std::string label = ids[i] + "  x " + ofToString(p.x0, 2) + ".." + ofToString(p.x1, 2)
			+ "  y " + ofToString(p.y0, 2) + ".." + ofToString(p.y1, 2);
		if(p.mirrored()) label += "  (mirrored)";
		if(selected) label += "  drag body / corners";
		ofDrawBitmapStringHighlight(label, full.x + 4, full.y + 12, ofColor(0, 140), col);
		if(selected){
			ofFill();
			const glm::vec2 hs[4] = {toScreen(p.x0, p.y0), toScreen(p.x1, p.y0), toScreen(p.x1, p.y1), toScreen(p.x0, p.y1)};
			for(int h = 0; h < 4; h++){
				const bool dragged = dragCorridor == static_cast<int>(i) && dragPlaceHandle == h;
				ofSetColor(col, dragged ? 255 : 200);
				ofDrawCircle(hs[h].x, hs[h].y, dragged ? kFrameHandleRadius + 2 : kFrameHandleRadius);
			}
			// arrow along the corridor direction on the canvas
			const glm::vec2 m0 = toScreen(p.x0 + 0.05f * p.spanX(), (p.y0 + p.y1) * 0.5f);
			const glm::vec2 m1 = toScreen(p.x0 + 0.20f * p.spanX(), (p.y0 + p.y1) * 0.5f);
			ofSetColor(col, 160);
			ofDrawArrow(glm::vec3(m0, 0), glm::vec3(m1, 0), 5.0f);
		}
	}

	// objects, mapped and cut exactly like the published data. Three
	// layers per object, each switchable: shape pixels, outline, box.
	if(trackingParam){
		for(const auto & cr : trackResults.corridors){
			const tracking::CorridorPlacement p = cfg.placementFor(cr.id);
			const CorridorToScreen mapCanvas = [&](float s, float y, glm::vec2 & out){
				out = toScreen(p.toCanvasX(s), p.toCanvasY(y));
				return true;
			};
			for(const auto & obj : cr.objects){
				if(!obj.confirmed) continue;
				const ShownGeom g = shownGeom(obj, showResultParam);
				float x0, x1, y0, y1;
				p.mapBox(g.x0, g.x1, g.y0, g.y1, x0, x1, y0, y1);
				const ofColor col = objectColor(obj);
				if(showPixelsParam) drawShapeCells(obj.shape, mapCanvas, ofColor(col, 110));
				if(showOutlineParam && obj.shape.valid && !obj.shape.outline.empty()){
					// uncut outline dim, the published (canvas-clipped) one bright
					drawShapeOutline(obj.shape.outline, mapCanvas, ofColor(col, 60), 1, 0);
					std::vector<std::pair<float, float>> onCanvas;
					for(const auto & q : obj.shape.outline) onCanvas.emplace_back(p.toCanvasX(q.first), p.toCanvasY(q.second));
					const auto clipped = output::clipPolygonToCanvas(onCanvas);
					drawShapeOutline(clipped, [&](float cx, float cy, glm::vec2 & out){ out = toScreen(cx, cy); return true; },
					                 col, 2, 40);
				}
				// uncut box (dim) — shows what the placement cuts away
				const glm::vec2 fa = toScreen(x0, y0), fb = toScreen(x1, y1);
				if(showBoxParam){
					ofNoFill();
					ofSetColor(col, 70);
					ofDrawRectangle(fa.x, fa.y, fb.x - fa.x, fb.y - fa.y);
				}
				const bool offL = x0 < 0, offR = x1 > 1;
				if(!tracking::CorridorPlacement::clipToCanvas(x0, x1, y0, y1)) continue;
				const glm::vec2 a = toScreen(x0, y0), b = toScreen(x1, y1);

				if(g.trail->size() > 1){
					for(size_t i = 1; i < g.trail->size(); i++){
						const glm::vec2 t0 = toScreen(p.toCanvasX((*g.trail)[i - 1].first), p.toCanvasY((*g.trail)[i - 1].second));
						const glm::vec2 t1 = toScreen(p.toCanvasX((*g.trail)[i].first), p.toCanvasY((*g.trail)[i].second));
						if(!c.inside(t0) || !c.inside(t1)) continue;
						const float alpha = static_cast<float>(i) / g.trail->size();
						ofSetColor(col.r, col.g, col.b, col.a * alpha * 0.6f);
						ofDrawLine(t0, t1);
					}
				}
				if(showBoxParam){
					ofNoFill();
					ofSetColor(col);
					ofSetLineWidth(2);
					ofDrawRectangle(a.x, a.y, b.x - a.x, b.y - a.y);
					ofSetLineWidth(1);
					// A corridor-end tick only where the drawn box actually sits on
					// that end. The flow box can be pinned there while the
					// hull box is not.
					const bool pinL = p.mirrored() ? g.x1 >= 0.98f && obj.clippedRight
					                               : g.x0 <= 0.02f && obj.clippedLeft;
					const bool pinR = p.mirrored() ? g.x0 <= 0.02f && obj.clippedLeft
					                               : g.x1 >= 0.98f && obj.clippedRight;
					const bool cutL = offL || pinL;
					const bool cutR = offR || pinR;
					ofSetColor(255, 60, 60);
					ofSetLineWidth(3);
					if(cutL) ofDrawLine(a.x, a.y, a.x, b.y);
					if(cutR) ofDrawLine(b.x, a.y, b.x, b.y);
					ofSetLineWidth(1);
				}

				ofSetColor(col);
				const glm::vec2 ctr((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f);
				ofFill();
				ofDrawCircle(ctr.x, ctr.y, 3);
				float dx = obj.vx * p.spanX() * arrowScale, dy = obj.vy * p.spanY() * arrowScale;
				const float len = std::hypot(dx, dy);
				if(len > kMaxArrow){ dx *= kMaxArrow / len; dy *= kMaxArrow / len; }
				ofDrawArrow(glm::vec3(ctr, 0), glm::vec3(ctr.x + dx * c.width, ctr.y + dy * c.height, 0), 5.0f);
				std::string text = "#" + ofToString(obj.id) + " " + obj.label + " " + obj.phase
					+ "  x=" + ofToString((x0 + x1) * 0.5f, 2) + " vx=" + ofToString(obj.vx * p.spanX(), 3);
				if(obj.shape.valid && !obj.shape.source.empty()) text += "  shape:" + obj.shape.source;
				ofDrawBitmapStringHighlight(text, a.x, a.y - 6, ofColor(0, 160), col);
			}
		}
	}
	endScissor();
	ofPopStyle();
	ofSetColor(255);
}

//--------------------------------------------------------------
// Debug overlay on the ORIGINAL image: per tracked object — the corridor
// box mapped into every lane of its corridor and projected through the lane
// quad back onto the (distorted) photo, center point, velocity arrow
// (1 s lookahead x arrowScale), clipped-edge markers, "#id label phase" and
// a fading trail, all in the object's stable per-id color. Raw detections
// are drawn as thin grey boxes.
void ofApp::drawTrackingOverlay(){
	if(!trackingParam || !overlayParam) return;
	if(paneRects.empty()) return;
	const float arrowScale = session.arrowScale();

	ofPushStyle();
	std::vector<tracking::LaneMap> laneMaps(trackResults.lanes.size());
	for(size_t li = 0; li < trackResults.lanes.size(); li++){
		const tracking::LaneConfig & lane = trackResults.lanes[li].lane;
		if(lane.pane < 0 || lane.pane >= static_cast<int>(paneRects.size())) continue;
		const ofRectangle & pane = paneRects[lane.pane];
		laneMaps[li] = tracking::LaneMap::build(lane, pane.width, pane.height);
		if(showDetectionsParam){
			ofSetColor(170, 170, 170, 160);
			ofNoFill();
			const auto proj = [&](float u, float v){ return projectLane(laneMaps[li], pane, u, v); };
			for(const auto & d : trackResults.lanes[li].detections){
				const glm::vec2 p[4] = {proj(d.x0, d.y0), proj(d.x1, d.y0), proj(d.x1, d.y1), proj(d.x0, d.y1)};
				for(int c = 0; c < 4; c++) ofDrawLine(p[c], p[(c + 1) % 4]);
			}
		}
	}

	for(const auto & cr : trackResults.corridors){
		for(const auto & obj : cr.objects){
			if(!obj.confirmed) continue;
			const ShownGeom g = shownGeom(obj, showResultParam);
			const ofColor col = objectColor(obj);
			bool labelled = false;
			for(int li : cr.lanes){
				if(li < 0 || li >= static_cast<int>(laneMaps.size()) || !laneMaps[li].ok) continue;
				const tracking::LaneConfig & lane = trackResults.lanes[li].lane;
				if(lane.pane < 0 || lane.pane >= static_cast<int>(paneRects.size())) continue;
				const ofRectangle & pane = paneRects[lane.pane];
				const tracking::LaneMap & map = laneMaps[li];
				const auto proj = [&](float x, float y){ return projectLane(map, pane, x, y); };

				float lx0, lx1;
				if(!lane.corridorRangeToLane(g.x0, g.x1, lx0, lx1)) continue;

				// trail (older = more transparent), in this lane where visible
				if(g.trail->size() > 1){
					for(size_t i = 1; i < g.trail->size(); i++){
						const float a0 = lane.toLane((*g.trail)[i - 1].first);
						const float a1 = lane.toLane((*g.trail)[i].first);
						if(a0 < 0 || a0 > 1 || a1 < 0 || a1 > 1) continue;
						const float a = static_cast<float>(i) / g.trail->size();
						ofSetColor(col.r, col.g, col.b, col.a * a * 0.6f);
						ofDrawLine(proj(a0, (*g.trail)[i - 1].second), proj(a1, (*g.trail)[i].second));
					}
				}

				// shape layers, projected through this lane's quad
				const CorridorToScreen mapLane = [&](float s, float y, glm::vec2 & out){
					const float lx = lane.toLane(s);
					if(lx < -0.001f || lx > 1.001f) return false;
					out = proj(lx, y);
					return true;
				};
				if(showPixelsParam) drawShapeCells(obj.shape, mapLane, ofColor(col, 110));
				if(showOutlineParam && obj.shape.valid){
					drawShapeOutline(obj.shape.outline, mapLane, col, 2, 40);
				}

				ofSetColor(col);
				const glm::vec2 bTl = proj(lx0, g.y0), bTr = proj(lx1, g.y0);
				const glm::vec2 bBr = proj(lx1, g.y1), bBl = proj(lx0, g.y1);
				if(showBoxParam){
					ofNoFill();
					ofSetLineWidth(2);
					ofDrawLine(bTl, bTr);
					ofDrawLine(bTr, bBr);
					ofDrawLine(bBr, bBl);
					ofDrawLine(bBl, bTl);
					ofSetLineWidth(1);
					// clipped edge markers (red) only when this box's edge
					// actually sits on the corridor end
					float e0, e1;
					if(obj.clippedLeft && g.x0 <= 0.02f && lane.corridorRangeToLane(0.0f, 0.002f, e0, e1)){
						ofSetColor(255, 60, 60);
						ofSetLineWidth(3);
						ofDrawLine(bTl, bBl);
						ofSetLineWidth(1);
					}
					if(obj.clippedRight && g.x1 >= 0.98f && lane.corridorRangeToLane(0.998f, 1.0f, e0, e1)){
						ofSetColor(255, 60, 60);
						ofSetLineWidth(3);
						ofDrawLine(bTr, bBr);
						ofSetLineWidth(1);
					}
				}
				ofSetColor(col);

				// center + velocity arrow (corridor -> lane velocity scale)
				const float lcx = lane.toLane(g.x);
				if(lcx >= 0 && lcx <= 1){
					const glm::vec2 c = proj(lcx, g.y);
					ofFill();
					ofDrawCircle(c.x, c.y, 3);
					const float span = lane.s1 - lane.s0;
					float dx = (std::fabs(span) > 1e-6f ? obj.vx / span : obj.vx) * arrowScale;
					float dy = obj.vy * arrowScale;
					const float len = std::hypot(dx, dy);
					if(len > kMaxArrow){ dx *= kMaxArrow / len; dy *= kMaxArrow / len; }
					const glm::vec2 tip = proj(lcx + dx, g.y + dy);
					ofDrawArrow(glm::vec3(c.x, c.y, 0), glm::vec3(tip.x, tip.y, 0), 5.0f);
				}

				if(!labelled){
					std::string text = "#" + ofToString(obj.id);
					if(!obj.label.empty()) text += " " + obj.label;
					text += " " + obj.phase + "  v=" + ofToString(obj.vx, 3);
					ofDrawBitmapStringHighlight(text, bTl.x, bTl.y - 6, ofColor(0, 160), col);
					labelled = true;
				}
			}
		}
	}
	ofPopStyle();
	ofSetColor(255);
}

//--------------------------------------------------------------
// The draggable lane strips on each pane: top and bottom borders, a rib at
// each station, handles on every point. The selected lane is yellow, others
// cyan; red = a crossed slice — drawn but NOT sent to the tracker until
// every slice is convex again.
void ofApp::drawLaneEditor(){
	if(!overlayParam) return;

	ofPushStyle();
	for(size_t li = 0; li < editLayout.lanes.size(); li++){
		const tracking::LaneConfig & lane = editLayout.lanes[li];
		if(lane.pane < 0 || lane.pane >= static_cast<int>(paneRects.size())) continue;
		const ofRectangle & pane = paneRects[lane.pane];
		const bool selected = static_cast<int>(li) == laneSelectParam.get();
		const bool valid = lane.isWarpable();
		const ofColor col = !valid ? ofColor(240, 80, 80)
			: selected ? ofColor(255, 210, 60) : ofColor(80, 200, 230);

		std::vector<std::array<float, 2>> top, bottom;
		lane.borders(top, bottom);
		const auto scr = [&](const std::array<float, 2> & p){
			return glm::vec2(pane.x + p[0] * pane.width, pane.y + p[1] * pane.height);
		};

		ofSetColor(col, selected ? 220 : 130);
		ofNoFill();
		for(size_t i = 0; i + 1 < top.size(); i++) ofDrawLine(scr(top[i]), scr(top[i + 1]));
		for(size_t i = 0; i + 1 < bottom.size(); i++) ofDrawLine(scr(bottom[i]), scr(bottom[i + 1]));
		ofSetColor(col, selected ? 90 : 50);
		for(size_t i = 0; i < top.size() && i < bottom.size(); i++) ofDrawLine(scr(top[i]), scr(bottom[i]));

		ofSetColor(col, selected ? 220 : 130);
		ofDrawBitmapStringHighlight(lane.id + " -> " + lane.corridor
			+ " s " + ofToString(lane.s0, 2) + ".." + ofToString(lane.s1, 2),
			scr(top.front()).x + 4, scr(top.front()).y + 12, ofColor(0, 140), col);
		if(selected){
			ofDrawBitmapStringHighlight("shift-click an edge to add an anchor, delete removes it",
			                            scr(top.front()).x + 4, scr(top.front()).y + 28, ofColor(0, 140), col);
		}

		ofFill();
		const auto handle = [&](int edge, int index, const glm::vec2 & p){
			const bool end = index == 0 || index + 1 == static_cast<int>(top.size());
			const bool dragged = static_cast<int>(li) == dragLane && edge == dragEdge && index == dragStation;
			const bool anchored = static_cast<int>(li) == anchorLane && edge == anchorEdge && index == anchorStation;
			ofSetColor(col, dragged || anchored ? 255 : selected ? 200 : 110);
			const float r = dragged ? kFrameHandleRadius + 2
			                        : (selected && (end || anchored)) ? kFrameHandleRadius
			                        : selected ? 4.5f : 3.5f;
			ofDrawCircle(p.x, p.y, r);
		};
		for(size_t i = 0; i < top.size(); i++) handle(0, static_cast<int>(i), scr(top[i]));
		for(size_t i = 0; i < bottom.size(); i++) handle(1, static_cast<int>(i), scr(bottom[i]));
		if(!valid){
			ofSetColor(col);
			ofDrawBitmapStringHighlight("invalid lane (crossed) - keeping the last valid one",
			                            pane.x + 6, pane.getBottom() - 8, ofColor(0, 160), col);
		}
	}
	ofPopStyle();
	ofSetColor(255);
}

//--------------------------------------------------------------
// The rectified lane views below the image: the lane quad warped to an
// upright rectangle (what the detector sees), with the fused motion mask as
// background, the raw detections and the corridor objects mapped into lane
// space. 0,0 = top-left / 1,1 = bottom-right of the lane.
void ofApp::drawTrackingResults(){
	if(laneResultRects.empty()) return;
	const float arrowScale = session.arrowScale();

	ofPushStyle();
	for(size_t li = 0; li < laneResultRects.size(); li++){
		const ofRectangle & r = laneResultRects[li];
		if(r.width <= 0) continue;
		const tracking::LaneConfig & lane = editLayout.lanes[li];

		ofFill();
		ofSetColor(18);
		ofDrawRectangle(r);
		if(showMaskParam && li < laneMaskTex.size() && laneMaskTex[li].isAllocated()){
			ofSetColor(120, 160, 200);
			laneMaskTex[li].draw(r);
		}
		ofNoFill();
		ofSetColor(static_cast<int>(li) == laneSelectParam.get() ? ofColor(255, 210, 60) : ofColor(110));
		ofDrawRectangle(r);
		ofSetColor(200);
		ofDrawBitmapStringHighlight(lane.id + "  (lane 0..1, corridor " + lane.corridor + ")"
		                            + (showResultParam ? "  result" : "  raw"),
		                            r.x + 6, r.y + 14, ofColor(0, 140), ofColor(220));

		if(!trackingParam) continue;

		// the worker's lane index for this lane id (layouts are in sync
		// except during a rebuild)
		int workerLane = -1;
		for(size_t w = 0; w < trackResults.lanes.size(); w++){
			if(trackResults.lanes[w].lane.id == lane.id){ workerLane = static_cast<int>(w); break; }
		}
		if(workerLane < 0) continue;

		if(showDetectionsParam){
			ofSetColor(170, 170, 170, 180);
			ofNoFill();
			for(const auto & d : trackResults.lanes[workerLane].detections){
				ofDrawRectangle(r.x + d.x0 * r.width, r.y + d.y0 * r.height,
				                (d.x1 - d.x0) * r.width, (d.y1 - d.y0) * r.height);
			}
		}

		for(const auto & cr : trackResults.corridors){
			if(std::find(cr.lanes.begin(), cr.lanes.end(), workerLane) == cr.lanes.end()) continue;
			for(const auto & obj : cr.objects){
				if(!obj.confirmed) continue;
				const ShownGeom g = shownGeom(obj, showResultParam);
				float lx0, lx1;
				if(!lane.corridorRangeToLane(g.x0, g.x1, lx0, lx1)) continue;
				const ofColor col = objectColor(obj);
				const float bx = r.x + lx0 * r.width, by = r.y + g.y0 * r.height;
				const float bw = (lx1 - lx0) * r.width, bh = g.h * r.height;

				if(g.trail->size() > 1){
					for(size_t i = 1; i < g.trail->size(); i++){
						const float a0 = lane.toLane((*g.trail)[i - 1].first);
						const float a1 = lane.toLane((*g.trail)[i].first);
						if(a0 < 0 || a0 > 1 || a1 < 0 || a1 > 1) continue;
						const float a = static_cast<float>(i) / g.trail->size();
						ofSetColor(col.r, col.g, col.b, col.a * a * 0.6f);
						ofDrawLine(r.x + a0 * r.width, r.y + (*g.trail)[i - 1].second * r.height,
						           r.x + a1 * r.width, r.y + (*g.trail)[i].second * r.height);
					}
				}

				const CorridorToScreen mapStrip = [&](float s, float y, glm::vec2 & out){
					const float lx = lane.toLane(s);
					if(lx < -0.001f || lx > 1.001f) return false;
					out = glm::vec2(r.x + lx * r.width, r.y + y * r.height);
					return true;
				};
				if(showPixelsParam) drawShapeCells(obj.shape, mapStrip, ofColor(col, 110));
				if(showOutlineParam && obj.shape.valid){
					drawShapeOutline(obj.shape.outline, mapStrip, col, 2, 40);
				}

				if(showBoxParam){
					ofSetColor(col);
					ofNoFill();
					ofSetLineWidth(2);
					ofDrawRectangle(bx, by, bw, bh);
					ofSetLineWidth(1);
					float e0, e1;
					if(obj.clippedLeft && g.x0 <= 0.02f && lane.corridorRangeToLane(0.0f, 0.002f, e0, e1)){
						ofSetColor(255, 60, 60);
						ofSetLineWidth(3);
						ofDrawLine(bx, by, bx, by + bh);
						ofSetLineWidth(1);
					}
					if(obj.clippedRight && g.x1 >= 0.98f && lane.corridorRangeToLane(0.998f, 1.0f, e0, e1)){
						ofSetColor(255, 60, 60);
						ofSetLineWidth(3);
						ofDrawLine(bx + bw, by, bx + bw, by + bh);
						ofSetLineWidth(1);
					}
				}
				ofSetColor(col);
				const float lcx = lane.toLane(g.x);
				if(lcx >= 0 && lcx <= 1){
					const float cx = r.x + lcx * r.width, cy = r.y + g.y * r.height;
					ofFill();
					ofDrawCircle(cx, cy, 3);
					const float span = lane.s1 - lane.s0;
					float dx = (std::fabs(span) > 1e-6f ? obj.vx / span : obj.vx) * arrowScale;
					float dy = obj.vy * arrowScale;
					const float len = std::hypot(dx, dy);
					if(len > kMaxArrow){ dx *= kMaxArrow / len; dy *= kMaxArrow / len; }
					ofDrawArrow(glm::vec3(cx, cy, 0), glm::vec3(cx + dx * r.width, cy + dy * r.height, 0), 5.0f);
				}
				std::string text = "#" + ofToString(obj.id) + " " + obj.phase
					+ "  s=" + ofToString(g.x, 2) + " w=" + ofToString(g.w, 2)
					+ " v=" + ofToString(obj.vx, 3);
				ofDrawBitmapStringHighlight(text, bx, by - 6, ofColor(0, 160), col);
			}
		}
	}
	ofPopStyle();
	ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::drawRecordingIndicator(){
	const float x = 250; // right of the gui panel
	float y = 26;
	if(session.isRecording()){
		const MjpegRecorder & rec = session.getRecorder();
		const bool blinkOn = (static_cast<int>(ofGetElapsedTimef() * 2) % 2) == 0;
		if(blinkOn){
			ofSetColor(230, 40, 40);
			ofDrawCircle(x, y - 4, 7);
		}
		const std::string label = "REC " + formatClock(rec.getDurationMs())
			+ "  " + ofToString(rec.getFramesWritten()) + " frames  "
			+ ofToString(rec.getBytesWritten() / (1024.0 * 1024.0), 1) + " MB";
		ofDrawBitmapStringHighlight(label, x + 14, y, ofColor(0, 180), ofColor(255, 90, 90));
		y += 22;
	}
	if(session.isMotionExporting()){
		const std::string label = "EXPORT " + ofFilePath::getFileName(session.motionExportFile())
			+ "  " + ofToString(session.motionExportFrames()) + " frames";
		ofDrawBitmapStringHighlight(label, x + 14, y, ofColor(0, 180), ofColor(120, 220, 160));
	}
	ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::drawStatsBar(){
	const StreamStats stats = session.streamStats();
	const SensorSnapshot & sensors = session.getSensors();
	const float y = ofGetHeight() - kBarHeight;
	ofSetColor(0, 200);
	ofDrawRectangle(0, y, ofGetWidth(), kBarHeight);

	// tracking summary shared by both modes
	std::string trackInfo;
	if(trackingParam){
		size_t objCount = 0;
		for(const auto & c : trackResults.corridors){
			for(const auto & o : c.objects) if(o.confirmed) objCount++;
		}
		trackInfo = "   trk[" + (trackResults.layoutName.empty() ? std::string("-") : trackResults.layoutName)
			+ (showResultParam ? " result" : " raw")
			+ "] " + ofToString(objCount) + " obj"
			+ " dec " + ofToString(trackResults.decodeMs, 1)
			+ " det " + ofToString(trackResults.detectMs, 1)
			+ " trk " + ofToString(trackResults.trackMs, 1)
			+ " shp " + ofToString(trackResults.shapeMs, 1) + " ms";
	}

	if(session.getMode() == Mode::Playback){
		const std::string line1 = "[playback] " + session.recordingName()
			+ "   recording " + ofToString(session.recordingIndex() + 1)
			+ "/" + ofToString(session.recordingCount());
		const std::string line2 =
			formatClock(session.positionSec() * 1000.0) + " / " + formatClock(session.durationSec() * 1000.0)
			+ "   frame " + ofToString(session.frameCursor()) + "/" + ofToString(session.frameCount())
			+ "   dec " + ofToString(session.playbackDecodeMs(), 1) + " ms"
			+ trackInfo
			+ (session.isPaused() ? "   PAUSED" : "")
			+ "   SPACE play/pause   LEFT/RIGHT prev/next   UP/DOWN +/-1s   TAB back to live"
			+ "   app " + ofToString(ofGetFrameRate(), 0) + " fps";
		ofDrawBitmapStringHighlight(line1, 12, y + 24, ofColor(0, 0, 0, 0), ofColor(120, 180, 255));
		ofDrawBitmapStringHighlight(line2, 12, y + 46, ofColor(0, 0, 0, 0), ofColor(220));
		ofSetColor(255);
		return;
	}

	ofColor stateColor = ofColor(180);
	if(stats.state == "streaming") stateColor = ofColor(90, 220, 120);
	else if(stats.state == "connecting" || stats.state == "reconnecting") stateColor = ofColor(255, 190, 60);
	else if(!stats.lastError.empty()) stateColor = ofColor(240, 90, 90);

	const double frameAgeMs = session.liveFrameAgeMs();
	std::string line1 = "[" + stats.state + "] " + session.streamUrl();
	if(!stats.lastError.empty()) line1 += "  error: " + stats.lastError;
	std::string line2 =
		"decoded " + ofToString(stats.decodedFps, 1) + " fps"
		+ "   camera " + (sensors.frj < 0 ? "n/a" : ofToString(sensors.cameraFps(), 1) + " fps")
		+ "   " + ofToString(stats.bytesPerSecond / (1024.0 * 1024.0), 2) + " MB/s"
		+ "   frame age " + (frameAgeMs < 0 ? "n/a" : ofToString(frameAgeMs, 0) + " ms")
		+ "   dec " + ofToString(stats.lastDecodeMs, 1) + " ms"
		+ "   ->preview " + ofToString(session.liveFrameLatencyMs(), 1) + " ms"
		+ "   frames " + ofToString(stats.framesDecoded)
		+ " dropped " + ofToString(stats.framesDropped)
		+ "   lux " + (sensors.la2 < 0 ? "n/a" : ofToString(sensors.multiSenseLux(), 1))
		+ trackInfo
		+ "   app " + ofToString(ofGetFrameRate(), 0) + " fps";

	ofDrawBitmapStringHighlight(line1, 12, y + 24, ofColor(0, 0, 0, 0), stateColor);
	ofDrawBitmapStringHighlight(line2, 12, y + 46, ofColor(0, 0, 0, 0), ofColor(220));
	ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::exit(){
	recordParam.removeListener(this, &ofApp::onRecordChanged);
	exportParam.removeListener(this, &ofApp::onExportChanged);
	playbackParam.removeListener(this, &ofApp::onPlaybackChanged);
	trackingParam.removeListener(this, &ofApp::onTrackingChanged);
	reconnectButton.removeListener(this, &ofApp::onReconnectPressed);
	saveButton.removeListener(this, &ofApp::onSavePressed);
	playParam.removeListener(this, &ofApp::onPlayChanged);
	playheadParam.removeListener(this, &ofApp::onPlayheadChanged);
	prevFrameButton.removeListener(this, &ofApp::onPrevFramePressed);
	nextFrameButton.removeListener(this, &ofApp::onNextFramePressed);
	prevRecordingButton.removeListener(this, &ofApp::onPrevRecordingPressed);
	nextRecordingButton.removeListener(this, &ofApp::onNextRecordingPressed);
	session.stop();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	const bool playback = session.getMode() == Mode::Playback;
	if((key == OF_KEY_BACKSPACE || key == OF_KEY_DEL)
	   && anchorLane >= 0 && anchorLane < static_cast<int>(editLayout.lanes.size())){
		tracking::LaneConfig & lane = editLayout.lanes[anchorLane];
		const tracking::LaneConfig backup = lane;
		if(lane.removeStation(anchorStation)){
			if(lane.isWarpable()){
				anchorStation = -1;
				commitLayout();
			}else{
				lane = backup;
				ofLogWarning("ofApp") << "lane '" << lane.id
					<< "' would be crossed without that anchor - keeping it";
			}
		}
		return;
	}
	if(key == 'r'){
		onReconnectPressed();
	}else if(key == ' '){
		if(playback) playParam = !playParam;
		else recordParam = !recordParam;
	}else if(key == 'e'){
		exportParam = !exportParam;
	}else if(key == 't'){
		trackingParam = !trackingParam;
	}else if(key == 'o'){
		overlayParam = !overlayParam;
	}else if(key == 'm'){
		showMaskParam = !showMaskParam;
	}else if(key == 'd'){
		showDetectionsParam = !showDetectionsParam;
	}else if(key == 'x'){
		showPixelsParam = !showPixelsParam;
	}else if(key == 'n'){
		showOutlineParam = !showOutlineParam;
	}else if(key == 'b'){
		showBoxParam = !showBoxParam;
	}else if(key == 'q'){
		showResultParam = !showResultParam;
	}else if(key == '['){
		if(laneSelectParam > laneSelectParam.getMin()) laneSelectParam = laneSelectParam - 1;
	}else if(key == ']'){
		if(laneSelectParam < laneSelectParam.getMax()) laneSelectParam = laneSelectParam + 1;
	}else if(key == 'v'){
		resultViewParam = resultViewParam == 0 ? 1 : 0;
	}else if(key == ','){
		if(corridorSelectParam > corridorSelectParam.getMin()) corridorSelectParam = corridorSelectParam - 1;
	}else if(key == '.'){
		if(corridorSelectParam < corridorSelectParam.getMax()) corridorSelectParam = corridorSelectParam + 1;
	}else if(key == OF_KEY_TAB){
		playbackParam = !playbackParam;
	}else if(key == OF_KEY_RIGHT){
		onNextRecordingPressed();
	}else if(key == OF_KEY_LEFT){
		onPrevRecordingPressed();
	}else if(key == OF_KEY_UP){
		session.seekBy(1.0);
	}else if(key == OF_KEY_DOWN){
		session.seekBy(-1.0);
	}
}

//--------------------------------------------------------------
// lane editing: drag a station of any lane strip (grabbing one selects that
// lane). Shift-click or double-click a top or bottom edge to insert a
// station there; the other border gains the matching point so the slice
// can still be flattened. The edit is committed on release, and only when
// every slice is convex — a crossed shape stays red and keeps the last
// valid lane active.

void ofApp::mousePressed(int x, int y, int){
	if(!overlayParam) return; // handles are hidden -> nothing to grab
	if(gui.getShape().inside(x, y) || editGui.getShape().inside(x, y)) return;
	if(session.getMode() == Mode::Playback && playbackGui.getShape().inside(x, y)) return;

	float bestD2 = kFrameHandleHitRadius * kFrameHandleHitRadius;
	dragLane = -1;
	dragEdge = dragStation = -1;
	// the selected lane wins ties (checked first)
	const int sel = laneSelectParam.get();
	for(int pass = 0; pass < 2; pass++){
		for(size_t li = 0; li < editLayout.lanes.size(); li++){
			const bool isSel = static_cast<int>(li) == sel;
			if((pass == 0) != isSel) continue;
			const tracking::LaneConfig & lane = editLayout.lanes[li];
			if(lane.pane < 0 || lane.pane >= static_cast<int>(paneRects.size())) continue;
			const ofRectangle & pane = paneRects[lane.pane];
			if(pane.width <= 0 || pane.height <= 0) continue;
			std::vector<std::array<float, 2>> top, bottom;
			lane.borders(top, bottom);
			const auto consider = [&](int edge, const std::vector<std::array<float, 2>> & pts){
				for(size_t i = 0; i < pts.size(); i++){
					const float hx = pane.x + pts[i][0] * pane.width;
					const float hy = pane.y + pts[i][1] * pane.height;
					const float d2 = (hx - x) * (hx - x) + (hy - y) * (hy - y);
					if(d2 < bestD2){
						bestD2 = d2;
						dragLane = static_cast<int>(li);
						dragEdge = edge;
						dragStation = static_cast<int>(i);
					}
				}
			};
			consider(0, top);
			consider(1, bottom);
		}
		if(dragLane >= 0) break;
	}

	const uint64_t now = ofGetElapsedTimeMillis();
	static uint64_t lastMs = 0;
	static int lastX = 0, lastY = 0;
	const bool dbl = now - lastMs < 400 && std::hypot(x - lastX, y - lastY) < 8.0f;
	const bool shift = ofGetKeyPressed(OF_KEY_LEFT_SHIFT) || ofGetKeyPressed(OF_KEY_RIGHT_SHIFT);
	lastMs = now;
	lastX = x;
	lastY = y;

	// No handle: shift-click or double-click on a border inserts a station
	// and grabs it, so the new point can be pulled off the edge at once.
	if(dragLane < 0 && (shift || dbl)){
		float bestEdge = 14.0f;
		int insLane = -1, insEdge = 0, insSeg = 0;
		float insT = 0;
		for(int pass = 0; pass < 2; pass++){
			for(size_t li = 0; li < editLayout.lanes.size(); li++){
				const bool isSel = static_cast<int>(li) == sel;
				if((pass == 0) != isSel) continue;
				const tracking::LaneConfig & lane = editLayout.lanes[li];
				if(lane.pane < 0 || lane.pane >= static_cast<int>(paneRects.size())) continue;
				const ofRectangle & pane = paneRects[lane.pane];
				if(pane.width <= 0 || pane.height <= 0) continue;
				int edge = 0, seg = 0;
				float t = 0, dist = 0;
				if(!nearestBorder(lane, pane, x, y, edge, seg, t, dist)) continue;
				if(dist < bestEdge && t > 0.05f && t < 0.95f){
					bestEdge = dist;
					insLane = static_cast<int>(li);
					insEdge = edge;
					insSeg = seg;
					insT = t;
				}
			}
			if(insLane >= 0) break;
		}
		if(insLane >= 0){
			tracking::LaneConfig & lane = editLayout.lanes[insLane];
			const ofRectangle & pane = paneRects[lane.pane];
			float ux, uy;
			clampLaneDrag(previewGeometry, pane, x, y, ux, uy);
			const int at = lane.insertStation(insEdge, insSeg, insT, ux, uy);
			if(at >= 0){
				dragLane = insLane;
				dragEdge = insEdge;
				dragStation = at;
			}else{
				ofLogWarning("ofApp") << "lane '" << lane.id << "' already has "
					<< tracking::LaneConfig::kMaxStations << " anchors per border";
			}
		}
	}

	if(dragLane >= 0){
		if(dragLane != sel) laneSelectParam = dragLane;
		anchorLane = dragLane;
		anchorEdge = dragEdge;
		anchorStation = dragStation;
	}
	if(dragLane < 0 && resultViewParam == 0) mousePressCanvas(x, y);
}

// Canvas view: grab a corner handle of a placement (the selected corridor
// wins ties) or a placement body to move it. Returns true when something
// was grabbed.
bool ofApp::mousePressCanvas(int x, int y){
	dragCorridor = -1;
	dragPlaceHandle = -1;
	if(canvasRect.width <= 0 || !canvasViewRect.inside(x, y)) return false;
	const std::vector<std::string> ids = session.corridorIds();
	if(ids.empty()) return false;
	const tracking::TrackingConfig & cfg = session.getConfig();
	const int sel = ofClamp(corridorSelectParam.get(), 0, static_cast<int>(ids.size()) - 1);
	const float cx = (x - canvasRect.x) / canvasRect.width;
	const float cy = (y - canvasRect.y) / canvasRect.height;

	// corners first (selected corridor, then the others)
	float bestD2 = kFrameHandleHitRadius * kFrameHandleHitRadius;
	for(int pass = 0; pass < 2 && dragCorridor < 0; pass++){
		for(size_t i = 0; i < ids.size(); i++){
			if((pass == 0) != (static_cast<int>(i) == sel)) continue;
			const tracking::CorridorPlacement p = cfg.placementFor(ids[i]);
			const float hx[4] = {p.x0, p.x1, p.x1, p.x0}, hy[4] = {p.y0, p.y0, p.y1, p.y1};
			for(int h = 0; h < 4; h++){
				const float sx = canvasRect.x + hx[h] * canvasRect.width;
				const float sy = canvasRect.y + hy[h] * canvasRect.height;
				const float d2 = (sx - x) * (sx - x) + (sy - y) * (sy - y);
				if(d2 < bestD2){
					bestD2 = d2;
					dragCorridor = static_cast<int>(i);
					dragPlaceHandle = h;
				}
			}
		}
	}
	// then bodies
	for(int pass = 0; pass < 2 && dragCorridor < 0; pass++){
		for(size_t i = 0; i < ids.size(); i++){
			if((pass == 0) != (static_cast<int>(i) == sel)) continue;
			const tracking::CorridorPlacement p = cfg.placementFor(ids[i]);
			if(cx >= std::min(p.x0, p.x1) && cx <= std::max(p.x0, p.x1)
			   && cy >= std::min(p.y0, p.y1) && cy <= std::max(p.y0, p.y1)){
				dragCorridor = static_cast<int>(i);
				dragPlaceHandle = 4;
				dragGrabX = cx - p.x0;
				dragGrabY = cy - p.y0;
				break;
			}
		}
	}
	if(dragCorridor >= 0 && dragCorridor != sel) corridorSelectParam = dragCorridor;
	return dragCorridor >= 0;
}

void ofApp::mouseDragCanvas(int x, int y){
	const std::vector<std::string> ids = session.corridorIds();
	if(dragCorridor < 0 || dragCorridor >= static_cast<int>(ids.size()) || canvasRect.width <= 0) return;
	// placements may leave the canvas (that cuts the corridor) but stay
	// within the slider range
	const float cx = ofClamp((x - canvasRect.x) / canvasRect.width, -1.0f, 2.0f);
	const float cy = ofClamp((y - canvasRect.y) / canvasRect.height, -1.0f, 2.0f);
	tracking::CorridorPlacement p = session.getConfig().placementFor(ids[dragCorridor]);
	switch(dragPlaceHandle){
		case 0: p.x0 = cx; p.y0 = cy; break;
		case 1: p.x1 = cx; p.y0 = cy; break;
		case 2: p.x1 = cx; p.y1 = cy; break;
		case 3: p.x0 = cx; p.y1 = cy; break;
		case 4: {
			const float w = p.x1 - p.x0, h = p.y1 - p.y0;
			p.x0 = cx - dragGrabX;
			p.y0 = cy - dragGrabY;
			p.x1 = p.x0 + w;
			p.y1 = p.y0 + h;
			break;
		}
		default: break;
	}
	session.setPlacement(ids[dragCorridor], p); // applied at publish time, no rebuild
}

void ofApp::mouseDragged(int x, int y, int){
	if(dragCorridor >= 0){
		mouseDragCanvas(x, y);
		return;
	}
	if(dragLane < 0 || dragLane >= static_cast<int>(editLayout.lanes.size())) return;
	tracking::LaneConfig & lane = editLayout.lanes[dragLane];
	if(lane.pane < 0 || lane.pane >= static_cast<int>(paneRects.size())) return;
	const ofRectangle & pane = paneRects[lane.pane];
	if(pane.width <= 0 || pane.height <= 0) return;
	float ux, uy;
	clampLaneDrag(previewGeometry, pane, x, y, ux, uy);
	lane.setStation(dragEdge, dragStation, ux, uy);
}

void ofApp::mouseReleased(int, int, int){
	if(dragCorridor >= 0){
		dragCorridor = -1;
		dragPlaceHandle = -1;
		lastCorridorSelect = -1; // re-sync the placement sliders
		return;
	}
	if(dragLane < 0) return;
	if(dragLane < static_cast<int>(editLayout.lanes.size()) && editLayout.lanes[dragLane].isWarpable()){
		commitLayout();
	}else if(dragLane < static_cast<int>(editLayout.lanes.size())){
		ofLogWarning("ofApp") << "lane '" << editLayout.lanes[dragLane].id
			<< "' is crossed/collapsed - keeping the last valid lane active";
	}
	dragLane = -1;
	dragEdge = dragStation = -1;
}

//--------------------------------------------------------------
// gui panels and listeners

// Main panel on top; in playback mode the transport panel sits under it
// and the edit panel moves down below that. The transport panel only
// listens to the mouse while it is shown — ofxGui would otherwise keep
// hit-testing the hidden widgets.
void ofApp::layoutGuiPanels(){
	const bool playback = session.getMode() == Mode::Playback;
	if(playback != playbackGuiActive){
		if(playback) playbackGui.registerMouseEvents();
		else playbackGui.unregisterMouseEvents();
		playbackGuiActive = playback;
	}
	float y = gui.getShape().getBottom() + 8;
	if(playback){
		playbackGui.setPosition(10, y);
		y = playbackGui.getShape().getBottom() + 8;
	}
	editGui.setPosition(10, y);
}

void ofApp::onPlayChanged(bool & value){
	if(value) session.play();
	else session.pause();
}

void ofApp::onPlayheadChanged(float & value){
	if(playheadSync) return; // mirrored from the player, not a scrub
	session.seekTo(value);
}

void ofApp::onPrevFramePressed(){
	session.stepFrame(-1);
}

void ofApp::onNextFramePressed(){
	session.stepFrame(1);
}

void ofApp::onPrevRecordingPressed(){
	session.previousRecording();
}

void ofApp::onNextRecordingPressed(){
	session.nextRecording();
}

void ofApp::onRecordChanged(bool & value){
	if(value){
		if(!session.startRecording()) recordParam = false; // nested listener call is a no-op
	}else{
		session.stopRecording();
	}
}

void ofApp::onExportChanged(bool & value){
	if(value) session.startMotionExport();
	else session.stopMotionExport();
}

void ofApp::onPlaybackChanged(bool & value){
	if(value){
		if(recordParam) recordParam = false; // stops an active recording
		if(!session.setMode(Mode::Playback)){
			playbackParam = false; // stay live
			return;
		}
		playParam = true; // scan() starts playing; keep the toggle in step
	}else{
		session.setMode(Mode::Live);
	}
	texture.clear();
	layoutGuiPanels();
}

void ofApp::onTrackingChanged(bool & value){
	session.setTrackingEnabled(value);
}

void ofApp::onReconnectPressed(){
	session.reconnect();
}

void ofApp::onSavePressed(){
	session.saveConfig();
}
