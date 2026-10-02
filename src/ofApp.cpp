#include "ofApp.h"

#include <chrono>

#include <opencv2/imgproc.hpp>

#include "output/MotionPathWriter.h"
#include "tracking/TrackingConfigJson.h"
#include "tracking/TrackingParams.h"

namespace {
constexpr double kSettingsDebounceMs = 600;
// Live tracking tuning is pushed to the worker at most this often while a
// slider moves (avoids recreating the MOG2 model on every slider pixel).
constexpr double kTuningPushIntervalMs = 250;
constexpr float kFrameHandleRadius = 6;     // drawn size
constexpr float kFrameHandleHitRadius = 12; // mouse pick radius
constexpr float kMaxArrow = 0.25f;          // velocity arrow clamp (lane widths)

// Homography mapping lane coordinates (0..1, unit square) onto the four
// screen-space corners of a lane quad. Returns a 3x3 CV_64F matrix.
cv::Mat laneToScreenHomography(const tracking::FrameQuad & quad, const ofRectangle & pane){
	const cv::Point2f src[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
	cv::Point2f dst[4];
	for(int c = 0; c < 4; c++){
		dst[c] = cv::Point2f(pane.x + quad.pts[c][0] * pane.width,
		                     pane.y + quad.pts[c][1] * pane.height);
	}
	return cv::getPerspectiveTransform(src, dst);
}

glm::vec2 projectPoint(const cv::Mat & h, float x, float y){
	const double * m = h.ptr<double>();
	const double w = m[6] * x + m[7] * y + m[8];
	return {static_cast<float>((m[0] * x + m[1] * y + m[2]) / w),
	        static_cast<float>((m[3] * x + m[4] * y + m[5]) / w)};
}

bool sameParams(const std::vector<tracking::ParamInfo> & a, const std::vector<tracking::ParamInfo> & b){
	if(a.size() != b.size()) return false;
	for(size_t i = 0; i < a.size(); i++){
		if(a[i].key != b[i].key || a[i].value != b[i].value) return false;
	}
	return true;
}

bool sameLayouts(const std::vector<tracking::LayoutConfig> & a, const std::vector<tracking::LayoutConfig> & b){
	if(a.size() != b.size()) return false;
	for(size_t i = 0; i < a.size(); i++){
		if(a[i].name != b[i].name || a[i].panes != b[i].panes
		   || a[i].recordings != b[i].recordings || a[i].lanes != b[i].lanes) return false;
	}
	return true;
}

bool sameCorridors(const std::vector<tracking::CorridorConfig> & a, const std::vector<tracking::CorridorConfig> & b){
	if(a.size() != b.size()) return false;
	for(size_t i = 0; i < a.size(); i++){
		if(a[i].id != b[i].id || a[i].label != b[i].label || a[i].placement != b[i].placement
		   || a[i].flow != b[i].flow || a[i].tracker != b[i].tracker || a[i].shape != b[i].shape) return false;
	}
	return true;
}

// Fieldwise comparison of the live-tunable parts of a TrackingConfig.
bool tuningEquals(const tracking::TrackingConfig & a, const tracking::TrackingConfig & b){
	return sameParams(tracking::flowParams(a.flow), tracking::flowParams(b.flow))
		&& sameParams(tracking::trackerParams(a.tracker), tracking::trackerParams(b.tracker))
		&& sameParams(tracking::shapeParams(a.shape), tracking::shapeParams(b.shape))
		&& a.bgs.history == b.bgs.history
		&& a.bgs.varThreshold == b.bgs.varThreshold
		&& a.yolo.confThreshold == b.yolo.confThreshold
		&& a.yolo.nmsThreshold == b.yolo.nmsThreshold
		&& a.canvas == b.canvas
		&& a.lens == b.lens
		&& sameCorridors(a.corridors, b.corridors)
		&& sameLayouts(a.layouts, b.layouts);
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

std::string formatLux(float lux){
	return lux < 0 ? "n/a" : ofToString(lux, 1) + " lx";
}
std::string formatTemp(float temp){
	return temp < 0 ? "n/a" : ofToString(temp, 1) + " C";
}
std::string formatClock(double ms){
	const int totalSec = static_cast<int>(ms / 1000.0);
	const int m = totalSec / 60;
	const int s = totalSec % 60;
	return ofToString(m, 2, '0') + ":" + ofToString(s, 2, '0');
}

double wallClockSec(){
	using namespace std::chrono;
	return duration<double>(system_clock::now().time_since_epoch()).count();
}

// ofTexture::loadData only reallocates when the new frame is LARGER than the
// current allocation; a smaller frame is uploaded into a sub-region and the
// texture keeps its old size (and stale pixels outside the region). Frame
// sizes change when playback cycles between dual- and single-camera
// recordings, when toggling half-res decode, or live when toggling the
// preview stream — so reallocate whenever the dimensions differ.
void loadFrameTexture(ofTexture & tex, const ofPixels & pixels){
	if(tex.isAllocated()
	   && (static_cast<size_t>(tex.getWidth()) != pixels.getWidth()
	       || static_cast<size_t>(tex.getHeight()) != pixels.getHeight())){
		tex.clear();
	}
	tex.loadData(pixels);
}

// The picture on screen is the lens-corrected frame. src stays untouched
// so the same pixels can be rewarped when the sliders move.
void uploadCorrected(ofTexture & tex, const ofPixels & src, LensCorrector & corr){
	if(!src.isAllocated()) return;
	const int w = static_cast<int>(src.getWidth());
	const int h = static_cast<int>(src.getHeight());
	const int ch = static_cast<int>(src.getNumChannels());
	const int type = ch == 4 ? CV_8UC4 : ch == 3 ? CV_8UC3 : CV_8UC1;
	const cv::Mat in(h, w, type, const_cast<unsigned char *>(src.getData()), src.getBytesStride());
	cv::Mat out;
	corr.apply(in, out);
	if(out.empty()) return;
	ofPixels pix;
	const ofPixelFormat fmt = ch == 4 ? OF_PIXELS_RGBA : ch == 3 ? OF_PIXELS_RGB : OF_PIXELS_GRAY;
	pix.setFromPixels(out.data, static_cast<size_t>(out.cols), static_cast<size_t>(out.rows), fmt);
	loadFrameTexture(tex, pix);
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
} // namespace

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetFrameRate(60);
	ofSetVerticalSync(true);
	ofBackground(12);

	loadConfig();

	gui.setup("stream", "gui.xml");
	gui.add(previewParam);
	gui.add(qualityParam);
	gui.add(showComDumpParam);
	gui.add(recordParam);
	gui.add(playbackParam);
	gui.add(trackingParam);
	gui.add(overlayParam);
	gui.add(showMaskParam);
	gui.add(showDetectionsParam);
	gui.add(showPixelsParam);
	gui.add(showOutlineParam);
	gui.add(showBoxParam);
	gui.add(resultViewParam);
	gui.add(detectorParam);
	gui.add(halfResParam);
	gui.add(udpParam);
	reconnectButton.setup("reconnect (r)");
	gui.add(&reconnectButton);
	gui.setPosition(10, 10);

	// tracking tuning panel. Values always come from config.json (assigned
	// below, after setup — so a stale tracking_gui.xml can never shadow the
	// config) and go back to config.json via the save button.
	trackingGui.setup("tracking", "tracking_gui.xml");
	trackingGui.add(lensEnabledParam);
	trackingGui.add(lensFovParam);
	trackingGui.add(lensK1Param);
	trackingGui.add(lensK2Param);
	trackingGui.add(lensBalanceParam);
	trackingGui.add(lensCenterXParam);
	trackingGui.add(lensCenterYParam);
	trackingGui.add(laneSelectParam);
	trackingGui.add(laneS0Param);
	trackingGui.add(laneS1Param);
	trackingGui.add(corridorSelectParam);
	trackingGui.add(placeX0Param);
	trackingGui.add(placeX1Param);
	trackingGui.add(placeY0Param);
	trackingGui.add(placeY1Param);
	trackingGui.add(flowMinFlowParam);
	trackingGui.add(flowEmaParam);
	trackingGui.add(flowBusyThreshParam);
	trackingGui.add(flowUseMogParam);
	trackingGui.add(flowMogVarParam);
	trackingGui.add(flowCloseWParam);
	trackingGui.add(flowCloseHParam);
	trackingGui.add(flowMinAreaParam);
	trackingGui.add(flowMinCoherenceParam);
	trackingGui.add(flowMinSpeedParam);
	trackingGui.add(flowMergeGapParam);
	trackingGui.add(trkConfirmParam);
	trackingGui.add(trkMaxMissesParam);
	trackingGui.add(trkGateGapParam);
	trackingGui.add(trkAbsorbGapParam);
	trackingGui.add(trkMergeFramesParam);
	trackingGui.add(trkReacquireMsParam);
	trackingGui.add(trkVelMeasNoiseParam);
	trackingGui.add(trkTrailParam);
	trackingGui.add(shapeSourceParam);
	trackingGui.add(shapeEmaParam);
	trackingGui.add(shapeThreshParam);
	trackingGui.add(shapePadXParam);
	trackingGui.add(shapePadYParam);
	trackingGui.add(shapeSimplifyParam);
	trackingGui.add(yoloConfParam);
	trackingGui.add(yoloNmsParam);
	trackingGui.add(arrowScaleParam);
	saveTrackingButton.setup("save to config.json");
	trackingGui.add(&saveTrackingButton);
	trackingGui.setPosition(10, gui.getShape().getBottom() + 8);
	saveTrackingButton.addListener(this, &ofApp::onSaveTrackingPressed);

	loadGuiFromConfig();

	previewParam = camConfig.preview;
	qualityParam = camConfig.quality;
	recordParam = false;   // never restore a stale "recording" state from gui.xml
	playbackParam = false; // always start live
	trackingParam = trackingConfig.enabled;
	detectorParam = trackingConfig.detector == "yolo" ? 2 : trackingConfig.detector == "bgs" ? 1 : 0;
	halfResParam = (displayDecodeScale >= 2);
	udpParam = udpEnabled;
	previewParam.addListener(this, &ofApp::onPreviewChanged);
	qualityParam.addListener(this, &ofApp::onQualityChanged);
	recordParam.addListener(this, &ofApp::onRecordChanged);
	playbackParam.addListener(this, &ofApp::onPlaybackChanged);
	trackingParam.addListener(this, &ofApp::onTrackingChanged);
	detectorParam.addListener(this, &ofApp::onDetectorChanged);
	halfResParam.addListener(this, &ofApp::onHalfResChanged);
	udpParam.addListener(this, &ofApp::onUdpChanged);
	reconnectButton.addListener(this, &ofApp::onReconnectPressed);

	// tracking worker (fed below by the raw-frame taps)
	trackingConfig.enabled = trackingParam;
	trackingConfig.sourceName = currentSourceName();
	lastSourceName = trackingConfig.sourceName;
	trackingManager.setup(trackingConfig);
	pushedTuning = currentTrackingTuning(); // worker already has this state

	if(udpEnabled) udp.open(udpHost, udpPort);

	// Raw-frame taps: the exact JPEG bytes of every complete frame, called
	// on the stream / playback thread. Recording and tracking both hang off
	// these; the main thread never touches raw frames.
	client.setDecodeScale(halfResParam ? 2 : 1);
	client.setRawFrameCallback([this](const uint8_t * data, size_t size, double tsMs){
		if(recorder.isRecording()) recorder.addFrame(data, size, tsMs);
		if(mode == AppMode::Live) trackingManager.submitRawJpeg(data, size, tsMs);
	});
	player.setDecodeScale(halfResParam ? 2 : 1);
	player.setRawFrameCallback([this](const uint8_t * data, size_t size, double tMs){
		if(mode == AppMode::Playback) trackingManager.submitRawJpeg(data, size, tMs);
	});

	if(camConfig.enabled && !camConfig.host.empty()){
		ofLogNotice("ofApp") << "starting stream: " << camConfig.buildStreamUrl();
		client.start(camConfig);
	}else{
		ofLogError("ofApp") << "no enabled camera in config/config.json";
	}

	// Headless verification: CAMTRACK_AUTOSHOT=<path> saves a screenshot
	// after CAMTRACK_AUTOSHOT_SEC (default 8) seconds and quits.
	// CAMTRACK_AUTOPLAYBACK=1 additionally starts in playback mode (tracking
	// against a recording, no camera); CAMTRACK_RECORDINGS overrides the
	// recordings folder, so a single recording can be played by pointing it at
	// a folder holding just that one.
	if(const char * autoshot = std::getenv("CAMTRACK_AUTOSHOT")){
		autoshotPath = autoshot;
		if(const char * sec = std::getenv("CAMTRACK_AUTOSHOT_SEC")) autoshotSec = std::max(1.0f, static_cast<float>(std::atof(sec)));
		ofLogNotice("ofApp") << "autoshot enabled -> " << autoshotPath << " after " << autoshotSec << " s";
	}
	if(std::getenv("CAMTRACK_AUTOPLAYBACK")){
		ofLogNotice("ofApp") << "autoplayback enabled";
		if(const char * dir = std::getenv("CAMTRACK_RECORDINGS")) recordingsDir = ofToDataPath(dir, true);
		playbackParam = true;
	}
}

//--------------------------------------------------------------
void ofApp::loadConfig(){
	recordingsDir = ofToDataPath("recordings", true);

	std::string path = "config/config.json";
	if(!ofFile::doesFileExist(path)){
		path = "config/config.example.json";
		ofLogWarning("ofApp") << "config/config.json not found, falling back to " << path;
	}
	configPath = path; // save target for the tracking-settings button
	ofJson json;
	try {
		json = ofLoadJson(path);
	} catch(const std::exception & e){
		ofLogError("ofApp") << "failed to load " << path << ": " << e.what();
		return;
	}
	const auto loaded = tracking::loadTrackingConfig(json, [](const std::string & p){ return ofToDataPath(p, true); });
	trackingConfig = loaded.config;
	overlayArrowScale = loaded.overlayArrowScale;
	displayDecodeScale = loaded.displayDecodeScale;
	recordingsDir = ofToDataPath(loaded.recordingsDir, true);

	if(json.contains("output") && json["output"].contains("udp")){
		const auto & u = json["output"]["udp"];
		udpEnabled = u.value("enabled", false);
		udpHost = u.value("host", std::string("127.0.0.1"));
		udpPort = u.value("port", 8765);
	}

	if(!json.contains("cameras") || !json["cameras"].is_array()){
		ofLogError("ofApp") << path << " has no cameras array";
		return;
	}
	for(const auto & cam : json["cameras"]){
		if(!cam.value("enabled", false)) continue;
		camConfig.id = cam.value("id", "camera");
		camConfig.enabled = true;
		camConfig.host = cam.value("host", "");
		camConfig.user = cam.value("user", "");
		camConfig.password = cam.value("password", "");
		camConfig.stream = cam.value("stream", "full");
		camConfig.fps = cam.value("fps", 0.0f);
		camConfig.preview = cam.value("preview", false);
		camConfig.previewSize = cam.value("previewSize", "1280x720");
		camConfig.quality = cam.value("quality", 60);
		break; // v1: first enabled camera only
	}
}

void ofApp::loadGuiFromConfig(){
	const auto & lens = trackingConfig.lens;
	lensEnabledParam = lens.enabled;
	lensFovParam = lens.fovDeg;
	lensK1Param = lens.k1;
	lensK2Param = lens.k2;
	lensBalanceParam = lens.balance;
	lensCenterXParam = lens.centerX;
	lensCenterYParam = lens.centerY;
	const auto & f = trackingConfig.flow;
	flowMinFlowParam = f.minFlowPx;
	flowEmaParam = f.flowEma;
	flowBusyThreshParam = f.busyThresh;
	flowUseMogParam = f.useMog2;
	flowMogVarParam = f.mogVarThreshold;
	flowCloseWParam = f.closeWFrac;
	flowCloseHParam = f.closeHFrac;
	flowMinAreaParam = f.minAreaFrac;
	flowMinCoherenceParam = f.minCoherence;
	flowMinSpeedParam = f.minSpeed;
	flowMergeGapParam = f.mergeGapFrac;
	const auto & k = trackingConfig.tracker;
	trkConfirmParam = k.confirmFrames;
	trkMaxMissesParam = k.maxMisses;
	trkGateGapParam = k.gateGapFrac;
	trkAbsorbGapParam = k.absorbGapFrac;
	trkMergeFramesParam = k.mergeFrames;
	trkReacquireMsParam = static_cast<float>(k.reacquireMs);
	trkVelMeasNoiseParam = k.velMeasNoise;
	trkTrailParam = k.trailLen;
	const auto & s = trackingConfig.shape;
	shapeSourceParam = s.source;
	shapeEmaParam = s.ema;
	shapeThreshParam = s.threshold;
	shapePadXParam = s.padX;
	shapePadYParam = s.padY;
	shapeSimplifyParam = s.simplify;
	yoloConfParam = trackingConfig.yolo.confThreshold;
	yoloNmsParam = trackingConfig.yolo.nmsThreshold;
	arrowScaleParam = overlayArrowScale;
}

std::string ofApp::currentSourceName() const {
	return mode == AppMode::Playback ? player.getCurrentName() : std::string("live");
}

//--------------------------------------------------------------
// The loaded config (corridors, model path, analysisReduce, layouts with
// the committed lane edits) with the live GUI values written over it — the
// exact state the worker should run with, and what the save button writes.
tracking::TrackingConfig ofApp::currentTrackingTuning() const {
	tracking::TrackingConfig c = trackingConfig;
	c.lens.enabled = lensEnabledParam;
	c.lens.fovDeg = lensFovParam;
	c.lens.k1 = lensK1Param;
	c.lens.k2 = lensK2Param;
	c.lens.balance = lensBalanceParam;
	c.lens.centerX = lensCenterXParam;
	c.lens.centerY = lensCenterYParam;
	c.flow.minFlowPx = flowMinFlowParam;
	c.flow.flowEma = flowEmaParam;
	c.flow.busyThresh = flowBusyThreshParam;
	c.flow.useMog2 = flowUseMogParam;
	c.flow.mogVarThreshold = flowMogVarParam;
	c.flow.closeWFrac = flowCloseWParam;
	c.flow.closeHFrac = flowCloseHParam;
	c.flow.minAreaFrac = flowMinAreaParam;
	c.flow.minCoherence = flowMinCoherenceParam;
	c.flow.minSpeed = flowMinSpeedParam;
	c.flow.mergeGapFrac = flowMergeGapParam;
	c.tracker.confirmFrames = trkConfirmParam;
	c.tracker.maxMisses = trkMaxMissesParam;
	c.tracker.gateGapFrac = trkGateGapParam;
	c.tracker.absorbGapFrac = trkAbsorbGapParam;
	c.tracker.mergeFrames = trkMergeFramesParam;
	c.tracker.reacquireMs = trkReacquireMsParam;
	c.tracker.velMeasNoise = trkVelMeasNoiseParam;
	c.tracker.trailLen = trkTrailParam;
	c.shape.source = shapeSourceParam;
	c.shape.ema = shapeEmaParam;
	c.shape.threshold = shapeThreshParam;
	c.shape.padX = shapePadXParam;
	c.shape.padY = shapePadYParam;
	c.shape.simplify = shapeSimplifyParam;
	c.yolo.confThreshold = yoloConfParam;
	c.yolo.nmsThreshold = yoloNmsParam;
	return c;
}

void ofApp::pushTrackingTuning(){
	const double now = MobotixMjpegClient::nowMs();
	if(now - lastTuningPushMs < kTuningPushIntervalMs) return;
	const tracking::TrackingConfig tuning = currentTrackingTuning();
	if(tuningEquals(tuning, pushedTuning)) return;
	trackingManager.setLiveConfig(tuning);
	pushedTuning = tuning;
	lastTuningPushMs = now;
}

// Save button: write the current tuning + layouts back into the loaded
// config file, touching only the tracking keys (cameras etc. stay as-is).
void ofApp::onSaveTrackingPressed(){
	ofJson json;
	try {
		json = ofLoadJson(configPath);
	} catch(const std::exception & e){
		ofLogError("ofApp") << "save: failed to re-read " << configPath << ": " << e.what();
	}
	if(!json.is_object()) json = ofJson::object();

	tracking::TrackingConfig tuning = currentTrackingTuning();
	tuning.detector = detectorParam == 2 ? "yolo" : detectorParam == 1 ? "bgs" : "flow";
	tracking::writeTrackingTuning(json, tuning, arrowScaleParam.get());

	if(ofSavePrettyJson(configPath, json)){
		ofLogNotice("ofApp") << "tracking settings saved to " << configPath;
	}else{
		ofLogError("ofApp") << "failed to save tracking settings to " << configPath;
	}
}

//--------------------------------------------------------------
// Lane editing state follows the current source / pane count: the layout
// the worker resolves for it becomes the editable copy. A synthesized
// fallback layout is materialized into trackingConfig.layouts on the first
// commit so it can be saved.
void ofApp::syncEditLayout(int paneCount){
	const std::string source = currentSourceName();
	if(paneCount == editPaneCount && source == lastSourceName) return;
	if(source != lastSourceName){
		lastSourceName = source;
		trackingManager.setSourceName(source);
	}
	editPaneCount = paneCount;
	editLayout = tracking::TrackingManager::findLayout(trackingConfig.layouts, paneCount, source);
	committedLayout = editLayout;
	dragLane = dragCorner = -1;
	laneSelectParam.setMax(std::max(0, static_cast<int>(editLayout.lanes.size()) - 1));
	laneSelectParam = std::min(laneSelectParam.get(), laneSelectParam.getMax());
	lastLaneSelect = -1; // re-sync the s0/s1 sliders
	laneMaskTex.assign(editLayout.lanes.size(), ofTexture());
}

void ofApp::commitLayout(){
	committedLayout = editLayout;
	auto it = std::find_if(trackingConfig.layouts.begin(), trackingConfig.layouts.end(),
	                       [&](const tracking::LayoutConfig & l){ return l.name == committedLayout.name; });
	if(it != trackingConfig.layouts.end()){
		*it = committedLayout;
	}else{
		// a synthesized fallback layout gets a proper name once edited
		committedLayout.name = editPaneCount > 1 ? "dual" : "single";
		editLayout.name = committedLayout.name;
		trackingConfig.layouts.push_back(committedLayout);
	}
	// picked up by pushTrackingTuning() in update(); the worker rebuilds
	// its lanes (background models and tracks restart)
}

// Corridors that can be placed: the configured ones (config order) plus any
// the edited layout references without a config entry.
std::vector<std::string> ofApp::editCorridorIds() const {
	std::vector<std::string> ids;
	for(const auto & c : trackingConfig.corridors) ids.push_back(c.id);
	for(const auto & lane : editLayout.lanes){
		if(std::find(ids.begin(), ids.end(), lane.corridor) == ids.end()) ids.push_back(lane.corridor);
	}
	return ids;
}

tracking::CorridorPlacement & ofApp::placementRef(const std::string & id){
	for(auto & c : trackingConfig.corridors){
		if(c.id == id) return c.placement;
	}
	tracking::CorridorConfig cc;
	cc.id = id;
	cc.label = id;
	trackingConfig.corridors.push_back(cc);
	return trackingConfig.corridors.back().placement;
}

//--------------------------------------------------------------
void ofApp::applyStreamSettings(){
	camConfig.preview = previewParam;
	camConfig.quality = qualityParam;
	client.restart(camConfig);
}

//--------------------------------------------------------------
void ofApp::update(){
	bool gotFrame = false;
	if(mode == AppMode::Playback){
		// live stream keeps running in the background but is ignored;
		// the player paces/reads/decodes on its own thread.
		ofPixels pixels;
		if(player.getLatestFrame(pixels, sensors, playbackSeenFrameId, playbackFrameTMs,
		                         showComDumpParam ? &comDump : nullptr)){
			rawFrame = pixels;
			rawFrameMode = AppMode::Playback;
			gotFrame = true;
		}
	}else{
		ofPixels pixels;
		double receivedAtMs = 0;
		if(client.getLatestFrame(pixels, sensors, lastSeenFrameId, receivedAtMs,
		                         showComDumpParam ? &comDump : nullptr)){
			rawFrame = pixels;
			rawFrameMode = AppMode::Live;
			gotFrame = true;
			lastFrameReceivedMs = receivedAtMs;
			lastUploadLatencyMs = MobotixMjpegClient::nowMs() - receivedAtMs;
		}
	}
	// Show the corrected frame, and rewarp the last one as soon as a lens
	// slider moves (playback may be sitting on one frame).
	if(rawFrame.isAllocated() && rawFrameMode == mode){
		const LensCorrection lens = currentTrackingTuning().lens;
		if(gotFrame || lens != shownLens){
			shownLens = lens;
			displayLens.set(lens);
			uploadCorrected(mode == AppMode::Playback ? playbackTexture : texture, rawFrame, displayLens);
		}
	}

	// newest tracking results (published by the tracking thread)
	if(trackingManager.getLatestResults(trackResults, trackResultsRevision)){
		if(udp.isOpen()){
			const double tSec = trackResults.frameTMs / 1000.0;
			const ofJson frame = output::buildFrame(output::fromTrackingResults(trackResults), tSec,
			                                        ++udpFrameIndex, 30.0, currentSourceName(),
			                                        trackResults.canvas.width, trackResults.canvas.height,
			                                        wallClockSec());
			udp.send(frame.dump());
		}
		// lane masks -> textures (only when shown)
		if(showMaskParam){
			if(laneMaskTex.size() < trackResults.lanes.size()) laneMaskTex.resize(trackResults.lanes.size());
			for(size_t i = 0; i < trackResults.lanes.size(); i++){
				const auto & dbg = trackResults.lanes[i];
				if(dbg.maskW <= 0 || dbg.maskH <= 0 || dbg.mask.empty()) continue;
				ofPixels p;
				p.setFromPixels(dbg.mask.data(), dbg.maskW, dbg.maskH, OF_PIXELS_GRAY);
				loadFrameTexture(laneMaskTex[i], p);
			}
		}
	}

	// lane selector -> s0/s1 sliders (and back)
	if(!editLayout.lanes.empty()){
		const int sel = ofClamp(laneSelectParam.get(), 0, static_cast<int>(editLayout.lanes.size()) - 1);
		if(sel != lastLaneSelect){
			lastLaneSelect = sel;
			laneSliderSync = true;
			laneS0Param = editLayout.lanes[sel].s0;
			laneS1Param = editLayout.lanes[sel].s1;
			laneSliderSync = false;
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
		const std::vector<std::string> ids = editCorridorIds();
		corridorSelectParam.setMax(std::max(0, static_cast<int>(ids.size()) - 1));
		if(!ids.empty()){
			const int sel = ofClamp(corridorSelectParam.get(), 0, static_cast<int>(ids.size()) - 1);
			if(sel != corridorSelectParam.get()) corridorSelectParam = sel;
			const tracking::CorridorPlacement current = trackingConfig.placementFor(ids[sel]);
			if(sel != lastCorridorSelect || dragCorridor >= 0){
				lastCorridorSelect = sel;
				placeX0Param = current.x0;
				placeX1Param = current.x1;
				placeY0Param = current.y0;
				placeY1Param = current.y1;
			}else if(current.x0 != placeX0Param.get() || current.x1 != placeX1Param.get()
			         || current.y0 != placeY0Param.get() || current.y1 != placeY1Param.get()){
				tracking::CorridorPlacement & p = placementRef(ids[sel]);
				p.x0 = placeX0Param;
				p.x1 = placeX1Param;
				p.y0 = placeY0Param;
				p.y1 = placeY1Param;
			}
		}
	}

	// live tracking tuning: any slider / committed-lane / placement change
	// goes to the worker, throttled so drags do not rebuild models on every
	// pixel (placements need no rebuild — they are applied at publish time)
	pushTrackingTuning();

	if(settingsDirty && MobotixMjpegClient::nowMs() - settingsDirtySinceMs > kSettingsDebounceMs){
		settingsDirty = false;
		applyStreamSettings();
	}

	// periodic stats log (useful when running headless / from terminal)
	if(ofGetElapsedTimeMillis() - lastStatsLogMs > 5000){
		lastStatsLogMs = ofGetElapsedTimeMillis();
		const StreamStats stats = client.getStats();
		ofLogNotice("ofApp")
			<< "[" << stats.state << "] decoded " << ofToString(stats.decodedFps, 1)
			<< " fps, " << ofToString(stats.bytesPerSecond / (1024.0 * 1024.0), 2)
			<< " MB/s, frames " << stats.framesDecoded
			<< ", MultiSense " << ofToString(sensors.multiSenseLux(), 1) << " lx (LA2=" << sensors.la2
			<< "), LXL=" << sensors.lxl << " LXR=" << sensors.lxr
			<< ", PIR=" << sensors.pi2 << "%"
			<< (stats.lastError.empty() ? "" : ", error: " + stats.lastError);
	}

	if(!autoshotPath.empty() && !autoshotDone && ofGetElapsedTimef() > autoshotSec){
		autoshotDone = true;
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	drawVideo();
	drawRecordingIndicator();
	drawSensorPanel();
	drawStatsBar();
	if(showComDumpParam) drawComDump();
	gui.draw();
	trackingGui.draw();

	if(autoshotDone && !autoshotPath.empty()){
		ofSaveScreen(autoshotPath);
		ofLogNotice("ofApp") << "autoshot saved to " << autoshotPath << ", exiting";
		autoshotPath.clear();
		ofExit();
	}
}

//--------------------------------------------------------------
void ofApp::drawVideo(){
	const float barHeight = 64;
	const float panelWidth = 320;
	const float splitGap = 8;
	// keep the image clear of the GUI panels on the left
	const float guiWidth = std::max(gui.getShape().getRight(), trackingGui.getShape().getRight()) + splitGap;
	const ofRectangle full(guiWidth, 0, ofGetWidth() - panelWidth - guiWidth, ofGetHeight() - barHeight);
	// top: the lens-corrected image(s) with lane editor + overlay; bottom: the
	// rectified lane views (motion mask + tracking, no camera pixels)
	const ofRectangle viewport(full.x, full.y, full.width, full.height * 0.6f);
	const ofRectangle resultView(full.x, viewport.getBottom() + splitGap,
	                             full.width, full.height * 0.4f - splitGap);

	const bool playback = (mode == AppMode::Playback);
	const ofTexture & tex = playback ? playbackTexture : texture;

	paneRects.clear();
	laneResultRects.clear();

	if(!tex.isAllocated()){
		ofSetColor(160);
		const std::string msg = playback
			? "waiting for playback frame (" + player.getCurrentName() + ")"
			: "waiting for stream: " + client.getStreamUrl();
		ofDrawBitmapString(msg, viewport.getCenter().x - 240, viewport.getCenter().y);
		ofSetColor(255);
		return;
	}

	ofSetColor(255);
	const bool splitBoth = tex.getWidth() >= tex.getHeight() * 2;

	if(splitBoth){
		// one BOTH frame = M1 | M2 side by side; draw as two panes with a gap
		const float halfW = tex.getWidth() / 2.0f;
		const float texH = tex.getHeight();
		const float gap = 4;
		const float paneW = (viewport.width - gap) / 2.0f;
		const float scale = std::min(paneW / halfW, viewport.height / texH);
		const float drawW = halfW * scale;
		const float drawH = texH * scale;
		const float y = viewport.y + (viewport.height - drawH) / 2.0f;
		const float x0 = viewport.x + (paneW - drawW) / 2.0f;
		const float x1 = viewport.x + paneW + gap + (paneW - drawW) / 2.0f;
		tex.drawSubsection(x0, y, drawW, drawH, 0, 0, halfW, texH);
		tex.drawSubsection(x1, y, drawW, drawH, halfW, 0, halfW, texH);
		paneRects.emplace_back(x0, y, drawW, drawH); // module 0 (M1)
		paneRects.emplace_back(x1, y, drawW, drawH); // module 1 (M2)
		ofSetColor(200);
		ofDrawBitmapString("M1 (left)", x0 + 6, y + 16);
		ofDrawBitmapString("M2 (right)", x1 + 6, y + 16);
		ofSetColor(255);
	}else{
		const float scale = std::min(viewport.width / tex.getWidth(),
		                             viewport.height / tex.getHeight());
		const float drawW = tex.getWidth() * scale;
		const float drawH = tex.getHeight() * scale;
		const float x = viewport.x + (viewport.width - drawW) / 2.0f;
		const float y = viewport.y + (viewport.height - drawH) / 2.0f;
		tex.draw(x, y, drawW, drawH);
		paneRects.emplace_back(x, y, drawW, drawH); // single module
	}

	syncEditLayout(static_cast<int>(paneRects.size()));

	// lane result rects: one column per pane, the pane's lanes stacked in
	// rows; each cell keeps the rectified lane's aspect
	const float texPaneW = tex.getWidth() / paneRects.size();
	const float texPaneH = tex.getHeight();
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
		const float aspect = lane.quad.aspect(texPaneW, texPaneH);
		float w = paneRects[lane.pane].width;
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
		const float aspect = trackingConfig.canvas.height > 0
			? static_cast<float>(trackingConfig.canvas.width) / trackingConfig.canvas.height : 1.0f;
		float w = resultView.width * 0.62f;
		float h = w / aspect;
		if(h > resultView.height * 0.72f){ h = resultView.height * 0.72f; w = h * aspect; }
		canvasViewRect = resultView;
		canvasRect = ofRectangle(resultView.getCenter().x - w / 2.0f, resultView.getCenter().y - h / 2.0f, w, h);
	}

	drawTrackingOverlay();
	drawLaneEditor();
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
	const std::vector<std::string> ids = editCorridorIds();
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
	ofDrawBitmapStringHighlight("canvas " + ofToString(trackingConfig.canvas.width) + "x"
		+ ofToString(trackingConfig.canvas.height) + "  (0,0 top-left, 1,1 bottom-right)",
		c.x + 6, c.y - 6, ofColor(0, 140), ofColor(200));

	// placements
	for(size_t i = 0; i < ids.size(); i++){
		const tracking::CorridorPlacement p = trackingConfig.placementFor(ids[i]);
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
			const tracking::CorridorPlacement p = trackingConfig.placementFor(cr.id);
			const CorridorToScreen mapCanvas = [&](float s, float y, glm::vec2 & out){
				out = toScreen(p.toCanvasX(s), p.toCanvasY(y));
				return true;
			};
			for(const auto & obj : cr.objects){
				float x0, x1, y0, y1;
				p.mapBox(obj.x0, obj.x1, obj.y0, obj.y1, x0, x1, y0, y1);
				const ofColor col = objectColor(obj);
				if(showPixelsParam) drawShapeCells(obj.shape, mapCanvas, ofColor(col, 110));
				if(showOutlineParam && obj.shape.valid && !obj.shape.outline.empty()){
					// uncut outline dim, the published (canvas-clipped) one bright
					drawShapeOutline(obj.shape.outline, mapCanvas, ofColor(col, 60), 1, 0);
					std::vector<std::pair<float, float>> onCanvas;
					for(const auto & q : obj.shape.outline) onCanvas.emplace_back(p.toCanvasX(q.first), p.toCanvasY(q.second));
					const auto clipped = output::clipPolygonToCanvas(onCanvas);
					drawShapeOutline(clipped, [&](float cx, float cy, glm::vec2 & out){ out = toScreen(cx, cy); return true; },
					                 col, obj.confirmed ? 2 : 1, 40);
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

				if(obj.trail.size() > 1){
					for(size_t i = 1; i < obj.trail.size(); i++){
						const glm::vec2 t0 = toScreen(p.toCanvasX(obj.trail[i - 1].first), p.toCanvasY(obj.trail[i - 1].second));
						const glm::vec2 t1 = toScreen(p.toCanvasX(obj.trail[i].first), p.toCanvasY(obj.trail[i].second));
						if(!c.inside(t0) || !c.inside(t1)) continue;
						const float alpha = static_cast<float>(i) / obj.trail.size();
						ofSetColor(col.r, col.g, col.b, col.a * alpha * 0.6f);
						ofDrawLine(t0, t1);
					}
				}
				if(showBoxParam){
					ofNoFill();
					ofSetColor(col);
					ofSetLineWidth(obj.confirmed ? 2 : 1);
					ofDrawRectangle(a.x, a.y, b.x - a.x, b.y - a.y);
					ofSetLineWidth(1);
					const bool cutL = offL || (p.mirrored() ? obj.clippedRight : obj.clippedLeft);
					const bool cutR = offR || (p.mirrored() ? obj.clippedLeft : obj.clippedRight);
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
				float dx = obj.vx * p.spanX() * arrowScaleParam, dy = obj.vy * p.spanY() * arrowScaleParam;
				const float len = std::hypot(dx, dy);
				if(len > kMaxArrow){ dx *= kMaxArrow / len; dy *= kMaxArrow / len; }
				ofDrawArrow(glm::vec3(ctr, 0), glm::vec3(ctr.x + dx * c.width, ctr.y + dy * c.height, 0), 5.0f);
				std::string text = "#" + ofToString(obj.id) + " " + obj.label + " " + obj.phase
					+ "  x=" + ofToString((x0 + x1) * 0.5f, 2) + " vx=" + ofToString(obj.vx * p.spanX(), 3);
				if(obj.shape.valid && !obj.shape.source.empty()) text += "  shape:" + obj.shape.source;
				if(!obj.confirmed) text += " ?";
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

	ofPushStyle();
	std::vector<cv::Mat> laneH(trackResults.lanes.size());
	for(size_t li = 0; li < trackResults.lanes.size(); li++){
		const tracking::LaneConfig & lane = trackResults.lanes[li].lane;
		if(lane.pane < 0 || lane.pane >= static_cast<int>(paneRects.size())) continue;
		const tracking::FrameQuad quad = lane.quad.isConvex() ? lane.quad : tracking::FrameQuad{};
		laneH[li] = laneToScreenHomography(quad, paneRects[lane.pane]);
		if(showDetectionsParam){
			ofSetColor(170, 170, 170, 160);
			ofNoFill();
			for(const auto & d : trackResults.lanes[li].detections){
				const glm::vec2 p[4] = {projectPoint(laneH[li], d.x0, d.y0), projectPoint(laneH[li], d.x1, d.y0),
				                        projectPoint(laneH[li], d.x1, d.y1), projectPoint(laneH[li], d.x0, d.y1)};
				for(int c = 0; c < 4; c++) ofDrawLine(p[c], p[(c + 1) % 4]);
			}
		}
	}

	for(const auto & cr : trackResults.corridors){
		for(const auto & obj : cr.objects){
			const ofColor col = objectColor(obj);
			bool labelled = false;
			for(int li : cr.lanes){
				if(li < 0 || li >= static_cast<int>(laneH.size()) || laneH[li].empty()) continue;
				const tracking::LaneConfig & lane = trackResults.lanes[li].lane;
				const cv::Mat & h = laneH[li];
				const auto proj = [&h](float x, float y){ return projectPoint(h, x, y); };

				float lx0, lx1;
				if(!lane.corridorRangeToLane(obj.x0, obj.x1, lx0, lx1)) continue;

				// trail (older = more transparent), in this lane where visible
				if(obj.trail.size() > 1){
					for(size_t i = 1; i < obj.trail.size(); i++){
						const float a0 = lane.toLane(obj.trail[i - 1].first);
						const float a1 = lane.toLane(obj.trail[i].first);
						if(a0 < 0 || a0 > 1 || a1 < 0 || a1 > 1) continue;
						const float a = static_cast<float>(i) / obj.trail.size();
						ofSetColor(col.r, col.g, col.b, col.a * a * 0.6f);
						ofDrawLine(proj(a0, obj.trail[i - 1].second), proj(a1, obj.trail[i].second));
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
					drawShapeOutline(obj.shape.outline, mapLane, col, obj.confirmed ? 2 : 1, 40);
				}

				ofSetColor(col);
				const glm::vec2 bTl = proj(lx0, obj.y0), bTr = proj(lx1, obj.y0);
				const glm::vec2 bBr = proj(lx1, obj.y1), bBl = proj(lx0, obj.y1);
				if(showBoxParam){
					ofNoFill();
					ofSetLineWidth(obj.confirmed ? 2 : 1);
					ofDrawLine(bTl, bTr);
					ofDrawLine(bTr, bBr);
					ofDrawLine(bBr, bBl);
					ofDrawLine(bBl, bTl);
					ofSetLineWidth(1);
					// clipped edge markers (red) where the corridor end is in this lane
					float e0, e1;
					if(obj.clippedLeft && lane.corridorRangeToLane(0.0f, 0.002f, e0, e1)){
						ofSetColor(255, 60, 60);
						ofSetLineWidth(3);
						ofDrawLine(bTl, bBl);
						ofSetLineWidth(1);
					}
					if(obj.clippedRight && lane.corridorRangeToLane(0.998f, 1.0f, e0, e1)){
						ofSetColor(255, 60, 60);
						ofSetLineWidth(3);
						ofDrawLine(bTr, bBr);
						ofSetLineWidth(1);
					}
				}
				ofSetColor(col);

				// center + velocity arrow (corridor -> lane velocity scale)
				const float lcx = lane.toLane(obj.x);
				if(lcx >= 0 && lcx <= 1){
					const glm::vec2 c = proj(lcx, obj.y);
					ofFill();
					ofDrawCircle(c.x, c.y, 3);
					const float span = lane.s1 - lane.s0;
					float dx = (std::fabs(span) > 1e-6f ? obj.vx / span : obj.vx) * arrowScaleParam;
					float dy = obj.vy * arrowScaleParam;
					const float len = std::hypot(dx, dy);
					if(len > kMaxArrow){ dx *= kMaxArrow / len; dy *= kMaxArrow / len; }
					const glm::vec2 tip = proj(lcx + dx, obj.y + dy);
					ofDrawArrow(glm::vec3(c.x, c.y, 0), glm::vec3(tip.x, tip.y, 0), 5.0f);
				}

				if(!labelled){
					std::string text = "#" + ofToString(obj.id);
					if(!obj.label.empty()) text += " " + obj.label;
					text += " " + obj.phase + "  v=" + ofToString(obj.vx, 3);
					if(!obj.confirmed) text += " ?";
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
// The draggable lane quads on each pane: outline + corner handles. The
// selected lane is yellow with handles, other lanes cyan; red = crossed /
// collapsed quad — drawn but NOT sent to the tracker until it is convex.
void ofApp::drawLaneEditor(){
	if(!overlayParam) return;

	ofPushStyle();
	for(size_t li = 0; li < editLayout.lanes.size(); li++){
		const tracking::LaneConfig & lane = editLayout.lanes[li];
		if(lane.pane < 0 || lane.pane >= static_cast<int>(paneRects.size())) continue;
		const ofRectangle & pane = paneRects[lane.pane];
		const bool selected = static_cast<int>(li) == laneSelectParam.get();
		const bool valid = lane.quad.isConvex();
		const ofColor col = !valid ? ofColor(240, 80, 80)
			: selected ? ofColor(255, 210, 60) : ofColor(80, 200, 230);

		glm::vec2 pts[4];
		for(int c = 0; c < 4; c++){
			pts[c] = {pane.x + lane.quad.pts[c][0] * pane.width,
			          pane.y + lane.quad.pts[c][1] * pane.height};
		}

		ofSetColor(col, selected ? 220 : 130);
		ofNoFill();
		for(int c = 0; c < 4; c++) ofDrawLine(pts[c], pts[(c + 1) % 4]);
		ofDrawBitmapStringHighlight(lane.id + " -> " + lane.corridor
			+ " s " + ofToString(lane.s0, 2) + ".." + ofToString(lane.s1, 2),
			pts[0].x + 4, pts[0].y + 12, ofColor(0, 140), col);

		ofFill();
		for(int c = 0; c < 4; c++){
			const bool dragged = (static_cast<int>(li) == dragLane && c == dragCorner);
			ofSetColor(col, dragged ? 255 : selected ? 200 : 110);
			ofDrawCircle(pts[c].x, pts[c].y, dragged ? kFrameHandleRadius + 2 : selected ? kFrameHandleRadius : 4);
		}
		if(!valid){
			ofSetColor(col);
			ofDrawBitmapStringHighlight("invalid lane quad (crossed) - keeping the last valid one",
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
		ofDrawBitmapStringHighlight(lane.id + "  (lane 0..1, corridor " + lane.corridor + ")",
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
				float lx0, lx1;
				if(!lane.corridorRangeToLane(obj.x0, obj.x1, lx0, lx1)) continue;
				const ofColor col = objectColor(obj);
				const float bx = r.x + lx0 * r.width, by = r.y + obj.y0 * r.height;
				const float bw = (lx1 - lx0) * r.width, bh = obj.h * r.height;

				if(obj.trail.size() > 1){
					for(size_t i = 1; i < obj.trail.size(); i++){
						const float a0 = lane.toLane(obj.trail[i - 1].first);
						const float a1 = lane.toLane(obj.trail[i].first);
						if(a0 < 0 || a0 > 1 || a1 < 0 || a1 > 1) continue;
						const float a = static_cast<float>(i) / obj.trail.size();
						ofSetColor(col.r, col.g, col.b, col.a * a * 0.6f);
						ofDrawLine(r.x + a0 * r.width, r.y + obj.trail[i - 1].second * r.height,
						           r.x + a1 * r.width, r.y + obj.trail[i].second * r.height);
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
					drawShapeOutline(obj.shape.outline, mapStrip, col, obj.confirmed ? 2 : 1, 40);
				}

				if(showBoxParam){
					ofSetColor(col);
					ofNoFill();
					ofSetLineWidth(obj.confirmed ? 2 : 1);
					ofDrawRectangle(bx, by, bw, bh);
					ofSetLineWidth(1);
					float e0, e1;
					if(obj.clippedLeft && lane.corridorRangeToLane(0.0f, 0.002f, e0, e1)){
						ofSetColor(255, 60, 60);
						ofSetLineWidth(3);
						ofDrawLine(bx, by, bx, by + bh);
						ofSetLineWidth(1);
					}
					if(obj.clippedRight && lane.corridorRangeToLane(0.998f, 1.0f, e0, e1)){
						ofSetColor(255, 60, 60);
						ofSetLineWidth(3);
						ofDrawLine(bx + bw, by, bx + bw, by + bh);
						ofSetLineWidth(1);
					}
				}
				ofSetColor(col);
				const float lcx = lane.toLane(obj.x);
				if(lcx >= 0 && lcx <= 1){
					const float cx = r.x + lcx * r.width, cy = r.y + obj.y * r.height;
					ofFill();
					ofDrawCircle(cx, cy, 3);
					const float span = lane.s1 - lane.s0;
					float dx = (std::fabs(span) > 1e-6f ? obj.vx / span : obj.vx) * arrowScaleParam;
					float dy = obj.vy * arrowScaleParam;
					const float len = std::hypot(dx, dy);
					if(len > kMaxArrow){ dx *= kMaxArrow / len; dy *= kMaxArrow / len; }
					ofDrawArrow(glm::vec3(cx, cy, 0), glm::vec3(cx + dx * r.width, cy + dy * r.height, 0), 5.0f);
				}
				std::string text = "#" + ofToString(obj.id) + " " + obj.phase
					+ "  s=" + ofToString(obj.x, 2) + " w=" + ofToString(obj.w, 2)
					+ " v=" + ofToString(obj.vx, 3);
				if(!obj.confirmed) text += " ?";
				ofDrawBitmapStringHighlight(text, bx, by - 6, ofColor(0, 160), col);
			}
		}
	}
	ofPopStyle();
	ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::drawRecordingIndicator(){
	if(!recorder.isRecording()) return;
	const float x = 250; // right of the gui panel
	const float y = 26;
	const bool blinkOn = (static_cast<int>(ofGetElapsedTimef() * 2) % 2) == 0;
	if(blinkOn){
		ofSetColor(230, 40, 40);
		ofDrawCircle(x, y - 4, 7);
	}
	const std::string label = "REC " + formatClock(recorder.getDurationMs())
		+ "  " + ofToString(recorder.getFramesWritten()) + " frames  "
		+ ofToString(recorder.getBytesWritten() / (1024.0 * 1024.0), 1) + " MB";
	ofDrawBitmapStringHighlight(label, x + 14, y, ofColor(0, 180), ofColor(255, 90, 90));
	ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::drawSensorPanel(){
	const float panelWidth = 320;
	const float x = ofGetWidth() - panelWidth;
	const float barHeight = 64;
	ofSetColor(0, 180);
	ofDrawRectangle(x, 0, panelWidth, ofGetHeight() - barHeight);
	ofSetColor(255);

	float y = 30;
	const float lx = x + 16;
	ofDrawBitmapStringHighlight("SENSORS (" + camConfig.id + ")", lx, y);
	y += 34;

	// Mx-F-MSA MultiSense (M3) — the brightness-control source
	ofPushMatrix();
	ofTranslate(lx, y);
	ofScale(2, 2);
	ofDrawBitmapStringHighlight("MultiSense M3", 0, 0, ofColor(0, 0, 0, 0), ofColor(255, 210, 60));
	ofDrawBitmapStringHighlight(formatLux(sensors.multiSenseLux()), 0, 18,
	                            ofColor(0, 0, 0, 0), ofColor(255, 210, 60));
	ofPopMatrix();
	y += 60;
	ofDrawBitmapString("raw LA2 = " + (sensors.la2 < 0 ? "n/a" : ofToString(sensors.la2)), lx, y);
	y += 16;
	ofDrawBitmapString("PIR  PI2 = " + (sensors.pi2 < 0 ? "n/a" : ofToString(sensors.pi2) + " %"), lx, y);
	y += 16;
	ofDrawBitmapString("temp TC2 = " + (sensors.tc2 < 0 ? "n/a" : ofToString(sensors.tc2)) + "  (" + formatTemp(sensors.multiSenseTempC()) + ")", lx, y);
	y += 34;

	ofDrawBitmapStringHighlight("Optical sensors", lx, y);
	y += 22;
	ofDrawBitmapString("left  LXL = " + (sensors.lxl < 0 ? "n/a" : ofToString(sensors.lxl)) + "  (" + formatLux(sensors.leftLux()) + ")", lx, y);
	y += 16;
	ofDrawBitmapString("right LXR = " + (sensors.lxr < 0 ? "n/a" : ofToString(sensors.lxr)) + "  (" + formatLux(sensors.rightLux()) + ")", lx, y);
	y += 34;

	ofDrawBitmapStringHighlight("Camera", lx, y);
	y += 22;
	ofDrawBitmapString("board TIN = " + (sensors.tin < 0 ? "n/a" : ofToString(sensors.tin)) + "  (" + formatTemp(sensors.boardTempC()) + ")", lx, y);
	y += 16;
	ofDrawBitmapString("cam fps FRJ = " + (sensors.frj < 0 ? "n/a" : ofToString(sensors.cameraFps(), 1)), lx, y);
	y += 16;
	ofDrawBitmapString("image = " + ofToString(sensors.width) + "x" + ofToString(sensors.height) + " " + sensors.cam, lx, y);
	y += 34;

	ofDrawBitmapStringHighlight("Tracking", lx, y);
	y += 22;
	ofDrawBitmapString("layout = " + (trackResults.layoutName.empty() ? std::string("-") : trackResults.layoutName)
		+ "  lanes " + ofToString(trackResults.lanes.size()), lx, y);
	y += 16;
	for(const auto & cr : trackResults.corridors){
		int confirmed = 0;
		for(const auto & o : cr.objects) if(o.confirmed) confirmed++;
		ofDrawBitmapString(cr.id + " (" + cr.label + "): " + ofToString(confirmed) + " obj"
			+ (cr.objects.size() > static_cast<size_t>(confirmed)
				? " +" + ofToString(cr.objects.size() - confirmed) + " tentative" : ""), lx, y);
		y += 16;
	}
	if(udp.isOpen()){
		ofDrawBitmapString("udp " + udp.getHost() + ":" + ofToString(udp.getPort())
			+ "  sent " + ofToString(udp.getSentCount()), lx, y);
		y += 16;
	}
}

//--------------------------------------------------------------
void ofApp::drawStatsBar(){
	const StreamStats stats = client.getStats();
	const float barHeight = 64;
	const float y = ofGetHeight() - barHeight;
	ofSetColor(0, 200);
	ofDrawRectangle(0, y, ofGetWidth(), barHeight);

	// tracking summary shared by both modes
	std::string trackInfo;
	if(trackingParam){
		size_t objCount = 0;
		for(const auto & c : trackResults.corridors){
			for(const auto & o : c.objects) if(o.confirmed) objCount++;
		}
		trackInfo = "   trk[" + (trackResults.detectorName.empty() ? std::string("-") : trackResults.detectorName)
			+ "] " + ofToString(objCount) + " obj"
			+ " dec " + ofToString(trackResults.decodeMs, 1)
			+ " det " + ofToString(trackResults.detectMs, 1)
			+ " trk " + ofToString(trackResults.trackMs, 1)
			+ " shp " + ofToString(trackResults.shapeMs, 1) + " ms";
	}

	if(mode == AppMode::Playback){
		const std::string line1 = "[playback] " + player.getCurrentName()
			+ "   recording " + ofToString(player.getCurrentIndex() + 1)
			+ "/" + ofToString(player.getCount());
		const std::string line2 =
			formatClock(player.getPositionMs()) + " / " + formatClock(player.getDurationMs())
			+ "   frame " + ofToString(player.getFrameCursor()) + "/" + ofToString(player.getFrameCount())
			+ "   dec " + ofToString(player.getLastDecodeMs(), 1) + " ms"
			+ trackInfo
			+ "   LEFT/RIGHT prev/next   UP/DOWN +/-1s   TAB back to live"
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

	const double frameAgeMs = lastFrameReceivedMs > 0
		? MobotixMjpegClient::nowMs() - lastFrameReceivedMs : -1;

	std::string line1 = "[" + stats.state + "] " + client.getStreamUrl();
	if(!stats.lastError.empty()) line1 += "  error: " + stats.lastError;
	std::string line2 =
		"decoded " + ofToString(stats.decodedFps, 1) + " fps"
		+ "   camera " + (sensors.frj < 0 ? "n/a" : ofToString(sensors.cameraFps(), 1) + " fps")
		+ "   " + ofToString(stats.bytesPerSecond / (1024.0 * 1024.0), 2) + " MB/s"
		+ "   frame age " + (frameAgeMs < 0 ? "n/a" : ofToString(frameAgeMs, 0) + " ms")
		+ "   dec " + ofToString(stats.lastDecodeMs, 1) + " ms"
		+ "   ->tex " + ofToString(lastUploadLatencyMs, 1) + " ms"
		+ "   frames " + ofToString(stats.framesDecoded)
		+ " dropped " + ofToString(stats.framesDropped)
		+ trackInfo
		+ "   app " + ofToString(ofGetFrameRate(), 0) + " fps";

	ofDrawBitmapStringHighlight(line1, 12, y + 24, ofColor(0, 0, 0, 0), stateColor);
	ofDrawBitmapStringHighlight(line2, 12, y + 46, ofColor(0, 0, 0, 0), ofColor(220));
	ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::drawComDump(){
	if(comDump.empty()) return;
	// right of the gui panels (the tracking panel now occupies the old spot)
	const float x = std::max(gui.getShape().getRight(), trackingGui.getShape().getRight()) + 10;
	ofSetColor(0, 210);
	ofDrawRectangle(x, 220, 420, ofGetHeight() - 300);
	ofSetColor(180, 255, 180);
	ofDrawBitmapString(comDump, x + 8, 240);
	ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::exit(){
	previewParam.removeListener(this, &ofApp::onPreviewChanged);
	qualityParam.removeListener(this, &ofApp::onQualityChanged);
	recordParam.removeListener(this, &ofApp::onRecordChanged);
	playbackParam.removeListener(this, &ofApp::onPlaybackChanged);
	trackingParam.removeListener(this, &ofApp::onTrackingChanged);
	detectorParam.removeListener(this, &ofApp::onDetectorChanged);
	halfResParam.removeListener(this, &ofApp::onHalfResChanged);
	udpParam.removeListener(this, &ofApp::onUdpChanged);
	reconnectButton.removeListener(this, &ofApp::onReconnectPressed);
	saveTrackingButton.removeListener(this, &ofApp::onSaveTrackingPressed);
	stopRecording();
	client.stop();
	player.close();
	trackingManager.stop();
	udp.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if(key == 'r'){
		onReconnectPressed();
	}else if(key == 'p'){
		previewParam = !previewParam;
	}else if(key == 'c'){
		showComDumpParam = !showComDumpParam;
	}else if(key == 's'){
		saveScreenshot();
	}else if(key == ' '){
		recordParam = !recordParam; // rejected in playback mode
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
		if(mode == AppMode::Playback){
			player.next();
			trackingManager.reset();
		}
	}else if(key == OF_KEY_LEFT){
		if(mode == AppMode::Playback){
			player.previous();
			trackingManager.reset();
		}
	}else if(key == OF_KEY_UP){
		if(mode == AppMode::Playback) player.seekBy(1000);
	}else if(key == OF_KEY_DOWN){
		if(mode == AppMode::Playback) player.seekBy(-1000);
	}
}

//--------------------------------------------------------------
// lane editing: drag a corner handle of any lane quad (grabbing one selects
// that lane). The edit is committed (sent to the worker) on release, and
// only when the quad is convex — a crossed shape stays visible in red and
// keeps the last valid lane active.

void ofApp::mousePressed(int x, int y, int){
	if(!overlayParam) return; // handles are hidden -> nothing to grab
	if(gui.getShape().inside(x, y) || trackingGui.getShape().inside(x, y)) return;

	float bestD2 = kFrameHandleHitRadius * kFrameHandleHitRadius;
	dragLane = -1;
	dragCorner = -1;
	// the selected lane wins ties (checked first with a slight bonus)
	const int sel = laneSelectParam.get();
	for(int pass = 0; pass < 2; pass++){
		for(size_t li = 0; li < editLayout.lanes.size(); li++){
			const bool isSel = static_cast<int>(li) == sel;
			if((pass == 0) != isSel) continue;
			const tracking::LaneConfig & lane = editLayout.lanes[li];
			if(lane.pane < 0 || lane.pane >= static_cast<int>(paneRects.size())) continue;
			const ofRectangle & pane = paneRects[lane.pane];
			if(pane.width <= 0 || pane.height <= 0) continue;
			for(int c = 0; c < 4; c++){
				const float hx = pane.x + lane.quad.pts[c][0] * pane.width;
				const float hy = pane.y + lane.quad.pts[c][1] * pane.height;
				const float d2 = (hx - x) * (hx - x) + (hy - y) * (hy - y);
				if(d2 < bestD2){
					bestD2 = d2;
					dragLane = static_cast<int>(li);
					dragCorner = c;
				}
			}
		}
		if(dragLane >= 0) break;
	}
	if(dragLane >= 0 && dragLane != sel) laneSelectParam = dragLane;
	if(dragLane < 0 && resultViewParam == 0) mousePressCanvas(x, y);
}

// Canvas view: grab a corner handle of a placement (the selected corridor
// wins ties) or a placement body to move it. Returns true when something
// was grabbed.
bool ofApp::mousePressCanvas(int x, int y){
	dragCorridor = -1;
	dragPlaceHandle = -1;
	if(canvasRect.width <= 0 || !canvasViewRect.inside(x, y)) return false;
	const std::vector<std::string> ids = editCorridorIds();
	if(ids.empty()) return false;
	const int sel = ofClamp(corridorSelectParam.get(), 0, static_cast<int>(ids.size()) - 1);
	const float cx = (x - canvasRect.x) / canvasRect.width;
	const float cy = (y - canvasRect.y) / canvasRect.height;

	// corners first (selected corridor, then the others)
	float bestD2 = kFrameHandleHitRadius * kFrameHandleHitRadius;
	for(int pass = 0; pass < 2 && dragCorridor < 0; pass++){
		for(size_t i = 0; i < ids.size(); i++){
			if((pass == 0) != (static_cast<int>(i) == sel)) continue;
			const tracking::CorridorPlacement p = trackingConfig.placementFor(ids[i]);
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
			const tracking::CorridorPlacement p = trackingConfig.placementFor(ids[i]);
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
	const std::vector<std::string> ids = editCorridorIds();
	if(dragCorridor < 0 || dragCorridor >= static_cast<int>(ids.size()) || canvasRect.width <= 0) return;
	// placements may leave the canvas (that cuts the corridor) but stay
	// within the slider range
	const float cx = ofClamp((x - canvasRect.x) / canvasRect.width, -1.0f, 2.0f);
	const float cy = ofClamp((y - canvasRect.y) / canvasRect.height, -1.0f, 2.0f);
	tracking::CorridorPlacement & p = placementRef(ids[dragCorridor]);
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
	// the change reaches the worker through pushTrackingTuning() (no
	// rebuild — placements are applied when results are published)
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
	// corners stay inside the pane
	lane.quad.pts[dragCorner][0] = ofClamp((x - pane.x) / pane.width, 0, 1);
	lane.quad.pts[dragCorner][1] = ofClamp((y - pane.y) / pane.height, 0, 1);
}

void ofApp::mouseReleased(int, int, int){
	if(dragCorridor >= 0){
		dragCorridor = -1;
		dragPlaceHandle = -1;
		lastCorridorSelect = -1; // re-sync the placement sliders
		return;
	}
	if(dragLane < 0) return;
	if(dragLane < static_cast<int>(editLayout.lanes.size()) && editLayout.lanes[dragLane].quad.isConvex()){
		commitLayout();
	}else if(dragLane < static_cast<int>(editLayout.lanes.size())){
		ofLogWarning("ofApp") << "lane quad '" << editLayout.lanes[dragLane].id
			<< "' is crossed/collapsed - keeping the last valid lane active";
	}
	dragLane = -1;
	dragCorner = -1;
}

//--------------------------------------------------------------
// recording & playback

void ofApp::startRecording(){
	if(recorder.isRecording()) return;
	if(!recorder.start(recordingsDir)){
		recordParam = false;
		return;
	}
	// frames reach the recorder through the permanent raw-frame tap
}

void ofApp::stopRecording(){
	if(!recorder.isRecording()) return;
	recorder.stop();
}

void ofApp::enterPlayback(){
	if(mode == AppMode::Playback) return;
	if(recordParam) recordParam = false; // stops an active recording
	if(!player.scan(recordingsDir)){
		ofLogWarning("ofApp") << "no recordings to play in " << recordingsDir;
		playbackParam = false; // stay live (nested listener call is a no-op)
		return;
	}
	playbackSeenFrameId = 0;
	mode = AppMode::Playback;
	trackingManager.reset(); // fresh background model / tracks for the recording
}

void ofApp::exitPlayback(){
	if(mode == AppMode::Live) return;
	mode = AppMode::Live;
	player.close();
	playbackTexture.clear();
	trackingManager.reset();
}

//--------------------------------------------------------------
void ofApp::saveScreenshot(){
	const std::string filename = "screenshot_" + ofGetTimestampString("%Y-%m-%d_%H-%M-%S") + ".png";
	ofSaveScreen(filename);
	ofLogNotice("ofApp") << "screenshot saved to bin/data/" << filename;
}

//--------------------------------------------------------------
void ofApp::onPreviewChanged(bool &){
	settingsDirty = true;
	settingsDirtySinceMs = MobotixMjpegClient::nowMs();
}

void ofApp::onQualityChanged(int &){
	if(!previewParam) return; // quality only affects preview streams
	settingsDirty = true;
	settingsDirtySinceMs = MobotixMjpegClient::nowMs();
}

void ofApp::onRecordChanged(bool & value){
	if(value){
		if(recorder.isRecording()) return;
		if(mode == AppMode::Playback){
			ofLogWarning("ofApp") << "recording only works in live mode";
			recordParam = false;
			return;
		}
		startRecording();
	}else{
		stopRecording();
	}
}

void ofApp::onPlaybackChanged(bool & value){
	if(value){
		enterPlayback();
	}else{
		exitPlayback();
	}
}

void ofApp::onTrackingChanged(bool & value){
	trackingManager.setEnabled(value);
	if(value) trackingManager.reset(); // fresh background model on re-enable
}

void ofApp::onDetectorChanged(int & value){
	trackingManager.setDetectorType(value == 2 ? "yolo" : value == 1 ? "bgs" : "flow");
}

void ofApp::onHalfResChanged(bool & value){
	// takes effect on the next frame; no stream restart needed
	client.setDecodeScale(value ? 2 : 1);
	player.setDecodeScale(value ? 2 : 1);
}

void ofApp::onUdpChanged(bool & value){
	if(value){
		if(!udp.isOpen() && !udp.open(udpHost, udpPort)) udpParam = false;
	}else{
		udp.close();
	}
}

void ofApp::onReconnectPressed(){
	settingsDirty = false;
	applyStreamSettings();
}
