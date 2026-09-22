#include "ofApp.h"

namespace {
constexpr double kSettingsDebounceMs = 600;

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
	gui.add(yoloParam);
	gui.add(halfResParam);
	reconnectButton.setup("reconnect (r)");
	gui.add(&reconnectButton);
	gui.setPosition(10, 10);

	previewParam = camConfig.preview;
	qualityParam = camConfig.quality;
	recordParam = false;   // never restore a stale "recording" state from gui.xml
	playbackParam = false; // always start live
	trackingParam = trackingConfig.enabled;
	yoloParam = (trackingConfig.detector == "yolo");
	halfResParam = (displayDecodeScale >= 2);
	previewParam.addListener(this, &ofApp::onPreviewChanged);
	qualityParam.addListener(this, &ofApp::onQualityChanged);
	recordParam.addListener(this, &ofApp::onRecordChanged);
	playbackParam.addListener(this, &ofApp::onPlaybackChanged);
	trackingParam.addListener(this, &ofApp::onTrackingChanged);
	yoloParam.addListener(this, &ofApp::onYoloChanged);
	halfResParam.addListener(this, &ofApp::onHalfResChanged);
	reconnectButton.addListener(this, &ofApp::onReconnectPressed);

	// tracking worker (fed below by the raw-frame taps)
	trackingConfig.enabled = trackingParam;
	trackingManager.setup(trackingConfig);

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
	// after ~8 s of streaming and quits. CAMTRACK_AUTOPLAYBACK=1 additionally
	// starts in playback mode (tracking against a recording, no camera).
	if(const char * autoshot = std::getenv("CAMTRACK_AUTOSHOT")){
		autoshotPath = autoshot;
		ofLogNotice("ofApp") << "autoshot enabled -> " << autoshotPath;
	}
	if(std::getenv("CAMTRACK_AUTOPLAYBACK")){
		ofLogNotice("ofApp") << "autoplayback enabled";
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
	ofJson json;
	try {
		json = ofLoadJson(path);
	} catch(const std::exception & e){
		ofLogError("ofApp") << "failed to load " << path << ": " << e.what();
		return;
	}
	recordingsDir = ofToDataPath(json.value("recordingsDir", std::string("recordings")), true);

	displayDecodeScale = 2;
	if(json.contains("display")){
		displayDecodeScale = json["display"].value("decodeScale", 2);
	}
	loadTrackingConfig(json);

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

//--------------------------------------------------------------
void ofApp::loadTrackingConfig(const ofJson & json){
	trackingConfig = tracking::TrackingConfig{};
	overlayArrowScale = 1.0f;
	if(!json.contains("tracking")) return;
	const auto & t = json["tracking"];

	trackingConfig.enabled = t.value("enabled", true);
	trackingConfig.detector = t.value("detector", std::string("bgs"));
	trackingConfig.analysisReduce = t.value("analysisReduce", 2);

	if(t.contains("bgs")){
		const auto & b = t["bgs"];
		trackingConfig.bgs.history = b.value("history", 500);
		trackingConfig.bgs.varThreshold = b.value("varThreshold", 16.0);
		trackingConfig.bgs.learningRate = b.value("learningRate", -1.0);
		trackingConfig.bgs.minAreaNorm = b.value("minAreaNorm", 0.0005f);
		trackingConfig.bgs.maxAreaNorm = b.value("maxAreaNorm", 0.5f);
		trackingConfig.bgs.morphOpenPx = b.value("morphOpen", 3);
		trackingConfig.bgs.morphClosePx = b.value("morphClose", 9);
	}
	if(t.contains("tracker")){
		const auto & k = t["tracker"];
		trackingConfig.tracker.confirmFrames = k.value("confirmFrames", 5);
		trackingConfig.tracker.maxMisses = k.value("maxMisses", 12);
		trackingConfig.tracker.minIoU = k.value("minIoU", 0.1f);
		trackingConfig.tracker.maxDist = k.value("maxDist", 0.05f);
	}
	if(t.contains("yolo")){
		const auto & y = t["yolo"];
		trackingConfig.yolo.modelPath = ofToDataPath(y.value("model", std::string("models/yolov8n.onnx")), true);
		trackingConfig.yolo.inputSize = y.value("inputSize", 640);
		trackingConfig.yolo.confThreshold = y.value("confThreshold", 0.35f);
		trackingConfig.yolo.nmsThreshold = y.value("nmsThreshold", 0.45f);
		trackingConfig.yolo.classFilter.clear();
		if(y.contains("classes") && y["classes"].is_array()){
			for(const auto & c : y["classes"]){
				trackingConfig.yolo.classFilter.push_back(c.get<std::string>());
			}
		}
	}
	if(t.contains("overlay")){
		const auto & o = t["overlay"];
		overlayArrowScale = o.value("arrowScale", 1.0f);
		trackingConfig.tracker.trailLen = o.value("trailFrames", 30);
	}
	// zones: {"M1": [ {label,y0,y1,x0,x1}, ... ], "M2": [...]}
	trackingConfig.moduleZones.assign(2, {});
	if(t.contains("zones")){
		const auto parseZones = [](const ofJson & arr){
			std::vector<tracking::Zone> zones;
			if(!arr.is_array()) return zones;
			for(const auto & z : arr){
				tracking::Zone zone;
				zone.label = z.value("label", std::string(""));
				zone.y0 = z.value("y0", 0.0f);
				zone.y1 = z.value("y1", 1.0f);
				zone.x0 = z.value("x0", 0.0f);
				zone.x1 = z.value("x1", 1.0f);
				zones.push_back(zone);
			}
			return zones;
		};
		if(t["zones"].contains("M1")) trackingConfig.moduleZones[0] = parseZones(t["zones"]["M1"]);
		if(t["zones"].contains("M2")) trackingConfig.moduleZones[1] = parseZones(t["zones"]["M2"]);
	}
}

//--------------------------------------------------------------
void ofApp::applyStreamSettings(){
	camConfig.preview = previewParam;
	camConfig.quality = qualityParam;
	client.restart(camConfig);
}

//--------------------------------------------------------------
void ofApp::update(){
	if(mode == AppMode::Playback){
		// live stream keeps running in the background but is ignored;
		// the player paces/reads/decodes on its own thread.
		ofPixels pixels;
		if(player.getLatestFrame(pixels, sensors, playbackSeenFrameId, playbackFrameTMs,
		                         showComDumpParam ? &comDump : nullptr)){
			playbackTexture.loadData(pixels);
		}
	}else{
		ofPixels pixels;
		double receivedAtMs = 0;
		if(client.getLatestFrame(pixels, sensors, lastSeenFrameId, receivedAtMs,
		                         showComDumpParam ? &comDump : nullptr)){
			texture.loadData(pixels);
			lastFrameReceivedMs = receivedAtMs;
			lastUploadLatencyMs = MobotixMjpegClient::nowMs() - receivedAtMs;
		}
	}

	// newest tracking results (published by the tracking thread)
	trackingManager.getLatestResults(trackResults, trackResultsRevision);

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

	if(!autoshotPath.empty() && !autoshotDone && ofGetElapsedTimef() > 8.0f){
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
	const ofRectangle viewport(0, 0, ofGetWidth() - panelWidth, ofGetHeight() - barHeight);

	const bool playback = (mode == AppMode::Playback);
	const ofTexture & tex = playback ? playbackTexture : texture;

	paneRects.clear();

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
	const bool splitBoth = (sensors.cam == "BOTH") && tex.getWidth() >= tex.getHeight() * 2;

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

	drawTrackingOverlay();
}

//--------------------------------------------------------------
// Debug overlay: per tracked object — bbox, center point, velocity arrow
// (1 s lookahead x arrowScale), "#id label" text and a fading trail, all in
// the object's stable per-id color. Tentative (unconfirmed) tracks are dim.
// Tracking coordinates are width-normalized per module, so mapping to the
// screen is just: screen = paneRect.xy + coord * paneRect.width.
void ofApp::drawTrackingOverlay(){
	if(!trackingParam || !overlayParam) return;
	if(trackResults.modules.empty() || paneRects.empty()) return;

	ofPushStyle();
	for(size_t m = 0; m < paneRects.size() && m < trackResults.modules.size(); m++){
		const ofRectangle & pane = paneRects[m];
		const float s = pane.width; // screen px per module-width unit

		for(const auto & obj : trackResults.modules[m].objects){
			ofColor col(obj.colorR * 255, obj.colorG * 255, obj.colorB * 255,
			            obj.confirmed ? 255 : 80);
			const float cx = pane.x + obj.x * s;
			const float cy = pane.y + obj.y * s;

			// trail (older = more transparent)
			if(obj.trail.size() > 1){
				for(size_t i = 1; i < obj.trail.size(); i++){
					const float a = static_cast<float>(i) / obj.trail.size();
					ofSetColor(col.r, col.g, col.b, col.a * a * 0.6f);
					ofDrawLine(pane.x + obj.trail[i - 1].first * s,
					           pane.y + obj.trail[i - 1].second * s,
					           pane.x + obj.trail[i].first * s,
					           pane.y + obj.trail[i].second * s);
				}
			}

			ofSetColor(col);
			// bounding box
			ofNoFill();
			ofDrawRectangle(cx - obj.w * s / 2, cy - obj.h * s / 2, obj.w * s, obj.h * s);
			// position point
			ofFill();
			ofDrawCircle(cx, cy, 3);
			// velocity arrow: where the object will be in 1 s (x arrowScale),
			// clamped to a quarter module width so fast objects stay readable
			float dx = obj.vx * overlayArrowScale;
			float dy = obj.vy * overlayArrowScale;
			const float len = std::hypot(dx, dy);
			constexpr float kMaxArrow = 0.25f;
			if(len > kMaxArrow){
				dx *= kMaxArrow / len;
				dy *= kMaxArrow / len;
			}
			ofDrawArrow(glm::vec3(cx, cy, 0), glm::vec3(cx + dx * s, cy + dy * s, 0), 5.0f);

			// label
			const float speed = std::hypot(obj.vx, obj.vy);
			std::string text = "#" + ofToString(obj.id);
			if(!obj.label.empty()) text += " " + obj.label;
			text += "  " + ofToString(speed, 3) + " w/s";
			if(!obj.confirmed) text += " ?";
			ofDrawBitmapStringHighlight(text, cx - obj.w * s / 2, cy - obj.h * s / 2 - 6,
			                            ofColor(0, 160), col);
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
		for(const auto & m : trackResults.modules) objCount += m.objects.size();
		trackInfo = "   trk[" + (trackResults.detectorName.empty() ? std::string("-") : trackResults.detectorName)
			+ "] " + ofToString(objCount) + " obj"
			+ " dec " + ofToString(trackResults.decodeMs, 1)
			+ " det " + ofToString(trackResults.detectMs, 1)
			+ " trk " + ofToString(trackResults.trackMs, 1) + " ms";
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
			+ "   LEFT/RIGHT prev/next   TAB back to live"
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
	ofSetColor(0, 210);
	ofDrawRectangle(10, 220, 420, ofGetHeight() - 300);
	ofSetColor(180, 255, 180);
	ofDrawBitmapString(comDump, 18, 240);
	ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::exit(){
	previewParam.removeListener(this, &ofApp::onPreviewChanged);
	qualityParam.removeListener(this, &ofApp::onQualityChanged);
	recordParam.removeListener(this, &ofApp::onRecordChanged);
	playbackParam.removeListener(this, &ofApp::onPlaybackChanged);
	trackingParam.removeListener(this, &ofApp::onTrackingChanged);
	yoloParam.removeListener(this, &ofApp::onYoloChanged);
	halfResParam.removeListener(this, &ofApp::onHalfResChanged);
	reconnectButton.removeListener(this, &ofApp::onReconnectPressed);
	stopRecording();
	client.stop();
	player.close();
	trackingManager.stop();
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
	}
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

void ofApp::onYoloChanged(bool & value){
	trackingManager.setDetectorType(value ? "yolo" : "bgs");
}

void ofApp::onHalfResChanged(bool & value){
	// takes effect on the next frame; no stream restart needed
	client.setDecodeScale(value ? 2 : 1);
	player.setDecodeScale(value ? 2 : 1);
}

void ofApp::onReconnectPressed(){
	settingsDirty = false;
	applyStreamSettings();
}
