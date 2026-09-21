#include "ofApp.h"

namespace {
constexpr double kSettingsDebounceMs = 600;

std::string formatLux(float lux){
	return lux < 0 ? "n/a" : ofToString(lux, 1) + " lx";
}
std::string formatTemp(float temp){
	return temp < 0 ? "n/a" : ofToString(temp, 1) + " C";
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
	reconnectButton.setup("reconnect (r)");
	gui.add(&reconnectButton);
	gui.setPosition(10, 10);

	previewParam = camConfig.preview;
	qualityParam = camConfig.quality;
	previewParam.addListener(this, &ofApp::onPreviewChanged);
	qualityParam.addListener(this, &ofApp::onQualityChanged);
	reconnectButton.addListener(this, &ofApp::onReconnectPressed);

	if(camConfig.enabled && !camConfig.host.empty()){
		ofLogNotice("ofApp") << "starting stream: " << camConfig.buildStreamUrl();
		client.start(camConfig);
	}else{
		ofLogError("ofApp") << "no enabled camera in config/config.json";
	}

	// Headless verification: CAMTRACK_AUTOSHOT=<path> saves a screenshot
	// after ~8 s of streaming and quits.
	if(const char * autoshot = std::getenv("CAMTRACK_AUTOSHOT")){
		autoshotPath = autoshot;
		ofLogNotice("ofApp") << "autoshot enabled -> " << autoshotPath;
	}
}

//--------------------------------------------------------------
void ofApp::loadConfig(){
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
void ofApp::applyStreamSettings(){
	camConfig.preview = previewParam;
	camConfig.quality = qualityParam;
	client.restart(camConfig);
}

//--------------------------------------------------------------
void ofApp::update(){
	ofPixels pixels;
	double receivedAtMs = 0;
	if(client.getLatestFrame(pixels, sensors, lastSeenFrameId, receivedAtMs,
	                         showComDumpParam ? &comDump : nullptr)){
		texture.loadData(pixels);
		lastFrameReceivedMs = receivedAtMs;
		lastUploadLatencyMs = MobotixMjpegClient::nowMs() - receivedAtMs;
	}

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

	if(!texture.isAllocated()){
		ofSetColor(160);
		ofDrawBitmapString("waiting for stream: " + client.getStreamUrl(),
		                   viewport.getCenter().x - 240, viewport.getCenter().y);
		ofSetColor(255);
		return;
	}

	ofSetColor(255);
	const bool splitBoth = (sensors.cam == "BOTH") && texture.getWidth() >= texture.getHeight() * 2;

	if(splitBoth){
		// one BOTH frame = M1 | M2 side by side; draw as two panes with a gap
		const float halfW = texture.getWidth() / 2.0f;
		const float texH = texture.getHeight();
		const float gap = 4;
		const float paneW = (viewport.width - gap) / 2.0f;
		const float scale = std::min(paneW / halfW, viewport.height / texH);
		const float drawW = halfW * scale;
		const float drawH = texH * scale;
		const float y = viewport.y + (viewport.height - drawH) / 2.0f;
		const float x0 = viewport.x + (paneW - drawW) / 2.0f;
		const float x1 = viewport.x + paneW + gap + (paneW - drawW) / 2.0f;
		texture.drawSubsection(x0, y, drawW, drawH, 0, 0, halfW, texH);
		texture.drawSubsection(x1, y, drawW, drawH, halfW, 0, halfW, texH);
		ofSetColor(200);
		ofDrawBitmapString("M1 (left)", x0 + 6, y + 16);
		ofDrawBitmapString("M2 (right)", x1 + 6, y + 16);
		ofSetColor(255);
	}else{
		const float scale = std::min(viewport.width / texture.getWidth(),
		                             viewport.height / texture.getHeight());
		const float drawW = texture.getWidth() * scale;
		const float drawH = texture.getHeight() * scale;
		texture.draw(viewport.x + (viewport.width - drawW) / 2.0f,
		             viewport.y + (viewport.height - drawH) / 2.0f, drawW, drawH);
	}
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
		+ "   decode->texture " + ofToString(lastUploadLatencyMs, 1) + " ms"
		+ "   frames " + ofToString(stats.framesDecoded)
		+ " dropped " + ofToString(stats.framesDropped)
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
	reconnectButton.removeListener(this, &ofApp::onReconnectPressed);
	client.stop();
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
	}
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

void ofApp::onReconnectPressed(){
	settingsDirty = false;
	applyStreamSettings();
}
