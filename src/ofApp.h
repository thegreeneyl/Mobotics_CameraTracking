#pragma once

#include "ofMain.h"
#include "ofxGui.h"

#include "mobotix/MobotixMjpegClient.h"
#include "mobotix/MobotixTypes.h"

class ofApp : public ofBaseApp {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void exit() override;
	void keyPressed(int key) override;

private:
	void loadConfig();
	void applyStreamSettings();
	void drawVideo();
	void drawSensorPanel();
	void drawStatsBar();
	void drawComDump();

	// camera
	CameraConfig camConfig;
	MobotixMjpegClient client;

	// latest frame state
	ofTexture texture;
	SensorSnapshot sensors;
	std::string comDump;
	uint64_t lastSeenFrameId = 0;
	double lastFrameReceivedMs = 0;   // client steady-clock ms
	double lastUploadLatencyMs = 0;   // receive-complete -> texture upload

	// gui
	ofxPanel gui;
	ofParameter<bool> previewParam{"preview (downscale)", false};
	ofParameter<int> qualityParam{"jpeg quality (preview)", 60, 10, 90};
	ofParameter<bool> showComDumpParam{"show COM dump", false};
	ofxButton reconnectButton;

	// debounced restart when stream params change
	bool settingsDirty = false;
	double settingsDirtySinceMs = 0;

	// diagnostics
	void saveScreenshot();
	double lastStatsLogMs = 0;
	std::string autoshotPath; // CAMTRACK_AUTOSHOT env: save screenshot + quit
	bool autoshotDone = false;

	void onPreviewChanged(bool & value);
	void onQualityChanged(int & value);
	void onReconnectPressed();
};
