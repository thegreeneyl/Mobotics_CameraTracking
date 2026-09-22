#pragma once

#include "ofMain.h"
#include "ofxGui.h"

#include "mobotix/MobotixMjpegClient.h"
#include "mobotix/MobotixTypes.h"
#include "recording/MjpegPlayer.h"
#include "recording/MjpegRecorder.h"

class ofApp : public ofBaseApp {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void exit() override;
	void keyPressed(int key) override;

private:
	enum class AppMode { Live, Playback };

	void loadConfig();
	void applyStreamSettings();
	void drawVideo();
	void drawRecordingIndicator();
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

	// recording & playback
	AppMode mode = AppMode::Live;
	std::string recordingsDir; // absolute
	MjpegRecorder recorder;
	MjpegPlayer player;
	ofTexture playbackTexture;
	void startRecording();
	void stopRecording();
	void enterPlayback();
	void exitPlayback();

	// gui
	ofxPanel gui;
	ofParameter<bool> previewParam{"preview (downscale)", false};
	ofParameter<int> qualityParam{"jpeg quality (preview)", 60, 10, 90};
	ofParameter<bool> showComDumpParam{"show COM dump", false};
	ofParameter<bool> recordParam{"record (SPACE)", false};
	ofParameter<bool> playbackParam{"playback mode (TAB)", false};
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
	void onRecordChanged(bool & value);
	void onPlaybackChanged(bool & value);
	void onReconnectPressed();
};
