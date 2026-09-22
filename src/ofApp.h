#pragma once

#include <atomic>

#include "ofMain.h"
#include "ofxGui.h"

#include "mobotix/MobotixMjpegClient.h"
#include "mobotix/MobotixTypes.h"
#include "recording/MjpegPlayer.h"
#include "recording/MjpegRecorder.h"
#include "tracking/TrackingManager.h"

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
	void loadTrackingConfig(const ofJson & json);
	void applyStreamSettings();
	void drawVideo();
	void drawTrackingOverlay();
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
	// mode is atomic: the raw-frame taps read it on the stream / playback
	// threads to decide where frames go.
	std::atomic<AppMode> mode{AppMode::Live};
	std::string recordingsDir; // absolute
	MjpegRecorder recorder;
	MjpegPlayer player;
	ofTexture playbackTexture;
	uint64_t playbackSeenFrameId = 0;
	double playbackFrameTMs = 0;
	void startRecording();
	void stopRecording();
	void enterPlayback();
	void exitPlayback();

	// tracking (worker thread; fed by the raw-frame taps of client/player)
	tracking::TrackingManager trackingManager;
	tracking::TrackingConfig trackingConfig;
	tracking::TrackingResults trackResults;
	uint64_t trackResultsRevision = 0;
	int displayDecodeScale = 2;      // config display.decodeScale
	float overlayArrowScale = 1.0f;  // config tracking.overlay.arrowScale
	// screen rects of the module panes, rebuilt every drawVideo()
	std::vector<ofRectangle> paneRects;

	// gui
	ofxPanel gui;
	ofParameter<bool> previewParam{"preview (downscale)", false};
	ofParameter<int> qualityParam{"jpeg quality (preview)", 60, 10, 90};
	ofParameter<bool> showComDumpParam{"show COM dump", false};
	ofParameter<bool> recordParam{"record (SPACE)", false};
	ofParameter<bool> playbackParam{"playback mode (TAB)", false};
	ofParameter<bool> trackingParam{"tracking (t)", true};
	ofParameter<bool> overlayParam{"overlay (o)", true};
	ofParameter<bool> yoloParam{"detector: YOLO", false};
	ofParameter<bool> halfResParam{"half-res decode", true};
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
	void onTrackingChanged(bool & value);
	void onYoloChanged(bool & value);
	void onHalfResChanged(bool & value);
	void onReconnectPressed();
};
