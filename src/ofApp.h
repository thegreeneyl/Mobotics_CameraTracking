#pragma once

#include <atomic>

#include "ofMain.h"
#include "ofxGui.h"

#include "mobotix/MobotixMjpegClient.h"
#include "mobotix/MobotixTypes.h"
#include "output/UdpSender.h"
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
	void mousePressed(int x, int y, int button) override;
	void mouseDragged(int x, int y, int button) override;
	void mouseReleased(int x, int y, int button) override;

private:
	enum class AppMode { Live, Playback };

	void loadConfig();
	void applyStreamSettings();
	void drawVideo();
	void drawTrackingOverlay();
	void drawLaneEditor();
	void drawTrackingResults();
	void drawCanvasView();
	void drawRecordingIndicator();
	void drawSensorPanel();
	void drawStatsBar();
	void drawComDump();

	// camera
	CameraConfig camConfig;
	MobotixMjpegClient client;

	// latest frame state
	ofTexture texture;
	// Latest decoded frame before lens correction, so a slider drag rewarps
	// the picture without waiting for the next JPEG.
	ofPixels rawFrame;
	AppMode rawFrameMode = AppMode::Live;
	LensCorrector displayLens;
	LensCorrection shownLens;
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
	std::string currentSourceName() const;

	// tracking (worker thread; fed by the raw-frame taps of client/player)
	tracking::TrackingManager trackingManager;
	tracking::TrackingConfig trackingConfig; // loaded config + committed lane edits
	tracking::TrackingResults trackResults;
	uint64_t trackResultsRevision = 0;
	int displayDecodeScale = 2;      // config display.decodeScale
	float overlayArrowScale = 1.0f;  // load buffer for tracking.overlay.arrowScale
	std::string lastSourceName;
	// screen rects of the module panes, rebuilt every drawVideo()
	std::vector<ofRectangle> paneRects;
	// one result view per lane (index = lane index in the edited layout)
	std::vector<ofRectangle> laneResultRects;
	std::vector<ofTexture> laneMaskTex;

	// lane editing. editLayout is what the mouse edits and what is drawn;
	// committedLayout is the last valid state handed to the worker (updated
	// on mouse release, only when the quad is convex) and written back into
	// trackingConfig.layouts.
	tracking::LayoutConfig editLayout;
	tracking::LayoutConfig committedLayout;
	int editPaneCount = 0;
	int dragLane = -1;
	int dragCorner = -1;
	int lastLaneSelect = -1;
	bool laneSliderSync = false;
	void syncEditLayout(int paneCount);
	void commitLayout();

	// corridor placement editing on the output canvas (result view 0).
	// Placements live directly in trackingConfig.corridors; a corridor the
	// layout uses without a config entry gets one on first edit.
	ofRectangle canvasRect;     // screen rect of the canvas itself
	ofRectangle canvasViewRect; // the surrounding area (off-canvas parts show here)
	std::vector<std::string> editCorridorIds() const;
	tracking::CorridorPlacement & placementRef(const std::string & id);
	int dragCorridor = -1;    // index into editCorridorIds()
	int dragPlaceHandle = -1; // 0..3 corner (TL,TR,BR,BL), 4 = body
	float dragGrabX = 0, dragGrabY = 0; // body drag: grab point offset (canvas units)
	int lastCorridorSelect = -1;
	bool mousePressCanvas(int x, int y);
	void mouseDragCanvas(int x, int y);

	// UDP publishing (output.udp in config.json)
	output::UdpSender udp;
	bool udpEnabled = false;
	std::string udpHost = "127.0.0.1";
	int udpPort = 8765;
	int udpFrameIndex = 0;

	// gui
	ofxPanel gui;
	ofParameter<bool> previewParam{"preview (downscale)", false};
	ofParameter<int> qualityParam{"jpeg quality (preview)", 60, 10, 90};
	ofParameter<bool> showComDumpParam{"show COM dump", false};
	ofParameter<bool> recordParam{"record (SPACE)", false};
	ofParameter<bool> playbackParam{"playback mode (TAB)", false};
	ofParameter<bool> trackingParam{"tracking (t)", true};
	ofParameter<bool> overlayParam{"overlay (o)", true};
	ofParameter<bool> showMaskParam{"motion mask (m)", true};
	ofParameter<bool> showDetectionsParam{"raw detections (d)", true};
	// the three result layers per tracked object, each switchable
	ofParameter<bool> showPixelsParam{"shape pixels (x)", false};
	ofParameter<bool> showOutlineParam{"shape outline (n)", true};
	ofParameter<bool> showBoxParam{"bounding box (b)", true};
	ofParameter<int> resultViewParam{"result 0canvas 1lanes (v)", 0, 0, 1};
	ofParameter<int> detectorParam{"detector 0flow 1bgs 2yolo", 0, 0, 2};
	ofParameter<bool> halfResParam{"half-res decode", true};
	ofParameter<bool> udpParam{"udp publish", false};
	ofxButton reconnectButton;

	// tracking tuning gui (values loaded from config.json, pushed live to
	// the worker, written back by the save button — never by gui.xml)
	ofxPanel trackingGui;
	ofParameter<bool> lensEnabledParam{"lens correct", true};
	ofParameter<float> lensFovParam{"lens fov deg", 170, 70, 175};
	ofParameter<float> lensK1Param{"lens k1", 0, -0.8f, 0.8f};
	ofParameter<float> lensK2Param{"lens k2", 0, -0.5f, 0.5f};
	ofParameter<float> lensBalanceParam{"lens balance 0fill 1full", 0, 0, 1};
	ofParameter<float> lensCenterXParam{"lens center x", 0.5f, 0.3f, 0.7f};
	ofParameter<float> lensCenterYParam{"lens center y", 0.5f, 0.3f, 0.7f};
	ofParameter<int> laneSelectParam{"lane (edit) [ ]", 0, 0, 7};
	ofParameter<float> laneS0Param{"lane s0", 0, 0, 1};
	ofParameter<float> laneS1Param{"lane s1", 1, 0, 1};
	ofParameter<int> corridorSelectParam{"corridor (place) , .", 0, 0, 7};
	ofParameter<float> placeX0Param{"place x0", 0, -1, 2};
	ofParameter<float> placeX1Param{"place x1", 1, -1, 2};
	ofParameter<float> placeY0Param{"place y0", 0, -1, 2};
	ofParameter<float> placeY1Param{"place y1", 1, -1, 2};
	ofParameter<float> flowMinFlowParam{"flow minFlowPx", 0.6f, 0.1f, 4};
	ofParameter<float> flowEmaParam{"flow ema", 0.5f, 0.05f, 1};
	ofParameter<float> flowBusyThreshParam{"flow busyThresh", 0.35f, 0.05f, 1};
	ofParameter<bool> flowUseMogParam{"flow useMog2", true};
	ofParameter<float> flowMogVarParam{"flow mogVarThreshold", 24, 4, 100};
	ofParameter<float> flowCloseWParam{"flow closeWFrac", 0.04f, 0, 0.3f};
	ofParameter<float> flowCloseHParam{"flow closeHFrac", 0.3f, 0, 1};
	ofParameter<float> flowMinAreaParam{"flow minAreaFrac", 0.003f, 0, 0.05f};
	ofParameter<float> flowMinCoherenceParam{"flow minCoherence", 0.45f, 0, 1};
	ofParameter<float> flowMinSpeedParam{"flow minSpeed", 0.01f, 0, 0.1f};
	ofParameter<float> flowMergeGapParam{"flow mergeGapFrac", 0.15f, 0, 0.5f};
	ofParameter<int> trkConfirmParam{"trk confirmFrames", 4, 1, 30};
	ofParameter<int> trkMaxMissesParam{"trk maxMisses", 20, 1, 90};
	ofParameter<float> trkGateGapParam{"trk gateGapFrac", 0.1f, 0.01f, 0.5f};
	ofParameter<float> trkAbsorbGapParam{"trk absorbGapFrac", 0.15f, 0, 0.5f};
	ofParameter<int> trkMergeFramesParam{"trk mergeFrames", 6, 1, 30};
	ofParameter<float> trkReacquireMsParam{"trk reacquireMs", 2000, 0, 20000};
	ofParameter<float> trkVelMeasNoiseParam{"trk velMeasNoise", 6e-4f, 1e-5f, 1e-2f};
	ofParameter<int> trkTrailParam{"trk trailFrames", 30, 0, 120};
	ofParameter<int> shapeSourceParam{"shape 0mask 1fg 2model", 2, 0, 2};
	ofParameter<float> shapeEmaParam{"shape ema", 0.25f, 0.02f, 1};
	ofParameter<float> shapeThreshParam{"shape threshold", 0.5f, 0.1f, 0.9f};
	ofParameter<float> shapePadXParam{"shape padX", 0.04f, 0, 0.3f};
	ofParameter<float> shapePadYParam{"shape padY", 0.15f, 0, 0.6f};
	ofParameter<float> shapeSimplifyParam{"shape simplify cells", 0.5f, 0, 4};
	ofParameter<float> yoloConfParam{"yolo confThreshold", 0.35f, 0.05f, 0.95f};
	ofParameter<float> yoloNmsParam{"yolo nmsThreshold", 0.45f, 0.05f, 0.95f};
	ofParameter<float> arrowScaleParam{"overlay arrowScale", 1, 0, 5};
	ofxButton saveTrackingButton;
	// live push of tuning changes (throttled in update())
	tracking::TrackingConfig currentTrackingTuning() const;
	void loadGuiFromConfig();
	void pushTrackingTuning();
	void onSaveTrackingPressed();
	tracking::TrackingConfig pushedTuning;
	double lastTuningPushMs = 0;
	std::string configPath; // the config file that was loaded (save target)

	// debounced restart when stream params change
	bool settingsDirty = false;
	double settingsDirtySinceMs = 0;

	// diagnostics
	void saveScreenshot();
	double lastStatsLogMs = 0;
	std::string autoshotPath; // CAMTRACK_AUTOSHOT env: save screenshot + quit
	float autoshotSec = 8.0f; // CAMTRACK_AUTOSHOT_SEC
	bool autoshotDone = false;

	void onPreviewChanged(bool & value);
	void onQualityChanged(int & value);
	void onRecordChanged(bool & value);
	void onPlaybackChanged(bool & value);
	void onTrackingChanged(bool & value);
	void onDetectorChanged(int & value);
	void onHalfResChanged(bool & value);
	void onUdpChanged(bool & value);
	void onReconnectPressed();
};
