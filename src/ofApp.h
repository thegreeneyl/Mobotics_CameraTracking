#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "ofMain.h"
#include "ofxGui.h"

#include "session/CameraTrackingSession.h"

// The stand-alone window around CameraTrackingSession: draws the preview,
// the lane editor, the tracking overlay and the result views, and offers
// the few operator controls that are not config-only — anchor and
// placement editing, save, reconnect, view toggles, live/playback with a
// transport, record and motion export.
//
// This is a temporary shell: the Steuerung build replaces it with a view
// in its own UI system (see PORT_TO_STEUERUNG.md). Only this file and
// ofApp.cpp use ofxGui.
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
	void drawVideo();
	void drawTrackingOverlay();
	void drawLaneEditor();
	void drawTrackingResults();
	void drawCanvasView();
	void drawRecordingIndicator();
	void drawStatsBar();

	CameraTrackingSession session;

	// preview (lens-corrected frame from the session) and newest results
	ofTexture texture;
	LensGeometry previewGeometry;
	uint64_t previewRev = 0;
	tracking::TrackingResults trackResults;
	uint64_t trackResultsRev = 0;
	// screen rects of the module panes, rebuilt every drawVideo()
	std::vector<ofRectangle> paneRects;
	// one result view per lane (index = lane index in the edited layout)
	std::vector<ofRectangle> laneResultRects;
	std::vector<ofTexture> laneMaskTex;

	// lane editing. editLayout is what the mouse edits and what is drawn;
	// it is re-seeded from the session when the resolved layout changes
	// and committed back (session.setLayout) on mouse release, only when
	// every strip slice is convex.
	tracking::LayoutConfig editLayout;
	uint64_t seenLayoutRev = 0;
	int dragLane = -1;
	int dragEdge = -1;     // 0 = top border, 1 = bottom border
	int dragStation = -1;  // index along that border
	int anchorLane = -1;   // last grabbed station, so delete can remove it
	int anchorEdge = -1;
	int anchorStation = -1;
	int lastLaneSelect = -1;
	void syncEditLayout();
	void commitLayout();

	// corridor placement editing on the output canvas (result view 0).
	ofRectangle canvasRect;     // screen rect of the canvas itself
	ofRectangle canvasViewRect; // the surrounding area (off-canvas parts show here)
	int dragCorridor = -1;    // index into session.corridorIds()
	int dragPlaceHandle = -1; // 0..3 corner (TL,TR,BR,BL), 4 = body
	float dragGrabX = 0, dragGrabY = 0; // body drag: grab point offset (canvas units)
	int lastCorridorSelect = -1;
	bool mousePressCanvas(int x, int y);
	void mouseDragCanvas(int x, int y);

	// main panel: source, record/export, view toggles
	ofxPanel gui;
	ofParameter<bool> playbackParam{"playback mode (TAB)", false};
	ofParameter<bool> recordParam{"record (SPACE)", false};
	ofParameter<bool> exportParam{"export motion (e)", false};
	ofParameter<bool> trackingParam{"tracking (t)", true};
	ofParameter<bool> overlayParam{"overlay (o)", true};
	// View zoom of the corrected picture. 1 fits the dim border in the
	// video area; higher crops that border and makes the bright frame larger.
	ofParameter<float> imageScaleParam{"image scale", 1.0f, 1.0f, 3.0f};
	ofParameter<bool> showMaskParam{"motion mask (m)", true};
	ofParameter<bool> showDetectionsParam{"raw detections (d)", true};
	// the three result layers per tracked object, each switchable
	ofParameter<bool> showPixelsParam{"shape pixels (x)", false};
	ofParameter<bool> showOutlineParam{"shape outline (n)", true};
	ofParameter<bool> showBoxParam{"bounding box (b)", true};
	// Result (default): the published box and center. For a boat that is
	// the outline's bounds. Raw: the flow box the tracker measures with.
	ofParameter<bool> showResultParam{"result data (q)", true};
	ofParameter<int> resultViewParam{"result 0canvas 1lanes (v)", 0, 0, 1};
	ofxButton reconnectButton;

	// playback transport (shown in playback mode only, under the main
	// panel; the edit panel moves down to make room)
	ofxPanel playbackGui;
	ofParameter<bool> playParam{"play (SPACE)", true};
	ofParameter<float> playheadParam{"playhead s", 0, 0, 1};
	ofxButton prevFrameButton;
	ofxButton nextFrameButton;
	ofxButton prevRecordingButton;
	ofxButton nextRecordingButton;
	bool playheadSync = false;      // true while update() mirrors the player into the slider
	bool playbackGuiActive = false; // panel shown and receiving mouse events
	void layoutGuiPanels();
	void onPlayChanged(bool & value);
	void onPlayheadChanged(float & value);
	void onPrevFramePressed();
	void onNextFramePressed();
	void onPrevRecordingPressed();
	void onNextRecordingPressed();

	// edit panel: lane anchors and corridor placement, saved to config.json
	ofxPanel editGui;
	ofParameter<int> laneSelectParam{"lane (edit) [ ]", 0, 0, 7};
	ofParameter<float> laneS0Param{"lane s0", 0, 0, 1};
	ofParameter<float> laneS1Param{"lane s1", 1, 0, 1};
	ofParameter<int> corridorSelectParam{"corridor (place) , .", 0, 0, 7};
	ofParameter<float> placeX0Param{"place x0", 0, -1, 2};
	ofParameter<float> placeX1Param{"place x1", 1, -1, 2};
	ofParameter<float> placeY0Param{"place y0", 0, -1, 2};
	ofParameter<float> placeY1Param{"place y1", 1, -1, 2};
	ofxButton saveButton;

	double lastStatsLogMs = 0;

	void onRecordChanged(bool & value);
	void onExportChanged(bool & value);
	void onPlaybackChanged(bool & value);
	void onTrackingChanged(bool & value);
	void onReconnectPressed();
	void onSavePressed();
};
