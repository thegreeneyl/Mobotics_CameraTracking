#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ofJson.h"
#include "ofPixels.h"

#include "../mobotix/MobotixMjpegClient.h"
#include "../mobotix/MobotixTypes.h"
#include "../output/MotionPathWriter.h"
#include "../recording/MjpegPlayer.h"
#include "../recording/MjpegRecorder.h"
#include "../tracking/TrackingManager.h"
#include "../util/LensCorrector.h"

// Everything the camera tracking does, without a window: the Mobotix
// stream, recording and playback of raw MJPEG, the tracking worker, the
// loaded config and its save-back, the motion-path export and the newest
// published frame as JSON. The host (today ofApp, later the Steuerung
// ofApp) owns one instance, calls update() once per frame on the main
// thread and draws whatever it takes from takePreview() / takeResults().
//
// No drawing, no textures, no ofxGui — nothing in here needs OpenGL.
//
// Threads: the stream and playback threads call the raw-frame taps; the
// tracking worker publishes results. All public methods are main-thread.
class CameraTrackingSession {
public:
	enum class Mode { Live, Playback };

	~CameraTrackingSession();

	// Reads the config document (path relative to the data folder). When
	// the file is missing, config.example.json beside it is read instead;
	// saveConfig() still writes to `path`. Returns false when neither
	// document could be read (defaults stay in place).
	bool load(const std::string & path = "config/config.json");
	// Writes the whole document from memory: cameras, recordingsDir,
	// display, tracking (including the edited anchors and placements).
	bool saveConfig() const;

	void start(); // tracking worker, raw-frame taps, camera stream
	void update(); // main thread, once per frame
	void stop();

	// --- source -----------------------------------------------------------
	// Entering playback scans recordingsDir and starts on the newest
	// recording; false (and still live) when there is nothing to play. An
	// active recording is stopped first. Tracking restarts fresh.
	bool setMode(Mode m);
	Mode getMode() const { return mode; }
	// The live stream keeps running in the background during playback.
	void reconnect();
	// "live" or the current recording's name — the layout key and the
	// source label of every published frame.
	std::string sourceName() const;

	// --- playback transport (no-ops while live) -----------------------------
	void play();
	void pause();
	bool isPaused() const { return player.isPaused(); }
	void seekTo(double positionSec);
	void seekBy(double deltaSec);
	void stepFrame(int delta);
	void nextRecording();
	void previousRecording();
	double positionSec() const { return player.getPositionMs() / 1000.0; }
	double durationSec() const { return player.getDurationMs() / 1000.0; }
	std::string recordingName() const { return player.getCurrentName(); }
	size_t recordingIndex() const { return player.getCurrentIndex(); }
	size_t recordingCount() const { return player.getCount(); }
	size_t frameCursor() const { return player.getFrameCursor(); }
	size_t frameCount() const { return player.getFrameCount(); }
	double playbackDecodeMs() const { return player.getLastDecodeMs(); }

	// --- recording (raw JPEG bytes, live only) ------------------------------
	bool startRecording();
	void stopRecording();
	bool isRecording() const { return recorder.isRecording(); }
	const MjpegRecorder & getRecorder() const { return recorder; }

	// --- motion-path export (editor file, both modes) -----------------------
	// One file per start under recordingsDir. A source change (live <->
	// playback, next recording) closes the file and opens the next one.
	void startMotionExport();
	void stopMotionExport(); // writes the file (dropped when empty)
	bool isMotionExporting() const { return motionExport != nullptr; }
	std::string motionExportFile() const { return motionExportPath; }
	size_t motionExportFrames() const { return motionExport ? motionExport->frameCount() : 0; }

	// --- tracking -----------------------------------------------------------
	void setTrackingEnabled(bool enabled);
	bool isTrackingEnabled() const { return config.enabled; }
	const tracking::TrackingConfig & getConfig() const { return config; }

	// The layout the worker runs for the current source and pane count.
	// layoutRevision() changes whenever that resolution changes (new
	// source, other pane count), so an editor can re-seed its copy.
	tracking::LayoutConfig currentLayout() const;
	uint64_t layoutRevision() const { return layoutRev; }
	int paneCount() const { return panes; }
	// Commits an edited layout (call on mouse-up; the worker rebuilds its
	// lanes). A synthesized fallback layout is given a real name and added
	// to the config so it can be saved; the stored layout is returned.
	tracking::LayoutConfig setLayout(const tracking::LayoutConfig & layout);
	// Corridors that can be placed: the configured ones plus any the
	// current layout references without a config entry.
	std::vector<std::string> corridorIds() const;
	// Moves a corridor on the canvas (fine to call while dragging — applied
	// at publish time, no rebuild). Unknown ids get a config entry.
	void setPlacement(const std::string & corridorId, const tracking::CorridorPlacement & placement);

	// --- outputs ------------------------------------------------------------
	// Newest lens-corrected display frame, copied when its revision is newer
	// than lastSeenRevision. Unallocated pixels mean "nothing to show"
	// (right after a mode switch). geometry is the pane geometry of that
	// frame (the filled frame is the whole pane).
	bool takePreview(ofPixels & pixels, LensGeometry & geometry, uint64_t & lastSeenRevision) const;
	// Newest full tracking snapshot (lanes, detections, corridors, masks).
	bool takeResults(tracking::TrackingResults & results, uint64_t & lastSeenRevision) const;
	// The last analysed frame as a motion-path.v1 object (output::buildFrame):
	// empty until the first result arrives. resultRevision() counts the
	// published frames, so a consumer polls it and reads on change.
	const ofJson & resultJson() const { return lastResultJson; }
	uint64_t resultRevision() const { return resultRev; }

	// --- info ---------------------------------------------------------------
	const CameraConfig & getCamera() const { return camera; }
	StreamStats streamStats() const { return client.getStats(); }
	std::string streamUrl() const { return client.getStreamUrl(); }
	const SensorSnapshot & getSensors() const { return sensors; }
	// ms since the newest live frame finished decoding (-1 before the first)
	double liveFrameAgeMs() const;
	// receive-complete -> preview ready, newest live frame
	double liveFrameLatencyMs() const { return lastFrameLatencyMs; }
	int displayDecodeScale() const { return decodeScale; }
	float arrowScale() const { return displayArrowScale; }
	const std::string & getRecordingsDir() const { return recordingsDir; }
	// Runtime override (CAMTRACK_RECORDINGS); the document keeps its value.
	void setRecordingsDir(const std::string & absolutePath){ recordingsDir = absolutePath; }

private:
	void applyConfigToWorker();
	void publishResults(const tracking::TrackingResults & results);
	// Camera fps when configured, else the recording's frame spacing in
	// playback or the measured stream rate live; 30 as the last resort.
	double nominalFps() const;
	void refreshLayoutKey();

	// config document
	std::string configPath;              // save target (data-relative)
	std::vector<CameraConfig> cameras;   // every entry, written back as-is
	CameraConfig camera;                 // the first enabled one
	tracking::TrackingConfig config;     // tracking section + runtime edits
	std::string recordingsDirSetting;    // as in the document (data-relative)
	std::string recordingsDir;           // absolute, may be overridden at runtime
	int decodeScale = 2;
	float displayArrowScale = 1.0f;

	// sources
	MobotixMjpegClient client;
	MjpegPlayer player;
	MjpegRecorder recorder;
	// read on the stream / playback threads by the raw-frame taps
	std::atomic<Mode> mode{Mode::Live};
	uint64_t liveSeenFrameId = 0;
	uint64_t playbackSeenFrameId = 0;
	double lastFrameReceivedMs = 0;
	double lastFrameLatencyMs = 0;
	SensorSnapshot sensors;

	// preview (lens-corrected, main thread only)
	LensCorrector displayLens;
	ofPixels preview;
	LensGeometry previewGeometry;
	uint64_t previewRev = 0;
	int panes = 1;
	int layoutPanes = -1;
	std::string layoutSource;
	uint64_t layoutRev = 1;

	// tracking
	tracking::TrackingManager trackingManager;
	tracking::TrackingResults results;
	uint64_t resultsRev = 0;
	ofJson lastResultJson;
	uint64_t resultRev = 0;
	int publishedFrames = 0;

	// motion export
	std::unique_ptr<output::MotionPathRecording> motionExport;
	std::string motionExportPath;
	std::string motionExportSource;

	bool started = false;
};
