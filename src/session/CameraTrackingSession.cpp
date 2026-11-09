#include "CameraTrackingSession.h"

#include <algorithm>
#include <chrono>

#include <opencv2/core.hpp>

#include "ofFileUtils.h"
#include "ofLog.h"
#include "ofUtils.h"

#include "../tracking/TrackingConfigJson.h"

namespace {
constexpr double kFallbackFps = 30.0;

double wallClockSec(){
	using namespace std::chrono;
	return duration<double>(system_clock::now().time_since_epoch()).count();
}

CameraConfig cameraFromJson(const ofJson & cam){
	CameraConfig c;
	c.id = cam.value("id", std::string("camera"));
	c.enabled = cam.value("enabled", false);
	c.host = cam.value("host", std::string());
	c.user = cam.value("user", std::string());
	c.password = cam.value("password", std::string());
	c.stream = cam.value("stream", std::string("full"));
	c.fps = cam.value("fps", 0.0f);
	c.preview = cam.value("preview", false);
	c.previewSize = cam.value("previewSize", std::string("1280x720"));
	c.quality = cam.value("quality", 60);
	return c;
}

ofJson cameraJson(const CameraConfig & c){
	return {
		{"id", c.id},
		{"enabled", c.enabled},
		{"host", c.host},
		{"user", c.user},
		{"password", c.password},
		{"stream", c.stream},
		{"fps", c.fps},
		{"preview", c.preview},
		{"previewSize", c.previewSize},
		{"quality", c.quality},
	};
}

// Lens-corrects src into dst (ofPixels in and out).
bool correctPixels(const ofPixels & src, LensCorrector & corr, ofPixels & dst){
	if(!src.isAllocated()) return false;
	const int w = static_cast<int>(src.getWidth());
	const int h = static_cast<int>(src.getHeight());
	const int ch = static_cast<int>(src.getNumChannels());
	const int type = ch == 4 ? CV_8UC4 : ch == 3 ? CV_8UC3 : CV_8UC1;
	const cv::Mat in(h, w, type, const_cast<unsigned char *>(src.getData()), src.getBytesStride());
	cv::Mat out;
	corr.apply(in, out);
	if(out.empty()) return false;
	const ofPixelFormat fmt = ch == 4 ? OF_PIXELS_RGBA : ch == 3 ? OF_PIXELS_RGB : OF_PIXELS_GRAY;
	dst.setFromPixels(out.data, static_cast<size_t>(out.cols), static_cast<size_t>(out.rows), fmt);
	return true;
}
} // namespace

CameraTrackingSession::~CameraTrackingSession(){
	stop();
}

//--------------------------------------------------------------
// config

bool CameraTrackingSession::load(const std::string & path){
	configPath = path;
	recordingsDir = ofToDataPath("recordings", true);

	std::string readPath = path;
	if(!ofFile::doesFileExist(readPath)){
		readPath = ofFilePath::join(ofFilePath::getEnclosingDirectory(path, false), "config.example.json");
		ofLogWarning("CameraTrackingSession") << path << " not found, reading " << readPath
			<< " (saves still go to " << path << ")";
	}
	ofJson json;
	try {
		json = ofLoadJson(readPath);
	} catch(const std::exception & e){
		ofLogError("CameraTrackingSession") << "failed to load " << readPath << ": " << e.what();
		return false;
	}
	if(!json.is_object()){
		ofLogError("CameraTrackingSession") << readPath << " is not a JSON object";
		return false;
	}

	const auto loaded = tracking::loadTrackingConfig(json, [](const std::string & p){ return ofToDataPath(p, true); });
	config = loaded.config;
	decodeScale = loaded.displayDecodeScale;
	displayArrowScale = loaded.displayArrowScale;
	recordingsDirSetting = loaded.recordingsDir;
	recordingsDir = ofToDataPath(recordingsDirSetting, true);
	displayLens.set(config.lens);

	cameras.clear();
	camera = CameraConfig{};
	if(json.contains("cameras") && json["cameras"].is_array()){
		for(const auto & cam : json["cameras"]) cameras.push_back(cameraFromJson(cam));
	}
	const auto first = std::find_if(cameras.begin(), cameras.end(), [](const CameraConfig & c){ return c.enabled; });
	if(first != cameras.end()){
		camera = *first;
	}else{
		ofLogError("CameraTrackingSession") << readPath << " has no enabled camera";
	}
	return true;
}

bool CameraTrackingSession::saveConfig() const {
	ofJson doc = ofJson::object();
	ofJson cams = ofJson::array();
	for(const auto & c : cameras) cams.push_back(cameraJson(c));
	doc["cameras"] = cams;
	doc["recordingsDir"] = recordingsDirSetting;
	doc["display"] = tracking::displayJson(decodeScale, displayArrowScale);
	doc["tracking"] = tracking::trackingJson(config);
	if(ofSavePrettyJson(configPath, doc)){
		ofLogNotice("CameraTrackingSession") << "config saved to " << configPath;
		return true;
	}
	ofLogError("CameraTrackingSession") << "failed to save config to " << configPath;
	return false;
}

//--------------------------------------------------------------
// lifecycle

void CameraTrackingSession::start(){
	if(started) return;
	started = true;

	config.sourceName = sourceName();
	layoutSource = config.sourceName;
	trackingManager.setup(config);

	// Raw-frame taps: the exact JPEG bytes of every complete frame, called
	// on the stream / playback thread. Recording and tracking both hang off
	// these; the main thread never touches raw frames.
	client.setDecodeScale(decodeScale);
	client.setRawFrameCallback([this](const uint8_t * data, size_t size, double tsMs){
		if(recorder.isRecording()) recorder.addFrame(data, size, tsMs);
		if(mode == Mode::Live) trackingManager.submitRawJpeg(data, size, tsMs);
	});
	player.setDecodeScale(decodeScale);
	player.setRawFrameCallback([this](const uint8_t * data, size_t size, double tMs){
		if(mode == Mode::Playback) trackingManager.submitRawJpeg(data, size, tMs);
	});

	if(camera.enabled && !camera.host.empty()){
		ofLogNotice("CameraTrackingSession") << "starting stream: " << camera.buildStreamUrl();
		client.start(camera);
	}else{
		ofLogError("CameraTrackingSession") << "no camera to stream from";
	}
}

void CameraTrackingSession::stop(){
	if(!started) return;
	started = false;
	stopRecording();
	stopMotionExport();
	client.setRawFrameCallback(nullptr);
	player.setRawFrameCallback(nullptr);
	client.stop();
	player.close();
	trackingManager.stop();
}

void CameraTrackingSession::update(){
	if(!started) return;

	// newest decoded frame of the active source -> corrected preview
	ofPixels pixels;
	bool gotFrame = false;
	if(mode == Mode::Playback){
		double frameTMs = 0;
		gotFrame = player.getLatestFrame(pixels, sensors, playbackSeenFrameId, frameTMs);
	}else{
		double receivedAtMs = 0;
		if(client.getLatestFrame(pixels, sensors, liveSeenFrameId, receivedAtMs)){
			gotFrame = true;
			lastFrameReceivedMs = receivedAtMs;
			lastFrameLatencyMs = MobotixMjpegClient::nowMs() - receivedAtMs;
		}
	}
	if(gotFrame && correctPixels(pixels, displayLens, preview)){
		previewGeometry = displayLens.geometry();
		previewRev++;
		// pane count: the corrector knows it when it ran; a pass-through
		// frame is two modules when it is at least twice as wide as tall
		const LensGeometry & g = previewGeometry;
		const bool geo = g.outPaneW > 0 && g.outPaneH > 0 && g.panes > 0
			&& static_cast<int>(preview.getWidth()) == g.panes * g.outPaneW;
		panes = geo ? g.panes : (preview.getWidth() >= preview.getHeight() * 2 ? 2 : 1);
	}
	refreshLayoutKey();

	// newest tracking results (published by the tracking thread)
	if(trackingManager.getLatestResults(results, resultsRev)) publishResults(results);
}

// Source or pane count changed: the worker resolves another layout, and
// so should any editor holding a copy.
void CameraTrackingSession::refreshLayoutKey(){
	const std::string source = sourceName();
	if(source == layoutSource && panes == layoutPanes) return;
	if(source != layoutSource){
		layoutSource = source;
		config.sourceName = source;
		trackingManager.setSourceName(source);
	}
	layoutPanes = panes;
	layoutRev++;
}

void CameraTrackingSession::publishResults(const tracking::TrackingResults & res){
	// canvas-space objects: the one mapping every output shares
	const std::vector<output::MotionObject> objects = output::fromTrackingResults(res);
	const double tSec = res.frameTMs / 1000.0;
	const double wall = wallClockSec();
	lastResultJson = output::buildFrame(objects, tSec, ++publishedFrames, nominalFps(), sourceName(),
	                                    res.canvas.width, res.canvas.height, wall);
	resultRev++;
	if(motionExport){
		// a new source means new track ids and a new time base: finish
		// this file and start the next one
		if(sourceName() != motionExportSource) startMotionExport();
		motionExport->addFrame(objects, tSec, wall);
	}
}

double CameraTrackingSession::nominalFps() const {
	if(camera.fps > 0) return camera.fps;
	if(mode == Mode::Playback){
		const double durationSec = player.getDurationMs() / 1000.0;
		if(durationSec > 0 && player.getFrameCount() > 1) return (player.getFrameCount() - 1) / durationSec;
	}else{
		const double measured = client.getStats().decodedFps;
		if(measured > 0) return measured;
	}
	return kFallbackFps;
}

//--------------------------------------------------------------
// source

std::string CameraTrackingSession::sourceName() const {
	return mode == Mode::Playback ? player.getCurrentName() : std::string("live");
}

bool CameraTrackingSession::setMode(Mode m){
	if(m == mode) return true;
	if(m == Mode::Playback){
		stopRecording();
		if(!player.scan(recordingsDir)){
			ofLogWarning("CameraTrackingSession") << "no recordings to play in " << recordingsDir;
			return false;
		}
		playbackSeenFrameId = 0;
		mode = Mode::Playback;
	}else{
		mode = Mode::Live;
		player.close();
	}
	preview.clear();
	previewRev++;
	trackingManager.reset(); // fresh background model / tracks for the new source
	refreshLayoutKey();
	return true;
}

void CameraTrackingSession::reconnect(){
	if(!started) return;
	if(camera.enabled && !camera.host.empty()) client.restart(camera);
}

double CameraTrackingSession::liveFrameAgeMs() const {
	return lastFrameReceivedMs > 0 ? MobotixMjpegClient::nowMs() - lastFrameReceivedMs : -1.0;
}

//--------------------------------------------------------------
// playback transport

void CameraTrackingSession::play(){
	if(mode == Mode::Playback) player.setPaused(false);
}

void CameraTrackingSession::pause(){
	if(mode == Mode::Playback) player.setPaused(true);
}

void CameraTrackingSession::seekTo(double positionSec){
	if(mode == Mode::Playback) player.seekTo(positionSec * 1000.0);
}

void CameraTrackingSession::seekBy(double deltaSec){
	if(mode == Mode::Playback) player.seekBy(static_cast<int>(deltaSec * 1000.0));
}

void CameraTrackingSession::stepFrame(int delta){
	if(mode == Mode::Playback) player.stepFrame(delta);
}

void CameraTrackingSession::nextRecording(){
	if(mode != Mode::Playback) return;
	player.next();
	trackingManager.reset();
}

void CameraTrackingSession::previousRecording(){
	if(mode != Mode::Playback) return;
	player.previous();
	trackingManager.reset();
}

//--------------------------------------------------------------
// recording

bool CameraTrackingSession::startRecording(){
	if(recorder.isRecording()) return true;
	if(mode != Mode::Live){
		ofLogWarning("CameraTrackingSession") << "recording only works in live mode";
		return false;
	}
	// frames reach the recorder through the permanent raw-frame tap
	return recorder.start(recordingsDir);
}

void CameraTrackingSession::stopRecording(){
	if(recorder.isRecording()) recorder.stop();
}

//--------------------------------------------------------------
// motion export

void CameraTrackingSession::startMotionExport(){
	stopMotionExport(); // a start always opens a fresh file
	motionExportSource = sourceName();
	ofDirectory::createDirectory(recordingsDir, false, true);
	motionExportPath = ofFilePath::join(recordingsDir,
		"motion_" + motionExportSource + "_" + ofGetTimestampString("%Y-%m-%d_%H-%M-%S") + ".json");
	// the editor file measures its own rate from the frames when it can
	motionExport = std::make_unique<output::MotionPathRecording>(
		motionExportSource, nominalFps(), config.canvas.width, config.canvas.height,
		output::MotionPathRecording::Style::Editor);
	ofLogNotice("CameraTrackingSession") << "motion export started -> " << motionExportPath;
}

void CameraTrackingSession::stopMotionExport(){
	if(!motionExport) return;
	const std::unique_ptr<output::MotionPathRecording> rec = std::move(motionExport);
	if(rec->frameCount() == 0){
		ofLogNotice("CameraTrackingSession") << "motion export discarded (no frames): " << motionExportPath;
		return;
	}
	if(rec->save(motionExportPath)){
		ofLogNotice("CameraTrackingSession") << "motion export written: " << motionExportPath
		                                      << " (" << rec->frameCount() << " frames)";
	}else{
		ofLogError("CameraTrackingSession") << "failed to write motion export " << motionExportPath;
	}
}

//--------------------------------------------------------------
// tracking

void CameraTrackingSession::setTrackingEnabled(bool enabled){
	config.enabled = enabled;
	trackingManager.setEnabled(enabled);
	if(enabled) trackingManager.reset(); // fresh background model on re-enable
}

void CameraTrackingSession::applyConfigToWorker(){
	if(started) trackingManager.setLiveConfig(config);
}

tracking::LayoutConfig CameraTrackingSession::currentLayout() const {
	return tracking::TrackingManager::findLayout(config.layouts, panes, sourceName());
}

tracking::LayoutConfig CameraTrackingSession::setLayout(const tracking::LayoutConfig & layout){
	tracking::LayoutConfig stored = layout;
	auto it = std::find_if(config.layouts.begin(), config.layouts.end(),
	                       [&](const tracking::LayoutConfig & l){ return l.name == stored.name; });
	if(it != config.layouts.end()){
		*it = stored;
	}else{
		// a synthesized fallback layout gets a proper name once edited
		stored.name = stored.panes > 1 ? "dual" : "single";
		config.layouts.push_back(stored);
	}
	applyConfigToWorker(); // the worker rebuilds its lanes (models and tracks restart)
	return stored;
}

std::vector<std::string> CameraTrackingSession::corridorIds() const {
	std::vector<std::string> ids;
	for(const auto & c : config.corridors) ids.push_back(c.id);
	for(const auto & lane : currentLayout().lanes){
		if(std::find(ids.begin(), ids.end(), lane.corridor) == ids.end()) ids.push_back(lane.corridor);
	}
	return ids;
}

void CameraTrackingSession::setPlacement(const std::string & corridorId, const tracking::CorridorPlacement & placement){
	auto it = std::find_if(config.corridors.begin(), config.corridors.end(),
	                       [&](const tracking::CorridorConfig & c){ return c.id == corridorId; });
	if(it == config.corridors.end()){
		tracking::CorridorConfig cc;
		cc.id = corridorId;
		cc.label = corridorId;
		config.corridors.push_back(cc);
		it = std::prev(config.corridors.end());
	}
	if(it->placement == placement) return;
	it->placement = placement;
	applyConfigToWorker(); // no rebuild — placements apply when results are published
}

//--------------------------------------------------------------
// outputs

bool CameraTrackingSession::takePreview(ofPixels & pixels, LensGeometry & geometry, uint64_t & lastSeenRevision) const {
	if(previewRev == lastSeenRevision) return false;
	lastSeenRevision = previewRev;
	pixels = preview;
	geometry = previewGeometry;
	return true;
}

bool CameraTrackingSession::takeResults(tracking::TrackingResults & out, uint64_t & lastSeenRevision) const {
	if(resultsRev == lastSeenRevision) return false;
	lastSeenRevision = resultsRev;
	out = results;
	return true;
}
