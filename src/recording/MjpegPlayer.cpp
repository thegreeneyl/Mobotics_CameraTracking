#include "MjpegPlayer.h"

#include <algorithm>
#include <chrono>

#include "ofFileUtils.h"
#include "ofJson.h"
#include "ofLog.h"

#include "../mobotix/JpegCommentParser.h"
#include "../util/JpegDecode.h"

namespace {
// After the last frame of a recording was shown, hold it for one extra frame
// interval before cycling to the next recording.
constexpr double kFallbackTailMs = 66.0;
// Idle sleep granularity of the playback thread.
constexpr int kIdleSleepMs = 2;

std::string sidecarPathFor(const std::string & mjpegPath){
	return mjpegPath.substr(0, mjpegPath.size() - std::string(".mjpeg").size()) + ".json";
}
} // namespace

double MjpegPlayer::nowMs(){
	using namespace std::chrono;
	return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

MjpegPlayer::~MjpegPlayer(){
	close();
}

// ------------------------------------------------------------------ playlist

bool MjpegPlayer::scan(const std::string & dirAbs){
	close();
	playlist.clear();
	currentIndex = 0;

	ofDirectory dir(dirAbs);
	if(!dir.exists()){
		ofLogWarning("MjpegPlayer") << "recordings dir does not exist: " << dirAbs;
		return false;
	}
	dir.allowExt("mjpeg");
	dir.listDir();
	for(size_t i = 0; i < dir.size(); i++){
		const std::string path = dir.getPath(i);
		if(!ofFile::doesFileExist(sidecarPathFor(path), false)){
			ofLogWarning("MjpegPlayer") << "skipping " << path << " (no .json sidecar)";
			continue;
		}
		playlist.push_back(path);
	}
	if(playlist.empty()){
		ofLogWarning("MjpegPlayer") << "no playable recordings in " << dirAbs;
		return false;
	}

	// Timestamps in the filename sort lexicographically -> newest first.
	std::sort(playlist.begin(), playlist.end(),
	          [](const std::string & a, const std::string & b){
		return ofFilePath::getFileName(a) > ofFilePath::getFileName(b);
	});

	pendingJump = 0;
	running = true;
	playThread = std::thread(&MjpegPlayer::threadFn, this);
	return true;
}

void MjpegPlayer::close(){
	running = false;
	if(playThread.joinable()) playThread.join();
	if(fileStream.is_open()) fileStream.close();
	fileStream.clear();
	frames.clear();
	durationMs = 0;
	frameCursor = 0;
	{
		std::lock_guard<std::mutex> lock(frameMutex);
		latestFrameId = 0;
	}
	{
		std::lock_guard<std::mutex> lock(infoMutex);
		info = Info{};
	}
}

bool MjpegPlayer::open(size_t index){
	if(playlist.empty()) return false;
	// Try the requested recording; on failure walk on (older, wrapping) so a
	// single corrupt file cannot kill playback.
	for(size_t attempt = 0; attempt < playlist.size(); attempt++){
		const size_t idx = (index + attempt) % playlist.size();
		if(fileStream.is_open()) fileStream.close();
		fileStream.clear();
		frames.clear();
		durationMs = 0;
		frameCursor = 0;
		if(!loadSidecar(playlist[idx])) continue;
		fileStream.open(playlist[idx], std::ios::binary);
		if(!fileStream.is_open()){
			ofLogError("MjpegPlayer") << "cannot open " << playlist[idx];
			continue;
		}
		currentIndex = idx;
		frameCursor = 0;
		clockStartMs = nowMs();
		publishInfo();
		ofLogNotice("MjpegPlayer")
			<< "playing " << getCurrentName() << " (" << (idx + 1) << "/" << playlist.size()
			<< ", " << frames.size() << " frames, " << durationMs / 1000.0 << " s)";
		return true;
	}
	frames.clear();
	ofLogError("MjpegPlayer") << "no recording could be opened";
	return false;
}

bool MjpegPlayer::loadSidecar(const std::string & mjpegPath){
	const std::string path = sidecarPathFor(mjpegPath);
	ofJson json;
	try {
		std::ifstream in(path);
		if(!in.is_open()){
			ofLogError("MjpegPlayer") << "cannot read sidecar " << path;
			return false;
		}
		in >> json;
	} catch(const std::exception & e){
		ofLogError("MjpegPlayer") << "invalid sidecar " << path << ": " << e.what();
		return false;
	}
	if(!json.contains("frames") || !json["frames"].is_array() || json["frames"].empty()){
		ofLogError("MjpegPlayer") << "sidecar has no frames: " << path;
		return false;
	}

	frames.clear();
	frames.reserve(json["frames"].size());
	for(const auto & f : json["frames"]){
		FrameIndex entry;
		entry.offset = f.value("o", 0ULL);
		entry.size = f.value("s", 0ULL);
		entry.tMs = f.value("t", 0.0);
		frames.push_back(entry);
	}
	durationMs = json.value("durationMs", frames.back().tMs);
	return true;
}

// ------------------------------------------------------------------ playback

void MjpegPlayer::next(){
	pendingJump++;
}

void MjpegPlayer::previous(){
	pendingJump--;
}

void MjpegPlayer::threadFn(){
	if(!open(0)){
		running = false;
		return;
	}

	while(running){
		// Apply queued jumps (arrow keys), wrapping in either direction.
		const int jump = pendingJump.exchange(0);
		if(jump != 0 && !playlist.empty()){
			const int n = static_cast<int>(playlist.size());
			const int target = ((static_cast<int>(currentIndex) + jump) % n + n) % n;
			open(static_cast<size_t>(target));
		}
		if(frames.empty()){
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			continue;
		}

		const double elapsed = nowMs() - clockStartMs;

		// Finished? Hold the last frame for one frame interval, then cycle on.
		if(frameCursor >= frames.size()){
			double tailMs = kFallbackTailMs;
			if(frames.size() > 1){
				tailMs = std::max(kFallbackTailMs, durationMs / (frames.size() - 1));
			}
			if(elapsed >= durationMs + tailMs){
				open((currentIndex + 1) % playlist.size());
			}else{
				std::this_thread::sleep_for(std::chrono::milliseconds(kIdleSleepMs));
			}
			continue;
		}

		// Collect all frames that are due and show only the newest of them
		// (skip stale ones so playback keeps up even if decoding is slow).
		bool anyDue = false;
		size_t dueIdx = 0;
		while(frameCursor < frames.size() && frames[frameCursor].tMs <= elapsed){
			anyDue = true;
			dueIdx = frameCursor;
			frameCursor++;
		}
		if(anyDue){
			decodeAndPublish(dueIdx); // corrupt frame -> just skip it
			publishInfo();
		}else{
			const double waitMs = frames[frameCursor].tMs - elapsed;
			const int sleepMs = std::clamp(static_cast<int>(waitMs), kIdleSleepMs, 20);
			std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
		}
	}
}

bool MjpegPlayer::decodeAndPublish(size_t frameIdx){
	const FrameIndex & f = frames[frameIdx];
	readBuffer.resize(f.size);
	fileStream.clear();
	fileStream.seekg(static_cast<std::streamoff>(f.offset));
	fileStream.read(reinterpret_cast<char *>(readBuffer.data()),
	                static_cast<std::streamsize>(f.size));
	if(!fileStream.good()){
		ofLogWarning("MjpegPlayer") << "read failed at frame " << frameIdx
		                            << " of " << getCurrentName();
		return false;
	}

	// Raw tap (tracking) — before decode, exactly like the live client.
	{
		std::lock_guard<std::mutex> lock(rawFrameMutex);
		if(rawFrameCallback) rawFrameCallback(readBuffer.data(), readBuffer.size(), f.tMs);
	}

	std::string comDump;
	SensorSnapshot sensors = JpegCommentParser::parse(readBuffer.data(), readBuffer.size(), &comDump);

	const double decodeStartMs = nowMs();
	ofPixels pixels;
	if(!jpegdecode::decodeToPixels(readBuffer.data(), readBuffer.size(), pixels, decodeScale)){
		ofLogWarning("MjpegPlayer") << "failed to decode frame " << frameIdx
		                            << " of " << getCurrentName();
		return false;
	}
	lastDecodeMs = nowMs() - decodeStartMs;

	{
		std::lock_guard<std::mutex> lock(frameMutex);
		latestPixels = std::move(pixels);
		latestSensors = sensors;
		latestComDump = std::move(comDump);
		latestFrameId++;
		latestFrameTMs = f.tMs;
	}
	return true;
}

bool MjpegPlayer::getLatestFrame(ofPixels & pixels, SensorSnapshot & sensors,
                                 uint64_t & lastSeenFrameId, double & frameTMs,
                                 std::string * comDump){
	std::lock_guard<std::mutex> lock(frameMutex);
	if(latestFrameId == 0 || latestFrameId == lastSeenFrameId) return false;
	pixels = latestPixels;
	sensors = latestSensors;
	frameTMs = latestFrameTMs;
	if(comDump) *comDump = latestComDump;
	lastSeenFrameId = latestFrameId;
	return true;
}

void MjpegPlayer::setRawFrameCallback(RawFrameCallback callback){
	std::lock_guard<std::mutex> lock(rawFrameMutex);
	rawFrameCallback = std::move(callback);
}

void MjpegPlayer::setDecodeScale(int scale){
	if(scale >= 8) scale = 8;
	else if(scale >= 4) scale = 4;
	else if(scale >= 2) scale = 2;
	else scale = 1;
	decodeScale = scale;
}

// ---------------------------------------------------------------------- info

void MjpegPlayer::publishInfo(){
	std::lock_guard<std::mutex> lock(infoMutex);
	info.name = (playlist.empty() || currentIndex >= playlist.size())
		? "" : ofFilePath::getBaseName(playlist[currentIndex]);
	info.index = currentIndex;
	info.count = playlist.size();
	info.durationMs = durationMs;
	info.frameCursor = frameCursor;
	info.frameCount = frames.size();
	info.clockStartMs = clockStartMs;
	info.lastDecodeMs = lastDecodeMs;
}

std::string MjpegPlayer::getCurrentName() const {
	std::lock_guard<std::mutex> lock(infoMutex);
	return info.name;
}

size_t MjpegPlayer::getCurrentIndex() const {
	std::lock_guard<std::mutex> lock(infoMutex);
	return info.index;
}

size_t MjpegPlayer::getCount() const {
	std::lock_guard<std::mutex> lock(infoMutex);
	return info.count;
}

double MjpegPlayer::getPositionMs() const {
	std::lock_guard<std::mutex> lock(infoMutex);
	if(!running || info.clockStartMs <= 0) return 0;
	return std::min(nowMs() - info.clockStartMs, info.durationMs);
}

double MjpegPlayer::getDurationMs() const {
	std::lock_guard<std::mutex> lock(infoMutex);
	return info.durationMs;
}

size_t MjpegPlayer::getFrameCursor() const {
	std::lock_guard<std::mutex> lock(infoMutex);
	return info.frameCursor;
}

size_t MjpegPlayer::getFrameCount() const {
	std::lock_guard<std::mutex> lock(infoMutex);
	return info.frameCount;
}

double MjpegPlayer::getLastDecodeMs() const {
	std::lock_guard<std::mutex> lock(infoMutex);
	return info.lastDecodeMs;
}
