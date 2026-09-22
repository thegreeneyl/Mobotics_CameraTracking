#include "MjpegPlayer.h"

#include <algorithm>
#include <chrono>

#include "ofFileUtils.h"
#include "ofImage.h"
#include "ofJson.h"
#include "ofLog.h"

#include "../mobotix/JpegCommentParser.h"

namespace {
// After the last frame of a recording was shown, hold it for one extra frame
// interval before cycling to the next recording.
constexpr double kFallbackTailMs = 66.0;

std::string sidecarPathFor(const std::string & mjpegPath){
	return mjpegPath.substr(0, mjpegPath.size() - std::string(".mjpeg").size()) + ".json";
}
} // namespace

double MjpegPlayer::nowMs(){
	using namespace std::chrono;
	return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
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

	return open(0);
}

void MjpegPlayer::close(){
	if(fileStream.is_open()) fileStream.close();
	fileStream.clear();
	frames.clear();
	durationMs = 0;
	frameCursor = 0;
}

bool MjpegPlayer::open(size_t index){
	if(playlist.empty()) return false;
	// Try the requested recording; on failure walk on (older, wrapping) so a
	// single corrupt file cannot kill playback.
	for(size_t attempt = 0; attempt < playlist.size(); attempt++){
		const size_t idx = (index + attempt) % playlist.size();
		close();
		if(!loadSidecar(playlist[idx])) continue;
		fileStream.open(playlist[idx], std::ios::binary);
		if(!fileStream.is_open()){
			ofLogError("MjpegPlayer") << "cannot open " << playlist[idx];
			continue;
		}
		currentIndex = idx;
		frameCursor = 0;
		clockStartMs = nowMs();
		ofLogNotice("MjpegPlayer")
			<< "playing " << getCurrentName() << " (" << (idx + 1) << "/" << playlist.size()
			<< ", " << frames.size() << " frames, " << durationMs / 1000.0 << " s)";
		return true;
	}
	close();
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
	if(playlist.empty()) return;
	open((currentIndex + 1) % playlist.size());
}

void MjpegPlayer::previous(){
	if(playlist.empty()) return;
	open((currentIndex + playlist.size() - 1) % playlist.size());
}

bool MjpegPlayer::update(ofPixels & pixels, SensorSnapshot & sensors, std::string * comDump){
	if(!isOpen() || frames.empty()) return false;

	const double elapsed = nowMs() - clockStartMs;

	// Finished? Hold the last frame for one frame interval, then cycle on.
	if(frameCursor >= frames.size()){
		double tailMs = kFallbackTailMs;
		if(frames.size() > 1){
			tailMs = std::max(kFallbackTailMs, durationMs / (frames.size() - 1));
		}
		if(elapsed >= durationMs + tailMs){
			next();
			// Show frame 0 of the next recording on the following update.
		}
		return false;
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
	if(!anyDue) return false;

	if(!decodeFrame(dueIdx, pixels, sensors, comDump)){
		// Corrupt frame — skip it; the next update tries the following one.
		return false;
	}
	return true;
}

bool MjpegPlayer::decodeFrame(size_t frameIdx, ofPixels & pixels, SensorSnapshot & sensors,
                              std::string * comDump){
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

	sensors = JpegCommentParser::parse(readBuffer.data(), readBuffer.size(), comDump);

	ofBuffer jpegBuffer(reinterpret_cast<const char *>(readBuffer.data()), readBuffer.size());
	if(!ofLoadImage(pixels, jpegBuffer)){
		ofLogWarning("MjpegPlayer") << "failed to decode frame " << frameIdx
		                            << " of " << getCurrentName();
		return false;
	}
	return true;
}

// ---------------------------------------------------------------------- info

std::string MjpegPlayer::getCurrentName() const {
	if(playlist.empty() || currentIndex >= playlist.size()) return "";
	return ofFilePath::getBaseName(playlist[currentIndex]);
}

double MjpegPlayer::getPositionMs() const {
	if(!fileStream.is_open()) return 0;
	const double pos = nowMs() - clockStartMs;
	return std::min(pos, durationMs);
}
