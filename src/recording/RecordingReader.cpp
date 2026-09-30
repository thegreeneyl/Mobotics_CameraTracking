#include "RecordingReader.h"

#include "ofFileUtils.h"
#include "ofJson.h"
#include "ofLog.h"

bool RecordingReader::open(const std::string & mjpegPath){
	close();
	const std::string ext = ".mjpeg";
	if(mjpegPath.size() <= ext.size() || mjpegPath.compare(mjpegPath.size() - ext.size(), ext.size(), ext) != 0){
		ofLogError("RecordingReader") << "not an .mjpeg path: " << mjpegPath;
		return false;
	}
	const std::string sidecar = mjpegPath.substr(0, mjpegPath.size() - ext.size()) + ".json";
	ofJson json;
	try {
		std::ifstream in(sidecar);
		if(!in.is_open()){
			ofLogError("RecordingReader") << "cannot read sidecar " << sidecar;
			return false;
		}
		in >> json;
	} catch(const std::exception & e){
		ofLogError("RecordingReader") << "invalid sidecar " << sidecar << ": " << e.what();
		return false;
	}
	if(!json.contains("frames") || !json["frames"].is_array() || json["frames"].empty()){
		ofLogError("RecordingReader") << "sidecar has no frames: " << sidecar;
		return false;
	}
	frames.reserve(json["frames"].size());
	for(const auto & f : json["frames"]){
		FrameInfo entry;
		entry.offset = f.value("o", 0ULL);
		entry.size = f.value("s", 0ULL);
		entry.tMs = f.value("t", 0.0);
		frames.push_back(entry);
	}
	duration = json.value("durationMs", frames.back().tMs);
	stream.open(mjpegPath, std::ios::binary);
	if(!stream.is_open()){
		ofLogError("RecordingReader") << "cannot open " << mjpegPath;
		frames.clear();
		return false;
	}
	baseName = ofFilePath::getBaseName(mjpegPath);
	return true;
}

void RecordingReader::close(){
	if(stream.is_open()) stream.close();
	stream.clear();
	frames.clear();
	duration = 0;
	baseName.clear();
}

bool RecordingReader::read(size_t idx, std::vector<uint8_t> & out){
	if(idx >= frames.size()) return false;
	const FrameInfo & f = frames[idx];
	out.resize(f.size);
	stream.clear();
	stream.seekg(static_cast<std::streamoff>(f.offset));
	stream.read(reinterpret_cast<char *>(out.data()), static_cast<std::streamsize>(f.size));
	return stream.good();
}
