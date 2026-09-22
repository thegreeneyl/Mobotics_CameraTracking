#include "MjpegRecorder.h"

#include "ofFileUtils.h"
#include "ofJson.h"
#include "ofLog.h"
#include "ofUtils.h"

namespace {
// Runaway guard: if the writer cannot keep up (dead disk, etc.) drop new
// frames instead of eating RAM. ~256 MB is ~50 s of the full stream.
constexpr size_t kMaxQueuedBytes = 256 * 1024 * 1024;
} // namespace

MjpegRecorder::~MjpegRecorder(){
	stop();
}

bool MjpegRecorder::start(const std::string & dirAbs){
	stop();

	ofDirectory dir(dirAbs);
	if(!dir.exists() && !dir.create(true)){
		ofLogError("MjpegRecorder") << "cannot create recordings dir " << dirAbs;
		return false;
	}

	const std::string stamp = ofGetTimestampString("%Y-%m-%d_%H-%M-%S");
	{
		std::lock_guard<std::mutex> lock(nameMutex);
		baseName = "rec_" + stamp;
		mjpegPath = ofFilePath::join(dirAbs, baseName + ".mjpeg");
		sidecarPath = ofFilePath::join(dirAbs, baseName + ".json");
	}

	file.open(mjpegPath, std::ios::binary | std::ios::trunc);
	if(!file.is_open()){
		ofLogError("MjpegRecorder") << "cannot open " << mjpegPath << " for writing";
		return false;
	}

	index.clear();
	writeOffset = 0;
	firstFrameTsMs = -1;
	lastFrameTsMs = -1;
	framesWritten = 0;
	bytesWritten = 0;
	{
		std::lock_guard<std::mutex> lock(queueMutex);
		queue.clear();
		queuedBytes = 0;
	}

	recording = true;
	writerThread = std::thread(&MjpegRecorder::writerFn, this);
	ofLogNotice("MjpegRecorder") << "recording to " << mjpegPath;
	return true;
}

void MjpegRecorder::stop(){
	if(!recording && !writerThread.joinable()) return;
	recording = false;
	queueCv.notify_all();
	if(writerThread.joinable()) writerThread.join();

	if(file.is_open()){
		file.close();
		writeSidecar();
		ofLogNotice("MjpegRecorder")
			<< "recording finished: " << getBaseName()
			<< " (" << framesWritten << " frames, "
			<< bytesWritten / (1024.0 * 1024.0) << " MB, "
			<< getDurationMs() / 1000.0 << " s)";
	}
}

void MjpegRecorder::addFrame(const uint8_t * data, size_t size, double tsMs){
	if(!recording || size == 0) return;
	std::lock_guard<std::mutex> lock(queueMutex);
	if(queuedBytes + size > kMaxQueuedBytes){
		ofLogWarning("MjpegRecorder") << "write queue full, dropping frame";
		return;
	}
	QueuedFrame frame;
	frame.bytes.assign(data, data + size);
	frame.tsMs = tsMs;
	queuedBytes += size;
	queue.push_back(std::move(frame));
	queueCv.notify_one();
}

std::string MjpegRecorder::getBaseName() const {
	std::lock_guard<std::mutex> lock(nameMutex);
	return baseName;
}

double MjpegRecorder::getDurationMs() const {
	// firstFrameTsMs/lastFrameTsMs are written by the writer thread; reading
	// doubles for a UI counter is benign. Guard the "no frame yet" case only.
	if(firstFrameTsMs < 0 || lastFrameTsMs < 0) return 0;
	return lastFrameTsMs - firstFrameTsMs;
}

// ------------------------------------------------------------- writer thread

void MjpegRecorder::writerFn(){
	while(true){
		QueuedFrame frame;
		{
			std::unique_lock<std::mutex> lock(queueMutex);
			queueCv.wait(lock, [this]{ return !queue.empty() || !recording; });
			if(queue.empty()){
				if(!recording) return; // drained and stopped
				continue;
			}
			frame = std::move(queue.front());
			queue.pop_front();
			queuedBytes -= frame.bytes.size();
		}

		if(firstFrameTsMs < 0) firstFrameTsMs = frame.tsMs;
		lastFrameTsMs = frame.tsMs;

		IndexEntry entry;
		entry.offset = writeOffset;
		entry.size = frame.bytes.size();
		entry.tMs = frame.tsMs - firstFrameTsMs;

		file.write(reinterpret_cast<const char *>(frame.bytes.data()),
		           static_cast<std::streamsize>(frame.bytes.size()));
		if(!file.good()){
			ofLogError("MjpegRecorder") << "write failed on " << mjpegPath << ", stopping recording";
			recording = false;
			return;
		}
		index.push_back(entry);
		writeOffset += entry.size;
		framesWritten++;
		bytesWritten += entry.size;
	}
}

void MjpegRecorder::writeSidecar(){
	ofJson json;
	json["version"] = 1;
	json["file"] = getBaseName() + ".mjpeg";
	json["frameCount"] = index.size();
	json["durationMs"] = getDurationMs();
	auto & frames = json["frames"];
	frames = ofJson::array();
	for(const auto & e : index){
		frames.push_back({{"o", e.offset}, {"s", e.size}, {"t", e.tMs}});
	}

	std::string path;
	{
		std::lock_guard<std::mutex> lock(nameMutex);
		path = sidecarPath;
	}
	std::ofstream out(path, std::ios::trunc);
	if(!out.is_open()){
		ofLogError("MjpegRecorder") << "cannot write sidecar " << path;
		return;
	}
	out << json.dump();
}
