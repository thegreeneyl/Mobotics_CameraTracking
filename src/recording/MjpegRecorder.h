#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Records a raw MJPEG stream to disk: each incoming JPEG is appended
// unchanged to one rec_<timestamp>.mjpeg file per recording, and a JSON
// sidecar (rec_<timestamp>.json) with per-frame {offset, size, tMs} entries
// is written on stop() so playback can be paced exactly like the original
// stream.
//
// addFrame() only copies the bytes into a queue (safe to call from the
// stream thread); a dedicated writer thread does all disk I/O.
//
// No UI dependencies (same rule as src/mobotix/), so YOUniverse_Steuerung
// can reuse this class later.
class MjpegRecorder {
public:
	~MjpegRecorder();

	// dirAbs: absolute path of the recordings directory (created if missing).
	// Returns false when the output file cannot be created.
	bool start(const std::string & dirAbs);
	// Flushes the queue, closes the .mjpeg file and writes the sidecar.
	void stop();
	bool isRecording() const { return recording; }

	// Called on the stream thread with the raw bytes of one complete JPEG.
	// tsMs is a steady-clock timestamp in ms (MobotixMjpegClient::nowMs()).
	void addFrame(const uint8_t * data, size_t size, double tsMs);

	// For the REC indicator / logging.
	std::string getBaseName() const;      // e.g. "rec_2026-09-22_11-06-33"
	uint64_t getFramesWritten() const { return framesWritten; }
	uint64_t getBytesWritten() const { return bytesWritten; }
	double getDurationMs() const;         // last frame ts - first frame ts

private:
	struct QueuedFrame {
		std::vector<uint8_t> bytes;
		double tsMs = 0;
	};

	struct IndexEntry {
		uint64_t offset = 0;
		uint64_t size = 0;
		double tMs = 0; // relative to first frame
	};

	void writerFn();
	void writeSidecar();

	std::atomic<bool> recording{false};
	std::thread writerThread;

	std::mutex queueMutex;
	std::condition_variable queueCv;
	std::deque<QueuedFrame> queue;
	size_t queuedBytes = 0;

	// writer-thread only (safe to read after join in stop())
	std::ofstream file;
	std::vector<IndexEntry> index;
	uint64_t writeOffset = 0;
	double firstFrameTsMs = -1;
	double lastFrameTsMs = -1;

	mutable std::mutex nameMutex;
	std::string baseName;    // rec_<timestamp>
	std::string mjpegPath;   // absolute
	std::string sidecarPath; // absolute

	std::atomic<uint64_t> framesWritten{0};
	std::atomic<uint64_t> bytesWritten{0};
};
