#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ofPixels.h"

#include "../mobotix/MobotixTypes.h"

// Plays back recordings made by MjpegRecorder: rec_*.mjpeg files with a JSON
// sidecar of per-frame {offset, size, tMs} entries. Frames are read from disk
// on demand via the sidecar offsets (files can be hundreds of MB — never
// fully loaded) and paced by their recorded timestamps.
//
// Threaded: pacing, disk reads and JPEG decoding all run on a dedicated
// playback thread; the main thread only polls the latest decoded frame,
// mirroring MobotixMjpegClient's latest-frame-slot API. A raw-frame tap
// provides the undecoded JPEG bytes of every displayed frame (tracking).
//
// The playlist is sorted newest-first (timestamps in the filename sort
// lexicographically). Playback starts at the most recent recording; when one
// finishes it advances to the next older one, wrapping to the newest after
// the oldest.
class MjpegPlayer {
public:
	~MjpegPlayer();

	// Rebuilds the playlist from dirAbs (absolute path) and starts the
	// playback thread on the most recent recording. Returns false when no
	// playable recording exists.
	bool scan(const std::string & dirAbs);
	void close(); // stops the thread

	bool isOpen() const { return running; }

	// Jump within the cycle (wrapping); the target starts from frame 0.
	void next();     // older recording
	void previous(); // newer recording

	// Copies out the newest decoded frame if newer than lastSeenFrameId.
	// frameTMs is the frame's recording-relative timestamp in ms.
	bool getLatestFrame(ofPixels & pixels, SensorSnapshot & sensors,
	                    uint64_t & lastSeenFrameId, double & frameTMs,
	                    std::string * comDump = nullptr);

	// Raw-frame tap: called on the playback thread with the exact bytes of
	// every displayed JPEG plus its recording-relative timestamp. Survives
	// scan()/close(); the callback must be fast (copy into a queue).
	using RawFrameCallback = std::function<void(const uint8_t * data, size_t size, double tMs)>;
	void setRawFrameCallback(RawFrameCallback callback);

	// Display decode scale: 1 = full res, 2 = DCT half-size (quarter cost).
	void setDecodeScale(int scale);

	// info for the stats bar (thread-safe snapshots)
	std::string getCurrentName() const;   // e.g. "rec_2026-09-22_11-06-33"
	size_t getCurrentIndex() const;       // 0 = newest
	size_t getCount() const;
	double getPositionMs() const;
	double getDurationMs() const;
	size_t getFrameCursor() const;
	size_t getFrameCount() const;
	double getLastDecodeMs() const;

private:
	struct FrameIndex {
		uint64_t offset = 0;
		uint64_t size = 0;
		double tMs = 0;
	};

	void threadFn();
	// Opens playlist[index]; on failure tries the following (older) entries
	// once around the cycle. Returns false when nothing could be opened.
	bool open(size_t index);
	bool loadSidecar(const std::string & mjpegPath);
	bool decodeAndPublish(size_t frameIdx);
	void publishInfo();

	static double nowMs();

	std::thread playThread;
	std::atomic<bool> running{false};
	std::atomic<int> pendingJump{0}; // accumulated next(+1)/previous(-1)
	std::atomic<int> decodeScale{1};

	// playback-thread state
	std::vector<std::string> playlist; // absolute .mjpeg paths, newest first
	size_t currentIndex = 0;
	std::ifstream fileStream;
	std::vector<FrameIndex> frames;
	double durationMs = 0;
	size_t frameCursor = 0;   // next frame to show
	double clockStartMs = 0;  // steady-clock ms when frame 0 was due
	std::vector<uint8_t> readBuffer;
	double lastDecodeMs = 0;

	// latest decoded frame slot
	mutable std::mutex frameMutex;
	ofPixels latestPixels;
	SensorSnapshot latestSensors;
	std::string latestComDump;
	uint64_t latestFrameId = 0;
	double latestFrameTMs = 0;

	// info mirror for the getters
	mutable std::mutex infoMutex;
	struct Info {
		std::string name;
		size_t index = 0;
		size_t count = 0;
		double durationMs = 0;
		size_t frameCursor = 0;
		size_t frameCount = 0;
		double clockStartMs = 0;
		double lastDecodeMs = 0;
	} info;

	mutable std::mutex rawFrameMutex;
	RawFrameCallback rawFrameCallback;
};
