#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ofPixels.h"

#include "JpegCommentParser.h"
#include "MobotixTypes.h"

// Threaded HTTP MJPEG client for the authenticated Mobotix live stream
// (/control/faststream.jpg?stream=full&fps=0). Uses libcurl (already linked
// by the OF core) with Basic/Digest auth. Frames are decoded on the stream
// thread; only the newest frame is kept ("latest frame slot") — older frames
// are dropped, never queued, to keep display latency minimal.
//
// No UI dependencies, so YOUniverse_Steuerung can reuse this class later.
class MobotixMjpegClient {
public:
	~MobotixMjpegClient();

	void start(const CameraConfig & config);
	void stop();
	void restart(const CameraConfig & config);
	bool isRunning() const { return running; }

	// Copies out the newest frame if it is newer than lastSeenFrameId.
	// Returns true and updates lastSeenFrameId when a new frame was copied.
	// receivedAtMs is the client-side steady-clock timestamp (ms) at which
	// the frame finished decoding on the stream thread.
	bool getLatestFrame(ofPixels & pixels, SensorSnapshot & sensors, uint64_t & lastSeenFrameId,
	                    double & receivedAtMs, std::string * comDump = nullptr);

	StreamStats getStats() const;
	std::string getStreamUrl() const;

	// Steady-clock "now" in ms, same timebase as receivedAtMs.
	static double nowMs();

private:
	void threadFn();
	void processBuffer();
	void handleCompleteJpeg(const uint8_t * data, size_t size);
	// Finds one complete JPEG in receiveBuffer. Returns true and sets
	// [start,end) when found; sets needMore when the buffer just ends
	// mid-frame.
	bool findCompleteJpeg(size_t & start, size_t & end, bool & needMore) const;

	// libcurl trampolines (defined in .cpp, keep curl out of this header)
	friend size_t mobotixWriteCb(char * ptr, size_t size, size_t nmemb, void * userdata);
	friend int mobotixXferCb(void * userdata, int64_t, int64_t, int64_t, int64_t);

	CameraConfig config;
	mutable std::mutex configMutex;

	std::thread streamThread;
	std::atomic<bool> running{false};

	// stream-thread only
	std::vector<uint8_t> receiveBuffer;
	void * currentCurl = nullptr; // CURL*, valid while perform() runs
	double statWindowStartMs = 0;
	uint64_t framesInWindow = 0;
	uint64_t bytesInWindow = 0;

	// shared latest-frame slot
	mutable std::mutex frameMutex;
	ofPixels latestPixels;
	SensorSnapshot latestSensors;
	std::string latestComDump;
	uint64_t latestFrameId = 0;
	double latestReceivedAtMs = 0;
	bool latestConsumed = true;

	// shared stats
	mutable std::mutex statsMutex;
	StreamStats stats;
};
