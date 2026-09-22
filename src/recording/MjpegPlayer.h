#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "ofPixels.h"

#include "../mobotix/MobotixTypes.h"

// Plays back recordings made by MjpegRecorder: rec_*.mjpeg files with a JSON
// sidecar of per-frame {offset, size, tMs} entries. Frames are read from disk
// on demand via the sidecar offsets (files can be hundreds of MB — never
// fully loaded) and paced by their recorded timestamps.
//
// The playlist is sorted newest-first (timestamps in the filename sort
// lexicographically). Playback starts at the most recent recording; when one
// finishes it advances to the next older one, wrapping to the newest after
// the oldest.
class MjpegPlayer {
public:
	// Rebuilds the playlist from dirAbs (absolute path) and opens the most
	// recent recording. Returns false when no playable recording exists.
	bool scan(const std::string & dirAbs);
	void close();

	bool isOpen() const { return fileStream.is_open(); }

	// Jump within the cycle (wrapping); the target starts from frame 0.
	void next();     // older recording
	void previous(); // newer recording

	// Advances the playback clock; returns true when a new frame is due and
	// was decoded into pixels/sensors (comDump optional). Automatically
	// advances to the next recording when the current one finishes.
	bool update(ofPixels & pixels, SensorSnapshot & sensors, std::string * comDump = nullptr);

	// info for the stats bar
	std::string getCurrentName() const;   // e.g. "rec_2026-09-22_11-06-33"
	size_t getCurrentIndex() const { return currentIndex; } // 0 = newest
	size_t getCount() const { return playlist.size(); }
	double getPositionMs() const;
	double getDurationMs() const { return durationMs; }
	size_t getFrameCursor() const { return frameCursor; }
	size_t getFrameCount() const { return frames.size(); }

private:
	struct FrameIndex {
		uint64_t offset = 0;
		uint64_t size = 0;
		double tMs = 0;
	};

	// Opens playlist[index]; on failure tries the following (older) entries
	// once around the cycle. Returns false when nothing could be opened.
	bool open(size_t index);
	bool loadSidecar(const std::string & mjpegPath);
	bool decodeFrame(size_t frameIdx, ofPixels & pixels, SensorSnapshot & sensors,
	                 std::string * comDump);

	static double nowMs();

	std::vector<std::string> playlist; // absolute .mjpeg paths, newest first
	size_t currentIndex = 0;

	// current recording
	std::ifstream fileStream;
	std::vector<FrameIndex> frames;
	double durationMs = 0;
	size_t frameCursor = 0;   // next frame to show
	double clockStartMs = 0;  // steady-clock ms when frame 0 was due
	std::vector<uint8_t> readBuffer;
};
