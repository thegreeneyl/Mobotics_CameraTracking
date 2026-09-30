#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

// Synchronous, unpaced reader for MjpegRecorder recordings (rec_*.mjpeg +
// JSON sidecar). Used by the headless batch runner: frames are read on
// demand via the sidecar offsets, in order, as fast as the caller wants.
// MjpegPlayer remains the paced, threaded variant for the GUI.
class RecordingReader {
public:
	struct FrameInfo {
		uint64_t offset = 0;
		uint64_t size = 0;
		double tMs = 0;
	};

	// mjpegPath: absolute path to the .mjpeg; the sidecar is derived.
	bool open(const std::string & mjpegPath);
	void close();

	size_t frameCount() const { return frames.size(); }
	double durationMs() const { return duration; }
	const FrameInfo & frame(size_t idx) const { return frames[idx]; }
	const std::string & name() const { return baseName; } // e.g. rec_2026-09-23_14-34-16

	// Reads the raw JPEG bytes of frame idx into out. False on I/O error.
	bool read(size_t idx, std::vector<uint8_t> & out);

private:
	std::ifstream stream;
	std::vector<FrameInfo> frames;
	double duration = 0;
	std::string baseName;
};
