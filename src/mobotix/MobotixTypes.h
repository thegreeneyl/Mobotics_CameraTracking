#pragma once

#include <cstdint>
#include <string>

// Connection settings for one Mobotix camera CPU (Mx-S74A).
// Loaded from bin/data/config/config.json.
struct CameraConfig {
	std::string id;
	bool enabled = false;
	std::string host;
	std::string user;
	std::string password;

	// Per-stream parameters for /control/faststream.jpg. These do NOT touch
	// the camera's saved configuration (no control?set calls).
	std::string stream = "full"; // "full" = independent JPEGs (decodable by OF)
	float fps = 0.0f;            // 0 = maximum rate
	bool preview = false;        // request downscaled preview frames
	std::string previewSize = "1280x720";
	int quality = 60;            // JPEG quality, only applied in preview mode

	// Authenticated live stream URL (user path, not the ~2 fps guest path).
	std::string buildStreamUrl() const {
		std::string url = "http://" + host + "/control/faststream.jpg?stream=" + stream;
		// fps as plain number; 0 means max rate
		url += "&fps=" + (fps <= 0.0f ? std::string("0") : std::to_string(fps));
		if(preview){
			url += "&preview&size=" + previewSize + "&quality=" + std::to_string(quality);
		}
		return url;
	}
};

// Raw sensor values parsed from the JPEG COM comment (SECTION SENSORS /
// SECTION IMAGE) of a Mobotix frame. All raw integers, -1 = not present.
//
// Verified on S74 MX-V7.3.1.57 (192.168.178.230):
//   LA2 = M3 MultiSense (Mx-F-MSA) illumination, tenths of lux
//   LXL / LXR = left / right optical sensor illumination, tenths of lux
//   PI2 = MultiSense PIR activity 0..100 (%)
//   TC2 = MultiSense temperature, tenths of degC
//   TIN = camera board temperature, tenths of degC
//   FRJ = camera-reported frame rate, tenths of fps
struct SensorSnapshot {
	int la2 = -1;
	int lxl = -1;
	int lxr = -1;
	int pi2 = -1;
	int tc2 = -1;
	int tin = -1;
	int frj = -1;
	int width = 0;
	int height = 0;
	std::string cam; // BOTH / LEFT / RIGHT
	bool valid = false;

	float multiSenseLux() const { return la2 < 0 ? -1.f : la2 / 10.0f; }
	float leftLux() const { return lxl < 0 ? -1.f : lxl / 10.0f; }
	float rightLux() const { return lxr < 0 ? -1.f : lxr / 10.0f; }
	float multiSenseTempC() const { return tc2 < 0 ? -1.f : tc2 / 10.0f; }
	float boardTempC() const { return tin < 0 ? -1.f : tin / 10.0f; }
	float cameraFps() const { return frj < 0 ? -1.f : frj / 10.0f; }
};

// Live statistics of one stream connection.
struct StreamStats {
	std::string state = "idle"; // idle / connecting / streaming / reconnecting / stopped
	double decodedFps = 0.0;    // frames decoded per second (client side)
	double bytesPerSecond = 0.0;
	uint64_t framesDecoded = 0;
	uint64_t framesDropped = 0; // decoded but replaced before the app consumed them
	double lastDecodeMs = 0.0;  // display-decode cost of the newest frame
	long httpStatus = 0;
	std::string lastError;
};
