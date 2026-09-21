#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "MobotixTypes.h"

// Extracts Mobotix sensor metadata from the JPEG COM comment segments of a
// complete JPEG frame. Mobotix embeds text blocks like:
//
//   SECTION SENSORS
//   LXR=224
//   LA2=881
//   ...
//   ENDSECTION SENSORS
//
// inside standard JPEG comment (0xFFFE) segments — not in HTTP headers.
class JpegCommentParser {
public:
	// data/size must span one complete JPEG (SOI..EOI).
	// comDump (optional) receives the raw text of all COM segments.
	static SensorSnapshot parse(const uint8_t * data, size_t size, std::string * comDump = nullptr);
};
