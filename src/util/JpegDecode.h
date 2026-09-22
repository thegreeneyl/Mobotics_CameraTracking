#pragma once

#include <cstddef>
#include <cstdint>

#include <opencv2/core.hpp>

#include "ofPixels.h"

// Shared JPEG decode helper backed by Apple ImageIO (ships with macOS, the
// app already links ApplicationServices; the OF-bundled OpenCV has no JPEG
// codec and FreeImage cannot decode at reduced scale).
//
// reduceScale (1|2|4|8) uses the ImageIO thumbnail path, which decodes
// JPEGs directly at the reduced size (scaled DCT) instead of full decode +
// resize — 1/4, 1/16, 1/64 of the cost.
//
// No UI dependencies (same rule as src/mobotix/). On a future Linux port,
// swap this implementation for libjpeg-turbo; the callers stay unchanged.
namespace jpegdecode {

// Display decode -> RGBA ofPixels (alpha = 255). False on failure.
bool decodeToPixels(const uint8_t * data, size_t size, ofPixels & out, int reduceScale = 1);

// Analysis decode -> 8-bit grayscale cv::Mat (rendered directly to gray).
bool decodeToGrayMat(const uint8_t * data, size_t size, cv::Mat & out, int reduceScale = 1);

// Analysis decode -> BGR cv::Mat (for DNN detectors).
bool decodeToBgrMat(const uint8_t * data, size_t size, cv::Mat & out, int reduceScale = 1);

} // namespace jpegdecode
