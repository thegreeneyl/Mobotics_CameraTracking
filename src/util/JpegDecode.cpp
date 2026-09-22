#include "JpegDecode.h"

#include <algorithm>

#include <ApplicationServices/ApplicationServices.h>

#include <opencv2/imgproc.hpp>

namespace jpegdecode {

namespace {

// Decodes the JPEG into a CGImage. reduceScale > 1 goes through the
// thumbnail path, which for JPEG decodes directly at the reduced size.
CGImageRef createImage(const uint8_t * data, size_t size, int reduceScale){
	CFDataRef cfData = CFDataCreateWithBytesNoCopy(kCFAllocatorDefault, data, size, kCFAllocatorNull);
	if(!cfData) return nullptr;
	CGImageSourceRef source = CGImageSourceCreateWithData(cfData, nullptr);
	CFRelease(cfData);
	if(!source) return nullptr;

	CGImageRef image = nullptr;
	if(reduceScale > 1){
		int width = 0, height = 0;
		CFDictionaryRef props = CGImageSourceCopyPropertiesAtIndex(source, 0, nullptr);
		if(props){
			const auto readInt = [&props](CFStringRef key, int & v){
				const auto num = static_cast<CFNumberRef>(CFDictionaryGetValue(props, key));
				if(num) CFNumberGetValue(num, kCFNumberIntType, &v);
			};
			readInt(kCGImagePropertyPixelWidth, width);
			readInt(kCGImagePropertyPixelHeight, height);
			CFRelease(props);
		}
		const int maxDim = std::max(width, height) / reduceScale;
		if(maxDim > 0){
			CFMutableDictionaryRef opts = CFDictionaryCreateMutable(
				kCFAllocatorDefault, 3,
				&kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
			CFNumberRef maxNum = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &maxDim);
			CFDictionarySetValue(opts, kCGImageSourceCreateThumbnailFromImageAlways, kCFBooleanTrue);
			CFDictionarySetValue(opts, kCGImageSourceShouldCacheImmediately, kCFBooleanTrue);
			CFDictionarySetValue(opts, kCGImageSourceThumbnailMaxPixelSize, maxNum);
			image = CGImageSourceCreateThumbnailAtIndex(source, 0, opts);
			CFRelease(maxNum);
			CFRelease(opts);
		}
	}
	if(!image){
		image = CGImageSourceCreateImageAtIndex(source, 0, nullptr);
	}
	CFRelease(source);
	return image;
}

bool renderRgba(CGImageRef image, uint8_t * dst, int w, int h, size_t bytesPerRow){
	CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
	CGContextRef ctx = CGBitmapContextCreate(dst, w, h, 8, bytesPerRow, cs,
	                                         kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
	CGColorSpaceRelease(cs);
	if(!ctx) return false;
	CGContextSetInterpolationQuality(ctx, kCGInterpolationNone);
	CGContextSetBlendMode(ctx, kCGBlendModeCopy);
	CGContextDrawImage(ctx, CGRectMake(0, 0, w, h), image);
	CGContextRelease(ctx);
	return true;
}

bool renderGray(CGImageRef image, uint8_t * dst, int w, int h, size_t bytesPerRow){
	CGColorSpaceRef cs = CGColorSpaceCreateDeviceGray();
	CGContextRef ctx = CGBitmapContextCreate(dst, w, h, 8, bytesPerRow, cs, kCGImageAlphaNone);
	CGColorSpaceRelease(cs);
	if(!ctx) return false;
	CGContextSetInterpolationQuality(ctx, kCGInterpolationNone);
	CGContextSetBlendMode(ctx, kCGBlendModeCopy);
	CGContextDrawImage(ctx, CGRectMake(0, 0, w, h), image);
	CGContextRelease(ctx);
	return true;
}

} // namespace

bool decodeToPixels(const uint8_t * data, size_t size, ofPixels & out, int reduceScale){
	CGImageRef image = createImage(data, size, reduceScale);
	if(!image) return false;
	const int w = static_cast<int>(CGImageGetWidth(image));
	const int h = static_cast<int>(CGImageGetHeight(image));
	out.allocate(w, h, OF_PIXELS_RGBA);
	const bool ok = renderRgba(image, out.getData(), w, h, static_cast<size_t>(w) * 4);
	CGImageRelease(image);
	return ok;
}

bool decodeToGrayMat(const uint8_t * data, size_t size, cv::Mat & out, int reduceScale){
	CGImageRef image = createImage(data, size, reduceScale);
	if(!image) return false;
	const int w = static_cast<int>(CGImageGetWidth(image));
	const int h = static_cast<int>(CGImageGetHeight(image));
	out.create(h, w, CV_8UC1);
	const bool ok = renderGray(image, out.data, w, h, out.step);
	CGImageRelease(image);
	return ok;
}

bool decodeToBgrMat(const uint8_t * data, size_t size, cv::Mat & out, int reduceScale){
	CGImageRef image = createImage(data, size, reduceScale);
	if(!image) return false;
	const int w = static_cast<int>(CGImageGetWidth(image));
	const int h = static_cast<int>(CGImageGetHeight(image));
	cv::Mat rgba(h, w, CV_8UC4);
	const bool ok = renderRgba(image, rgba.data, w, h, rgba.step);
	CGImageRelease(image);
	if(!ok) return false;
	cv::cvtColor(rgba, out, cv::COLOR_RGBA2BGR);
	return true;
}

} // namespace jpegdecode
