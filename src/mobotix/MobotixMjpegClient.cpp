#include "MobotixMjpegClient.h"

#include <chrono>
#include <cstring>

#include <curl/curl.h>

#include "ofLog.h"

#include "../util/JpegDecode.h"

namespace {
constexpr size_t kMaxBufferBytes = 32 * 1024 * 1024; // runaway guard
constexpr double kStatWindowMs = 1000.0;
constexpr int kBackoffStartMs = 1000;
constexpr int kBackoffMaxMs = 10000;
} // namespace

// ---------------------------------------------------------------- trampolines

size_t mobotixWriteCb(char * ptr, size_t size, size_t nmemb, void * userdata){
	auto * client = static_cast<MobotixMjpegClient *>(userdata);
	const size_t total = size * nmemb;
	if(!client->running) return 0; // abort transfer

	// Drop bodies of non-200 responses (e.g. the Digest 401 challenge page).
	long status = 0;
	curl_easy_getinfo(static_cast<CURL *>(client->currentCurl), CURLINFO_RESPONSE_CODE, &status);
	{
		std::lock_guard<std::mutex> lock(client->statsMutex);
		client->stats.httpStatus = status;
	}
	if(status != 200) return total;

	client->receiveBuffer.insert(client->receiveBuffer.end(),
	                             reinterpret_cast<uint8_t *>(ptr),
	                             reinterpret_cast<uint8_t *>(ptr) + total);
	client->bytesInWindow += total;
	if(client->receiveBuffer.size() > kMaxBufferBytes){
		ofLogWarning("MobotixMjpegClient") << "receive buffer overflow, resetting";
		client->receiveBuffer.clear();
	}
	client->processBuffer();
	return total;
}

int mobotixXferCb(void * userdata, int64_t, int64_t, int64_t, int64_t){
	auto * client = static_cast<MobotixMjpegClient *>(userdata);
	return client->running ? 0 : 1; // non-zero aborts curl_easy_perform
}

// --------------------------------------------------------------------- public

MobotixMjpegClient::~MobotixMjpegClient(){
	stop();
}

void MobotixMjpegClient::start(const CameraConfig & cfg){
	stop();
	{
		std::lock_guard<std::mutex> lock(configMutex);
		config = cfg;
	}
	{
		std::lock_guard<std::mutex> lock(statsMutex);
		stats = StreamStats{};
		stats.state = "connecting";
	}
	running = true;
	streamThread = std::thread(&MobotixMjpegClient::threadFn, this);
}

void MobotixMjpegClient::stop(){
	running = false;
	if(streamThread.joinable()) streamThread.join();
	{
		std::lock_guard<std::mutex> lock(statsMutex);
		stats.state = "stopped";
	}
}

void MobotixMjpegClient::restart(const CameraConfig & cfg){
	start(cfg);
}

bool MobotixMjpegClient::getLatestFrame(ofPixels & pixels, SensorSnapshot & sensors,
                                        uint64_t & lastSeenFrameId, double & receivedAtMs,
                                        std::string * comDump){
	std::lock_guard<std::mutex> lock(frameMutex);
	if(latestFrameId == 0 || latestFrameId == lastSeenFrameId) return false;
	pixels = latestPixels;
	sensors = latestSensors;
	receivedAtMs = latestReceivedAtMs;
	if(comDump) *comDump = latestComDump;
	lastSeenFrameId = latestFrameId;
	latestConsumed = true;
	return true;
}

StreamStats MobotixMjpegClient::getStats() const {
	std::lock_guard<std::mutex> lock(statsMutex);
	return stats;
}

std::string MobotixMjpegClient::getStreamUrl() const {
	std::lock_guard<std::mutex> lock(configMutex);
	return config.buildStreamUrl();
}

void MobotixMjpegClient::setRawFrameCallback(RawFrameCallback callback){
	std::lock_guard<std::mutex> lock(rawFrameMutex);
	rawFrameCallback = std::move(callback);
}

void MobotixMjpegClient::setDecodeScale(int scale){
	if(scale >= 8) scale = 8;
	else if(scale >= 4) scale = 4;
	else if(scale >= 2) scale = 2;
	else scale = 1;
	decodeScale = scale;
}

double MobotixMjpegClient::nowMs(){
	using namespace std::chrono;
	return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

// --------------------------------------------------------------------- thread

void MobotixMjpegClient::threadFn(){
	int backoffMs = kBackoffStartMs;

	while(running){
		CameraConfig cfg;
		{
			std::lock_guard<std::mutex> lock(configMutex);
			cfg = config;
		}
		const std::string url = cfg.buildStreamUrl();
		const std::string userPwd = cfg.user + ":" + cfg.password;

		receiveBuffer.clear();
		statWindowStartMs = nowMs();
		framesInWindow = 0;
		bytesInWindow = 0;

		{
			std::lock_guard<std::mutex> lock(statsMutex);
			stats.state = "connecting";
			stats.lastError.clear();
		}

		uint64_t framesBeforeConnect = 0;
		{
			std::lock_guard<std::mutex> lock(statsMutex);
			framesBeforeConnect = stats.framesDecoded;
		}

		CURL * curl = curl_easy_init();
		if(!curl){
			std::lock_guard<std::mutex> lock(statsMutex);
			stats.lastError = "curl_easy_init failed";
			break;
		}
		currentCurl = curl;

		curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
		curl_easy_setopt(curl, CURLOPT_USERPWD, userPwd.c_str());
		curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_ANY); // Basic or Digest
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, mobotixWriteCb);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, this);
		curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
		curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, mobotixXferCb);
		curl_easy_setopt(curl, CURLOPT_XFERINFODATA, this);
		curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
		// Endless stream: no total timeout, but detect stalls.
		curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
		curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME, 10L);
		curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);
		curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

		const CURLcode rc = curl_easy_perform(curl);
		long status = 0;
		curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
		currentCurl = nullptr;
		curl_easy_cleanup(curl);

		if(!running) break;

		{
			std::lock_guard<std::mutex> lock(statsMutex);
			// A connection that delivered frames resets the backoff.
			if(stats.framesDecoded > framesBeforeConnect) backoffMs = kBackoffStartMs;
			stats.state = "reconnecting";
			stats.httpStatus = status;
			if(rc != CURLE_OK && rc != CURLE_ABORTED_BY_CALLBACK){
				stats.lastError = curl_easy_strerror(rc);
			}else if(status != 200 && status != 0){
				stats.lastError = "HTTP " + std::to_string(status);
			}
		}
		ofLogNotice("MobotixMjpegClient")
			<< "stream ended (" << curl_easy_strerror(rc) << ", HTTP " << status
			<< "), reconnecting in " << backoffMs << " ms";

		// interruptible backoff
		for(int waited = 0; waited < backoffMs && running; waited += 100){
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		backoffMs = std::min(backoffMs * 2, kBackoffMaxMs);
	}
}

// -------------------------------------------------------------------- framing

bool MobotixMjpegClient::findCompleteJpeg(size_t & start, size_t & end, bool & needMore) const {
	needMore = false;
	const uint8_t * data = receiveBuffer.data();
	const size_t size = receiveBuffer.size();

	// Find SOI (FF D8 FF — require a marker byte after SOI to avoid random hits)
	size_t soi = std::string::npos;
	for(size_t i = 0; i + 2 < size; i++){
		if(data[i] == 0xFF && data[i + 1] == 0xD8 && data[i + 2] == 0xFF){
			soi = i;
			break;
		}
	}
	if(soi == std::string::npos){
		needMore = true;
		return false;
	}

	// Walk marker segments until SOS, then scan entropy-coded data for EOI.
	size_t i = soi + 2;
	while(true){
		if(i + 4 > size){ needMore = true; return false; }
		if(data[i] != 0xFF){
			// corrupt — resync past this SOI
			start = soi + 2;
			end = soi + 2;
			return false;
		}
		const uint8_t marker = data[i + 1];
		if(marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)){
			i += 2;
			continue;
		}
		if(marker == 0xDA){ // SOS — entropy data follows
			const size_t segLen = (static_cast<size_t>(data[i + 2]) << 8) | data[i + 3];
			i += 2 + segLen;
			// scan for a real marker (FF followed by non-zero, non-RST)
			while(true){
				if(i + 1 >= size){ needMore = true; return false; }
				if(data[i] != 0xFF){ i++; continue; }
				const uint8_t m = data[i + 1];
				if(m == 0x00 || (m >= 0xD0 && m <= 0xD7)){ i += 2; continue; }
				if(m == 0xD9){ // EOI
					start = soi;
					end = i + 2;
					return true;
				}
				// another header segment mid-scan (rare) — go back to header walk
				break;
			}
			continue;
		}
		// regular segment with length
		const size_t segLen = (static_cast<size_t>(data[i + 2]) << 8) | data[i + 3];
		if(segLen < 2){
			start = soi + 2;
			end = soi + 2;
			return false;
		}
		if(i + 2 + segLen > size){ needMore = true; return false; }
		i += 2 + segLen;
	}
}

void MobotixMjpegClient::processBuffer(){
	while(running){
		size_t start = 0, end = 0;
		bool needMore = false;
		const bool found = findCompleteJpeg(start, end, needMore);
		if(needMore) break;
		if(!found){
			// resync: drop the unusable prefix and try again
			if(end > 0){
				receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + end);
				continue;
			}
			break;
		}
		handleCompleteJpeg(receiveBuffer.data() + start, end - start);
		receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + end);
	}
}

void MobotixMjpegClient::handleCompleteJpeg(const uint8_t * data, size_t size){
	{
		std::lock_guard<std::mutex> lock(rawFrameMutex);
		if(rawFrameCallback) rawFrameCallback(data, size, nowMs());
	}

	std::string comDump;
	SensorSnapshot sensors = JpegCommentParser::parse(data, size, &comDump);

	const double decodeStartMs = nowMs();
	ofPixels pixels;
	if(!jpegdecode::decodeToPixels(data, size, pixels, decodeScale)){
		ofLogWarning("MobotixMjpegClient") << "failed to decode JPEG frame (" << size << " bytes)";
		return;
	}
	const double decodeMs = nowMs() - decodeStartMs;

	const double now = nowMs();
	{
		std::lock_guard<std::mutex> lock(frameMutex);
		if(!latestConsumed){
			std::lock_guard<std::mutex> statsLock(statsMutex);
			stats.framesDropped++;
		}
		latestPixels = std::move(pixels);
		latestSensors = sensors;
		latestComDump = std::move(comDump);
		latestFrameId++;
		latestReceivedAtMs = now;
		latestConsumed = false;
	}

	framesInWindow++;
	{
		std::lock_guard<std::mutex> lock(statsMutex);
		stats.framesDecoded++;
		stats.lastDecodeMs = decodeMs;
		stats.state = "streaming";
		const double windowMs = now - statWindowStartMs;
		if(windowMs >= kStatWindowMs){
			stats.decodedFps = framesInWindow * 1000.0 / windowMs;
			stats.bytesPerSecond = bytesInWindow * 1000.0 / windowMs;
			statWindowStartMs = now;
			framesInWindow = 0;
			bytesInWindow = 0;
		}
	}
}
