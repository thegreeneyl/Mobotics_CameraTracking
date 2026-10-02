#include "BatchRunner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include <opencv2/imgproc.hpp>

#include "ofFileUtils.h"
#include "ofImage.h"
#include "ofJson.h"
#include "ofLog.h"
#include "ofUtils.h"

#include "../output/MotionPathWriter.h"
#include "../recording/RecordingReader.h"
#include "../tracking/TrackingConfigJson.h"
#include "../tracking/TrackingManager.h"
#include "../util/JpegDecode.h"
#include "../util/LensCorrector.h"

namespace batch {

namespace {

double nowMs(){
	using namespace std::chrono;
	return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

double wallClockSec(){
	using namespace std::chrono;
	return duration<double>(system_clock::now().time_since_epoch()).count();
}

std::vector<std::string> splitCsv(const std::string & s){
	std::vector<std::string> out;
	std::stringstream ss(s);
	std::string item;
	while(std::getline(ss, item, ',')){
		item.erase(0, item.find_first_not_of(" \t"));
		item.erase(item.find_last_not_of(" \t") + 1);
		if(!item.empty()) out.push_back(item);
	}
	return out;
}

// ------------------------------------------------------------ contact sheet

cv::Scalar colorForId(int id){
	const float h = std::fmod(static_cast<float>(id) * 0.61803398875f, 1.0f) * 6.0f;
	const float x = 1.0f - std::fabs(std::fmod(h, 2.0f) - 1.0f);
	float r = 0, g = 0, b = 0;
	switch(static_cast<int>(h)){
		case 0: r = 1; g = x; b = 0; break;
		case 1: r = x; g = 1; b = 0; break;
		case 2: r = 0; g = 1; b = x; break;
		case 3: r = 0; g = x; b = 1; break;
		case 4: r = x; g = 0; b = 1; break;
		default: r = 1; g = 0; b = x; break;
	}
	return cv::Scalar(b * 255, g * 255, r * 255);
}

cv::Point2f project(const cv::Mat & h, float x, float y){
	const double * m = h.ptr<double>();
	const double w = m[6] * x + m[7] * y + m[8];
	return {static_cast<float>((m[0] * x + m[1] * y + m[2]) / w),
	        static_cast<float>((m[3] * x + m[4] * y + m[5]) / w)};
}

// Draws the tracked objects of one snapshot onto a BGR thumbnail of the
// frame. Corridor coordinates are mapped back into each lane and projected
// onto the photo through the lane quad. Raw detections are drawn thin.
void drawOverlay(cv::Mat & img, const tracking::TrackingResults & res, double tSec){
	const int paneCount = std::max(1, res.paneCount);
	const float paneW = static_cast<float>(img.cols) / paneCount;
	const float paneH = static_cast<float>(img.rows);

	std::vector<cv::Mat> laneH(res.lanes.size());
	for(size_t li = 0; li < res.lanes.size(); li++){
		const tracking::LaneConfig & lane = res.lanes[li].lane;
		const tracking::FrameQuad quad = lane.quad.isConvex() ? lane.quad : tracking::FrameQuad{};
		const cv::Point2f src[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
		cv::Point2f dst[4];
		for(int c = 0; c < 4; c++){
			dst[c] = cv::Point2f(lane.pane * paneW + quad.pts[c][0] * paneW, quad.pts[c][1] * paneH);
		}
		laneH[li] = cv::getPerspectiveTransform(src, dst);
		for(int c = 0; c < 4; c++){
			cv::line(img, dst[c], dst[(c + 1) % 4], cv::Scalar(60, 210, 255), 1, cv::LINE_AA);
		}
		cv::putText(img, lane.id, dst[0] + cv::Point2f(2, 10), cv::FONT_HERSHEY_SIMPLEX, 0.35,
		            cv::Scalar(60, 210, 255), 1, cv::LINE_AA);
		for(const auto & d : res.lanes[li].detections){
			const cv::Point2f p[4] = {project(laneH[li], d.x0, d.y0), project(laneH[li], d.x1, d.y0),
			                          project(laneH[li], d.x1, d.y1), project(laneH[li], d.x0, d.y1)};
			for(int c = 0; c < 4; c++) cv::line(img, p[c], p[(c + 1) % 4], cv::Scalar(160, 160, 160), 1, cv::LINE_AA);
		}
	}

	for(const auto & cr : res.corridors){
		for(const auto & o : cr.objects){
			const cv::Scalar col = colorForId(o.id);
			const int thick = o.confirmed ? 2 : 1;
			bool labelled = false;
			for(int li : cr.lanes){
				if(li < 0 || li >= static_cast<int>(res.lanes.size())) continue;
				const tracking::LaneConfig & lane = res.lanes[li].lane;
				float lx0, lx1;
				if(!lane.corridorRangeToLane(o.x0, o.x1, lx0, lx1)) continue;
				const cv::Mat & h = laneH[li];
				// silhouette outline (corridor coords -> this lane -> photo)
				if(o.shape.valid && o.shape.outline.size() >= 3){
					std::vector<cv::Point> poly;
					for(const auto & q : o.shape.outline){
						const float lx = lane.toLane(q.first);
						if(lx < -0.001f || lx > 1.001f) continue;
						const cv::Point2f pp = project(h, lx, q.second);
						poly.emplace_back(static_cast<int>(std::lround(pp.x)), static_cast<int>(std::lround(pp.y)));
					}
					if(poly.size() >= 3){
						cv::Mat fill = img.clone();
						cv::fillPoly(fill, std::vector<std::vector<cv::Point>>{poly}, col);
						cv::addWeighted(img, 0.75, fill, 0.25, 0, img);
						cv::polylines(img, poly, true, col, thick, cv::LINE_AA);
					}
				}
				const cv::Point2f p[4] = {project(h, lx0, o.y0), project(h, lx1, o.y0),
				                          project(h, lx1, o.y1), project(h, lx0, o.y1)};
				for(int c = 0; c < 4; c++) cv::line(img, p[c], p[(c + 1) % 4], col, thick, cv::LINE_AA);
				// clipped edge markers
				if(o.clippedLeft && lane.corridorRangeToLane(0.0f, 0.001f, lx0, lx1)){
					cv::line(img, p[0], p[3], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
				}
				if(o.clippedRight && lane.corridorRangeToLane(0.999f, 1.0f, lx0, lx1)){
					cv::line(img, p[1], p[2], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
				}
				// center + velocity arrow in lane space
				const float lcx = lane.toLane(o.x);
				if(lcx >= 0 && lcx <= 1){
					const cv::Point2f c = project(h, lcx, o.y);
					cv::circle(img, c, 3, col, -1, cv::LINE_AA);
					const float span = lane.s1 - lane.s0;
					float dx = std::fabs(span) > 1e-6f ? o.vx / span : o.vx, dy = o.vy;
					const float len = std::hypot(dx, dy);
					if(len > 0.25f){ dx *= 0.25f / len; dy *= 0.25f / len; }
					const cv::Point2f tip = project(h, lcx + dx, o.y + dy);
					cv::arrowedLine(img, c, tip, col, thick, cv::LINE_AA, 0, 0.2);
				}
				if(!labelled){
					std::string text = "#" + std::to_string(o.id);
					if(!o.label.empty()) text += " " + o.label;
					text += " " + o.phase;
					if(!o.confirmed) text += "?";
					cv::putText(img, text, cv::Point(static_cast<int>(p[0].x), static_cast<int>(p[0].y) - 4),
					            cv::FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv::LINE_AA);
					labelled = true;
				}
			}
		}
	}
	char buf[64];
	std::snprintf(buf, sizeof(buf), "%.1fs", tSec);
	cv::rectangle(img, cv::Rect(0, 0, 70, 18), cv::Scalar(0, 0, 0), -1);
	cv::putText(img, buf, cv::Point(4, 13), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}

// Detector's-eye view for one snapshot: per lane the rectified strip (as the
// detector sees it, at mask resolution) with the raw detections drawn, and
// the fused motion mask below it. Lanes are stacked vertically.
cv::Mat stripSheet(const cv::Mat & frame, const tracking::TrackingResults & res){
	const int paneCount = std::max(1, res.paneCount);
	const float paneW = static_cast<float>(frame.cols) / paneCount;
	const float paneH = static_cast<float>(frame.rows);
	std::vector<cv::Mat> rows;
	int width = 0;
	for(const auto & ld : res.lanes){
		if(ld.maskW <= 0 || ld.maskH <= 0) continue;
		const tracking::FrameQuad quad = ld.lane.quad.isConvex() ? ld.lane.quad : tracking::FrameQuad{};
		cv::Point2f src[4], dst[4] = {{0, 0}, {static_cast<float>(ld.maskW), 0},
		                              {static_cast<float>(ld.maskW), static_cast<float>(ld.maskH)},
		                              {0, static_cast<float>(ld.maskH)}};
		for(int c = 0; c < 4; c++){
			src[c] = cv::Point2f(ld.lane.pane * paneW + quad.pts[c][0] * paneW, quad.pts[c][1] * paneH);
		}
		cv::Mat strip;
		cv::warpPerspective(frame, strip, cv::getPerspectiveTransform(src, dst),
		                    cv::Size(ld.maskW, ld.maskH), cv::INTER_LINEAR);
		cv::Mat mask(ld.maskH, ld.maskW, CV_8U, const_cast<uint8_t *>(ld.mask.data()));
		cv::Mat maskBgr;
		cv::cvtColor(mask, maskBgr, cv::COLOR_GRAY2BGR);
		for(const auto & d : ld.detections){
			const cv::Rect r(cv::Point(d.x0 * ld.maskW, d.y0 * ld.maskH), cv::Point(d.x1 * ld.maskW, d.y1 * ld.maskH));
			cv::rectangle(strip, r, cv::Scalar(0, 255, 255), 1);
			cv::rectangle(maskBgr, r, cv::Scalar(0, 255, 255), 1);
			const cv::Point c = (r.tl() + r.br()) / 2;
			cv::arrowedLine(strip, c, c + cv::Point(d.vx * ld.maskW, d.vy * ld.maskH), cv::Scalar(0, 255, 0), 1);
			cv::putText(strip, "v" + ofToString(d.vx, 3) + " coh" + ofToString(d.coherence, 2),
			            r.tl() + cv::Point(2, -3), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1);
		}
		// tracked objects' outlines in lane space (on the strip)
		for(const auto & cr : res.corridors){
			const int laneIdx = static_cast<int>(&ld - res.lanes.data());
			if(std::find(cr.lanes.begin(), cr.lanes.end(), laneIdx) == cr.lanes.end()) continue;
			for(const auto & o : cr.objects){
				if(!o.shape.valid || o.shape.outline.size() < 3) continue;
				std::vector<cv::Point> poly;
				for(const auto & q : o.shape.outline){
					const float lx = ld.lane.toLane(q.first);
					if(lx < -0.001f || lx > 1.001f) continue;
					poly.emplace_back(static_cast<int>(std::lround(lx * ld.maskW)),
					                  static_cast<int>(std::lround(q.second * ld.maskH)));
				}
				if(poly.size() >= 3) cv::polylines(strip, poly, true, colorForId(o.id), o.confirmed ? 2 : 1, cv::LINE_AA);
			}
		}
		cv::putText(strip, ld.lane.id, cv::Point(3, 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(60, 210, 255), 1);
		rows.push_back(strip);
		rows.push_back(maskBgr);
		width = std::max(width, ld.maskW);
	}
	if(rows.empty()) return cv::Mat();
	int height = 0;
	for(const auto & r : rows) height += r.rows + 2;
	cv::Mat sheet(height, width, CV_8UC3, cv::Scalar(40, 40, 40));
	int y = 0;
	for(const auto & r : rows){
		r.copyTo(sheet(cv::Rect(0, y, r.cols, r.rows)));
		y += r.rows + 2;
	}
	return sheet;
}

bool saveBgr(const cv::Mat & bgr, const std::string & path){
	cv::Mat rgb;
	cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
	ofPixels pixels;
	pixels.setFromPixels(rgb.data, rgb.cols, rgb.rows, OF_PIXELS_RGB);
	return ofSaveImage(pixels, path, OF_IMAGE_QUALITY_HIGH);
}

cv::Mat tile(const std::vector<cv::Mat> & thumbs, int cols){
	if(thumbs.empty()) return cv::Mat();
	const int w = thumbs[0].cols, h = thumbs[0].rows;
	const int rows = (static_cast<int>(thumbs.size()) + cols - 1) / cols;
	cv::Mat sheet(rows * h, cols * w, CV_8UC3, cv::Scalar(20, 20, 20));
	for(size_t i = 0; i < thumbs.size(); i++){
		const int r = static_cast<int>(i) / cols, c = static_cast<int>(i) % cols;
		if(thumbs[i].cols == w && thumbs[i].rows == h){
			thumbs[i].copyTo(sheet(cv::Rect(c * w, r * h, w, h)));
		}
	}
	return sheet;
}

// ------------------------------------------------------------ one recording

int runRecording(const Options & opts, const tracking::LoadedTrackingConfig & loaded,
                 const std::string & mjpegPath){
	RecordingReader reader;
	if(!reader.open(mjpegPath)) return 2;

	const size_t n = reader.frameCount();
	const double durationSec = reader.durationMs() / 1000.0;
	const double fps = durationSec > 0 ? (n - 1) / durationSec : 30.0;
	ofLogNotice("batch") << reader.name() << ": " << n << " frames, " << ofToString(durationSec, 1)
		<< " s, " << ofToString(fps, 1) << " fps";

	tracking::TrackingManager manager;
	tracking::TrackingConfig cfg = loaded.config;
	cfg.enabled = true;
	cfg.sourceName = reader.name();
	manager.setupSync(cfg);
	LensCorrector sheetLens;
	sheetLens.set(cfg.lens);

	std::unique_ptr<output::MotionPathRecording> recording; // created on the first frame (size)
	std::vector<cv::Mat> thumbs;
	int thumbCols = 4;
	double nextThumbSec = opts.startSec;
	double nextStripSec = opts.startSec;
	const std::string stripDir = ofFilePath::join(opts.outDir, reader.name() + ".strips");

	std::ofstream debug;
	if(opts.debugDump){
		ofDirectory::createDirectory(opts.outDir, false, true);
		debug.open(ofFilePath::join(opts.outDir, reader.name() + ".debug.jsonl"));
	}
	const auto r4 = [](double v){ return std::round(v * 10000.0) / 10000.0; };

	std::vector<uint8_t> jpeg;
	tracking::TrackingResults res;
	double sumDecode = 0, sumDetect = 0, sumTrack = 0, sumShape = 0;
	size_t processed = 0, failed = 0;
	const double t0 = nowMs();
	size_t nextProgress = n / 10;

	for(size_t i = 0; i < n; i += std::max(1, opts.stride)){
		const double tSec = reader.frame(i).tMs / 1000.0;
		if(tSec < opts.startSec) continue;
		if(opts.endSec >= 0 && tSec > opts.endSec) break;
		if(!reader.read(i, jpeg)){ failed++; continue; }
		if(!manager.processFrame(jpeg.data(), jpeg.size(), reader.frame(i).tMs, res)){ failed++; continue; }
		processed++;
		sumDecode += res.decodeMs;
		sumDetect += res.detectMs;
		sumTrack += res.trackMs;
		sumShape += res.shapeMs;

		if(!recording){
			thumbCols = res.paneCount >= 2 ? 2 : 4;
			// published space is the output canvas (tracking.canvas, the
			// facade's Lines area proportions)
			recording = std::make_unique<output::MotionPathRecording>(reader.name(), fps,
			                                                          res.canvas.width, res.canvas.height);
			ofLogNotice("batch") << "  layout '" << res.layoutName << "', " << res.lanes.size()
				<< " lane(s), " << res.corridors.size() << " corridor(s), detector " << res.detectorName;
		}
		recording->addFrame(output::fromTrackingResults(res), tSec, wallClockSec());

		if(debug.is_open()){
			ofJson lanes = ofJson::array();
			for(const auto & ld : res.lanes){
				ofJson dets = ofJson::array();
				for(const auto & d : ld.detections){
					dets.push_back({r4(d.x0), r4(d.x1), r4(d.y0), r4(d.y1), r4(d.vx), r4(d.vy),
					                r4(d.coherence), r4(d.area), d.touchesLeft ? 1 : 0, d.touchesRight ? 1 : 0});
				}
				lanes.push_back({{"id", ld.lane.id}, {"dets", dets}});
			}
			ofJson tracks = ofJson::array();
			for(const auto & cr : res.corridors){
				for(const auto & o : cr.objects){
					tracks.push_back({{"id", o.id}, {"c", cr.id}, {"x0", r4(o.x0)}, {"x1", r4(o.x1)},
					                  {"y0", r4(o.y0)}, {"y1", r4(o.y1)}, {"vx", r4(o.vx)}, {"vy", r4(o.vy)},
					                  {"conf", o.confirmed}, {"ph", o.phase}, {"hits", o.hits}, {"miss", o.misses},
					                  {"cl", o.clippedLeft}, {"cr", o.clippedRight}});
				}
			}
			debug << ofJson{{"t", r4(tSec)}, {"lanes", lanes}, {"tracks", tracks}}.dump() << "\n";
		}

		if(opts.sheetIntervalSec > 0 && tSec >= nextThumbSec){
			nextThumbSec += opts.sheetIntervalSec;
			cv::Mat bgr;
			if(jpegdecode::decodeToBgrMat(jpeg.data(), jpeg.size(), bgr, 4)){
				cv::Mat corrected;
				sheetLens.apply(bgr, corrected);
				drawOverlay(corrected, res, tSec);
				bgr = corrected;
				thumbs.push_back(bgr);
			}
		}
		if(opts.stripIntervalSec > 0 && tSec >= nextStripSec){
			nextStripSec += opts.stripIntervalSec;
			cv::Mat bgr;
			if(jpegdecode::decodeToBgrMat(jpeg.data(), jpeg.size(), bgr, 2)){
				cv::Mat corrected;
				sheetLens.apply(bgr, corrected);
				ofDirectory::createDirectory(stripDir, false, true);
				saveBgr(stripSheet(corrected, res), ofFilePath::join(stripDir,
					reader.name() + "_" + ofToString(tSec, 1, 6, '0') + ".jpg"));
			}
		}
		if(i >= nextProgress){
			nextProgress += n / 10;
			ofLogNotice("batch") << "  " << (100 * i / n) << "%  t=" << ofToString(tSec, 1)
				<< "s  " << ofToString(processed / ((nowMs() - t0) / 1000.0), 0) << " fps";
		}
	}
	const double elapsedSec = (nowMs() - t0) / 1000.0;

	if(!recording){
		ofLogError("batch") << reader.name() << ": no frame could be processed";
		return 3;
	}
	ofDirectory::createDirectory(opts.outDir, false, true);
	const std::string base = ofFilePath::join(opts.outDir, reader.name());
	if(!recording->save(base + ".motion.json", "Batch run of " + reader.name())){
		ofLogError("batch") << "failed to write " << base << ".motion.json";
		return 4;
	}
	if(!thumbs.empty()){
		saveBgr(tile(thumbs, thumbCols), base + ".sheet.jpg");
	}
	ofJson summary = {
		{"recording", reader.name()},
		{"frames_total", n},
		{"frames_processed", processed},
		{"frames_failed", failed},
		{"duration_s", durationSec},
		{"fps_recording", fps},
		{"elapsed_s", elapsedSec},
		{"fps_processing", processed / std::max(elapsedSec, 1e-6)},
		{"avg_decode_ms", processed ? sumDecode / processed : 0.0},
		{"avg_detect_ms", processed ? sumDetect / processed : 0.0},
		{"avg_track_ms", processed ? sumTrack / processed : 0.0},
		{"avg_shape_ms", processed ? sumShape / processed : 0.0},
		{"detector", res.detectorName},
		{"layout", res.layoutName},
		{"analysis_size", {res.analysisW, res.analysisH}},
	};
	std::ofstream(base + ".summary.json") << summary.dump(2);
	ofLogNotice("batch") << reader.name() << ": " << processed << " frames in " << ofToString(elapsedSec, 1)
		<< " s (" << ofToString(processed / std::max(elapsedSec, 1e-6), 0) << " fps), avg dec "
		<< ofToString(summary["avg_decode_ms"].get<double>(), 1) << " det "
		<< ofToString(summary["avg_detect_ms"].get<double>(), 1) << " trk "
		<< ofToString(summary["avg_track_ms"].get<double>(), 1) << " ms -> " << base << ".*";
	return 0;
}

} // namespace

bool optionsFromEnv(Options & opts){
	const char * batchEnv = std::getenv("CAMTRACK_BATCH");
	if(!batchEnv || !*batchEnv) return false;
	const std::string sel = batchEnv;
	if(sel != "all") opts.recordings = splitCsv(sel);
	if(const char * v = std::getenv("CAMTRACK_OUT")) opts.outDir = v;
	if(const char * v = std::getenv("CAMTRACK_CONFIG")) opts.configPath = v;
	if(const char * v = std::getenv("CAMTRACK_RECORDINGS")) opts.recordingsDir = v;
	if(const char * v = std::getenv("CAMTRACK_SHEET_SEC")) opts.sheetIntervalSec = std::atof(v);
	if(const char * v = std::getenv("CAMTRACK_STRIDE")) opts.stride = std::max(1, std::atoi(v));
	if(const char * v = std::getenv("CAMTRACK_DEBUG")) opts.debugDump = std::atoi(v) != 0;
	if(const char * v = std::getenv("CAMTRACK_STRIPS")) opts.stripIntervalSec = std::atof(v);
	if(const char * v = std::getenv("CAMTRACK_RANGE")){
		const std::string r = v;
		const auto dash = r.find('-');
		if(dash != std::string::npos){
			opts.startSec = std::atof(r.substr(0, dash).c_str());
			opts.endSec = std::atof(r.substr(dash + 1).c_str());
		}
	}
	return true;
}

int run(const Options & optsIn){
	Options opts = optsIn;
	if(opts.configPath.empty()){
		opts.configPath = ofToDataPath("config/config.json", true);
		if(!ofFile::doesFileExist(opts.configPath)){
			opts.configPath = ofToDataPath("config/config.example.json", true);
		}
	}
	ofJson root;
	try {
		root = ofLoadJson(opts.configPath);
	} catch(const std::exception & e){
		ofLogError("batch") << "cannot load " << opts.configPath << ": " << e.what();
		return 1;
	}
	const auto loaded = tracking::loadTrackingConfig(root, [](const std::string & p){ return ofToDataPath(p, true); });
	if(opts.recordingsDir.empty()) opts.recordingsDir = ofToDataPath(loaded.recordingsDir, true);
	if(opts.outDir.empty()) opts.outDir = ofToDataPath("eval", true);

	ofDirectory dir(opts.recordingsDir);
	if(!dir.exists()){
		ofLogError("batch") << "recordings dir missing: " << opts.recordingsDir;
		return 1;
	}
	dir.allowExt("mjpeg");
	dir.listDir();
	std::vector<std::string> paths;
	for(size_t i = 0; i < dir.size(); i++){
		const std::string path = dir.getPath(i);
		const std::string base = ofFilePath::getBaseName(path);
		if(!opts.recordings.empty()
		   && std::find(opts.recordings.begin(), opts.recordings.end(), base) == opts.recordings.end()){
			continue;
		}
		paths.push_back(path);
	}
	std::sort(paths.begin(), paths.end());
	if(paths.empty()){
		ofLogError("batch") << "no matching recordings in " << opts.recordingsDir;
		return 1;
	}
	ofLogNotice("batch") << "config " << opts.configPath << ", " << paths.size()
		<< " recording(s), out " << opts.outDir;

	int rc = 0;
	for(const auto & p : paths){
		const int r = runRecording(opts, loaded, p);
		if(r != 0) rc = r;
	}
	return rc;
}

} // namespace batch
