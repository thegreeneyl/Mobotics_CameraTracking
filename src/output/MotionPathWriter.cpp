#include "MotionPathWriter.h"

#include <algorithm>
#include <cmath>
#include <fstream>

namespace output {

namespace {
constexpr const char * kSchema = "motion-path.v1";
constexpr int kMaxPathPoints = 80;

double round4(double v){
	return std::round(v * 10000.0) / 10000.0;
}

double headingDeg(double vx, double vy){
	double deg = std::atan2(vy, vx) * 180.0 / M_PI;
	if(deg < 0) deg += 360.0;
	return deg;
}

std::string regionLabel(float x, float y){
	const std::string col = x < 1.0f / 3 ? "left" : x > 2.0f / 3 ? "right" : "center";
	const std::string row = y < 1.0f / 3 ? "top" : y > 2.0f / 3 ? "bottom" : "mid";
	return col + "-" + row;
}
} // namespace

std::vector<MotionObject> fromTrackingResults(const tracking::TrackingResults & res){
	std::vector<MotionObject> out;
	for(const auto & cr : res.corridors){
		const tracking::CorridorPlacement & pl = cr.placement;
		for(const auto & o : cr.objects){
			// corridor box -> canvas box, cut at the canvas edges
			float cx0, cx1, cy0, cy1;
			pl.mapBox(o.x0, o.x1, o.y0, o.y1, cx0, cx1, cy0, cy1);
			const bool offLeft = cx0 < 0.0f, offRight = cx1 > 1.0f;
			if(!tracking::CorridorPlacement::clipToCanvas(cx0, cx1, cy0, cy1)) continue;

			MotionObject mo;
			mo.id = o.id;
			mo.view = cr.id;
			mo.label = o.label;
			mo.x = (cx0 + cx1) * 0.5f;
			mo.y = (cy0 + cy1) * 0.5f;
			mo.w = cx1 - cx0;
			mo.h = cy1 - cy0;
			mo.vx = o.vx * pl.spanX(); // a mirrored placement flips the sign
			mo.vy = o.vy * pl.spanY();
			mo.phase = o.phase;
			mo.confirmed = o.confirmed;
			// cut on the canvas side, either by a corridor end (mapped
			// through the mirror) or by the canvas edge
			const bool corrLeft = pl.mirrored() ? o.clippedRight : o.clippedLeft;
			const bool corrRight = pl.mirrored() ? o.clippedLeft : o.clippedRight;
			mo.clippedLeft = corrLeft || offLeft;
			mo.clippedRight = corrRight || offRight;
			mo.path.reserve(o.trail.size());
			for(const auto & p : o.trail){
				mo.path.emplace_back(pl.toCanvasX(p.first), pl.toCanvasY(p.second));
			}
			mo.corridor.x = o.x;
			mo.corridor.y = o.y;
			mo.corridor.w = o.w;
			mo.corridor.h = o.h;
			mo.corridor.vx = o.vx;
			mo.corridor.vy = o.vy;
			// Back-project onto each lane of the corridor. The lane quad is
			// a perspective map; we approximate the box by the bilinear
			// image of its lane-space corners' bounding box (exact for
			// rectangular quads, close enough for the overlay).
			for(int li : cr.lanes){
				if(li < 0 || li >= static_cast<int>(res.lanes.size())) continue;
				const tracking::LaneConfig & lane = res.lanes[li].lane;
				float lx0, lx1;
				if(!lane.corridorRangeToLane(o.x0, o.x1, lx0, lx1)) continue;
				const auto & q = lane.quad.pts;
				const auto map = [&](float u, float v, float & px, float & py){
					const float tx = q[0][0] + (q[1][0] - q[0][0]) * u;
					const float ty = q[0][1] + (q[1][1] - q[0][1]) * u;
					const float bx = q[3][0] + (q[2][0] - q[3][0]) * u;
					const float by = q[3][1] + (q[2][1] - q[3][1]) * u;
					px = tx + (bx - tx) * v;
					py = ty + (by - ty) * v;
				};
				float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
				const float us[2] = {lx0, lx1}, vs[2] = {o.y0, o.y1};
				for(float u : us) for(float v : vs){
					float px, py;
					map(u, v, px, py);
					minX = std::min(minX, px); maxX = std::max(maxX, px);
					minY = std::min(minY, py); maxY = std::max(maxY, py);
				}
				MotionObject::PaneBox pb;
				pb.pane = lane.pane;
				pb.x = minX; pb.y = minY; pb.w = maxX - minX; pb.h = maxY - minY;
				mo.panes.push_back(pb);
			}
			out.push_back(std::move(mo));
		}
	}
	return out;
}

std::string headingLabel(double degrees){
	static const char * kLabels[8] = {
		"right", "down-right", "down", "down-left", "left", "up-left", "up", "up-right"};
	const int idx = static_cast<int>(std::floor((degrees + 22.5) / 45.0)) % 8;
	return kLabels[(idx + 8) % 8];
}

ofJson buildFrame(const std::vector<MotionObject> & objects, double tSec, int frame,
                  double fps, const std::string & camera, int sizeW, int sizeH,
                  double wallClockSec){
	ofJson objs = ofJson::array();
	double sumVx = 0, sumVy = 0;
	int count = 0;
	for(const auto & o : objects){
		if(!o.confirmed) continue;
		const double speed = std::hypot(o.vx, o.vy);
		const double heading = headingDeg(o.vx, o.vy);
		ofJson path = ofJson::array();
		const size_t start = o.path.size() > kMaxPathPoints ? o.path.size() - kMaxPathPoints : 0;
		for(size_t i = start; i < o.path.size(); i++){
			path.push_back({round4(o.path[i].first), round4(o.path[i].second)});
		}
		ofJson panes = ofJson::array();
		for(const auto & p : o.panes){
			panes.push_back({{"pane", p.pane},
			                 {"bbox", {round4(p.x), round4(p.y), round4(p.w), round4(p.h)}}});
		}
		ofJson j = {
			{"id", o.id},
			{"view", o.view},
			{"x", round4(o.x)},
			{"y", round4(o.y)},
			{"vx", round4(o.vx)},
			{"vy", round4(o.vy)},
			{"speed", round4(speed)},
			{"heading_deg", std::round(heading * 10.0) / 10.0},
			{"heading", speed > 0.003 ? headingLabel(heading) : "still"},
			{"phase", o.phase},
			{"fill", std::round(o.w * o.h * 1000.0) / 1000.0},
			{"class", o.label.empty() ? ofJson(nullptr) : ofJson(o.label)},
			{"region", regionLabel(o.x, o.y)},
			{"bbox", {round4(o.x - o.w / 2), round4(o.y - o.h / 2), round4(o.w), round4(o.h)}},
			{"clipped", {{"left", o.clippedLeft}, {"right", o.clippedRight}}},
			{"corridor", {{"x", round4(o.corridor.x)}, {"y", round4(o.corridor.y)},
			              {"w", round4(o.corridor.w)}, {"h", round4(o.corridor.h)},
			              {"vx", round4(o.corridor.vx)}, {"vy", round4(o.corridor.vy)}}},
			{"panes", panes},
			{"path", path},
		};
		objs.push_back(std::move(j));
		sumVx += o.vx;
		sumVy += o.vy;
		count++;
	}
	const double sceneVx = count ? sumVx / count : 0.0;
	const double sceneVy = count ? sumVy / count : 0.0;
	const double sceneSpeed = std::hypot(sceneVx, sceneVy);
	const double sceneHeading = headingDeg(sceneVx, sceneVy);

	return ofJson{
		{"schema", kSchema},
		{"ts", std::round(wallClockSec * 1000.0) / 1000.0},
		{"t", round4(tSec)},
		{"frame", frame},
		{"fps", std::round(fps * 100.0) / 100.0},
		{"camera", camera},
		{"size", {sizeW, sizeH}},
		{"scene", {
			{"vx", round4(sceneVx)},
			{"vy", round4(sceneVy)},
			{"speed", round4(sceneSpeed)},
			{"heading_deg", std::round(sceneHeading * 10.0) / 10.0},
			{"heading", sceneSpeed > 0.002 ? headingLabel(sceneHeading) : "still"},
			{"count", count}}},
		{"objects", objs},
	};
}

MotionPathRecording::MotionPathRecording(std::string src, double f, int w, int h)
	: source(std::move(src)), fps(f), sizeW(w), sizeH(h){}

void MotionPathRecording::addFrame(const std::vector<MotionObject> & objects, double tSec,
                                   double wallClockSec){
	frames.push_back(buildFrame(objects, tSec, static_cast<int>(frames.size()) + 1, fps, source,
	                            sizeW, sizeH, wallClockSec));
	lastTSec = tSec;
}

bool MotionPathRecording::save(const std::string & path, const std::string & note) const {
	ofJson doc = {
		{"schema", kSchema},
		{"kind", "recording"},
		{"source", source},
		{"fps", std::round(fps * 100.0) / 100.0},
		{"frame_count", frames.size()},
		{"duration_s", frames.empty() ? 0.0 : round4(lastTSec + 1.0 / std::max(fps, 1.0))},
		{"size", {sizeW, sizeH}},
		{"live", {{"transport", "udp"}, {"host", "127.0.0.1"}, {"port", 8765},
		          {"note", "In production each frames[] item is one UDP datagram."}}},
		{"frames", frames},
	};
	if(!note.empty()) doc["note"] = note;
	std::ofstream out(path);
	if(!out.is_open()) return false;
	out << doc.dump();
	return out.good();
}

} // namespace output
