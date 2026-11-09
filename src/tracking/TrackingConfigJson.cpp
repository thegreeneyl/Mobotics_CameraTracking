#include "TrackingConfigJson.h"

#include <algorithm>

#include "ofLog.h"

#include "TrackingParams.h"

namespace tracking {

namespace {
float clamp01(float v){ return std::min(1.0f, std::max(0.0f, v)); }

// Lane stations: 0..1 is the filled frame. A hard stop only rejects a
// corrupt file.
float clampPaneUv(float v){ return std::min(5.0f, std::max(-4.0f, v)); }

bool parseQuad(const ofJson & arr, FrameQuad & quad){
	if(!arr.is_array() || arr.size() != 4) return false;
	FrameQuad q;
	for(int i = 0; i < 4; i++){
		if(!arr[i].is_array() || arr[i].size() < 2) return false;
		q.pts[i][0] = clampPaneUv(arr[i][0].get<float>());
		q.pts[i][1] = clampPaneUv(arr[i][1].get<float>());
	}
	quad = q;
	return true;
}

ofJson quadJson(const FrameQuad & quad){
	ofJson arr = ofJson::array();
	for(const auto & p : quad.pts) arr.push_back({p[0], p[1]});
	return arr;
}

bool parsePolyline(const ofJson & arr, std::vector<std::array<float, 2>> & pts){
	if(!arr.is_array() || arr.size() < 2 || arr.size() > static_cast<size_t>(LaneConfig::kMaxStations)) return false;
	std::vector<std::array<float, 2>> out;
	out.reserve(arr.size());
	for(const auto & p : arr){
		if(!p.is_array() || p.size() < 2) return false;
		out.push_back({clampPaneUv(p[0].get<float>()), clampPaneUv(p[1].get<float>())});
	}
	pts = std::move(out);
	return true;
}

ofJson polylineJson(const std::vector<std::array<float, 2>> & pts){
	ofJson arr = ofJson::array();
	for(const auto & p : pts) arr.push_back({p[0], p[1]});
	return arr;
}

// {"key": number|bool, ...} -> flat map (bools become 0/1)
std::map<std::string, double> parseOverrides(const ofJson & obj){
	std::map<std::string, double> out;
	if(!obj.is_object()) return out;
	for(auto it = obj.begin(); it != obj.end(); ++it){
		if(it->is_boolean()) out[it.key()] = it->get<bool>() ? 1.0 : 0.0;
		else if(it->is_number()) out[it.key()] = it->get<double>();
	}
	return out;
}

ofJson paramsJson(const std::vector<ParamInfo> & params){
	ofJson obj = ofJson::object();
	for(const auto & p : params){
		switch(p.type){
			case ParamType::Bool: obj[p.key] = p.value >= 0.5; break;
			case ParamType::Int: obj[p.key] = static_cast<int>(std::lround(p.value)); break;
			case ParamType::Float: obj[p.key] = p.value; break;
		}
	}
	return obj;
}

ofJson overridesJson(const std::map<std::string, double> & o, const std::vector<ParamInfo> & schema){
	ofJson obj = ofJson::object();
	for(const auto & kv : o){
		auto it = std::find_if(schema.begin(), schema.end(), [&](const ParamInfo & p){ return p.key == kv.first; });
		if(it == schema.end()){ obj[kv.first] = kv.second; continue; }
		switch(it->type){
			case ParamType::Bool: obj[kv.first] = kv.second >= 0.5; break;
			case ParamType::Int: obj[kv.first] = static_cast<int>(std::lround(kv.second)); break;
			case ParamType::Float: obj[kv.first] = kv.second; break;
		}
	}
	return obj;
}

// Applies every numeric/bool member of obj through a TrackingParams setter,
// warning about keys the schema does not know.
template <typename Target, typename Setter>
void applyParams(const ofJson & obj, Target & target, Setter setter, const char * section){
	if(!obj.is_object()) return;
	for(auto it = obj.begin(); it != obj.end(); ++it){
		double v = 0;
		if(it->is_boolean()) v = it->get<bool>() ? 1 : 0;
		else if(it->is_number()) v = it->get<double>();
		else continue;
		if(!setter(target, it.key(), v)){
			ofLogWarning("TrackingConfig") << "unknown " << section << " parameter '" << it.key() << "'";
		}
	}
}
} // namespace

LoadedTrackingConfig loadTrackingConfig(const ofJson & root,
                                        const std::function<std::string(const std::string &)> & resolvePath){
	LoadedTrackingConfig out;
	out.recordingsDir = root.value("recordingsDir", std::string("recordings"));
	bool haveArrowScale = false;
	if(root.contains("display") && root["display"].is_object()){
		const auto & d = root["display"];
		out.displayDecodeScale = d.value("decodeScale", 2);
		if(d.contains("arrowScale")){
			out.displayArrowScale = d.value("arrowScale", 1.0f);
			haveArrowScale = true;
		}
	}
	TrackingConfig & c = out.config;
	if(!root.contains("tracking") || !root["tracking"].is_object()) return out;
	const auto & t = root["tracking"];

	const bool legacyFrame = !(t.contains("lens") && t["lens"].is_object()
	                          && t["lens"].contains("frameFovDeg"));
	c.enabled = t.value("enabled", true);
	c.analysisReduce = t.value("analysisReduce", 2);
	c.laneMaxWidth = t.value("laneMaxWidth", 800);
	c.laneMinHeight = t.value("laneMinHeight", 64);
	if(t.contains("lens") && t["lens"].is_object()){
		const auto & l = t["lens"];
		c.lens.enabled = l.value("enabled", c.lens.enabled);
		c.lens.fovDeg = l.value("fovDeg", c.lens.fovDeg);
		c.lens.k1 = l.value("k1", c.lens.k1);
		c.lens.k2 = l.value("k2", c.lens.k2);
		c.lens.k3 = l.value("k3", c.lens.k3);
		c.lens.k4 = l.value("k4", c.lens.k4);
		c.lens.frameFovDeg = l.value("frameFovDeg", 0.0f);
		c.lens.centerX = l.value("centerX", c.lens.centerX);
		c.lens.centerY = l.value("centerY", c.lens.centerY);
		if(l.contains("balance")){
			ofLogNotice("TrackingConfig") << "lens.balance is ignored; the frame spans the sensor";
		}
	}

	if(t.contains("flow")) applyParams(t["flow"], c.flow, setFlowParam, "flow");
	if(t.contains("tracker")) applyParams(t["tracker"], c.tracker, setTrackerParam, "tracker");
	// shape: flat ShapeEstimator parameters plus "model" (ONNX path)
	c.shape.modelFile = "models/yolov8n-seg.onnx";
	if(t.contains("shape") && t["shape"].is_object()){
		ofJson shape = t["shape"];
		if(shape.contains("model")){
			if(shape["model"].is_string()) c.shape.modelFile = shape["model"].get<std::string>();
			shape.erase("model");
		}
		applyParams(shape, c.shape, setShapeParam, "shape");
	}
	c.shape.modelPath = c.shape.modelFile.empty() ? std::string() : resolvePath(c.shape.modelFile);
	if(t.contains("smoothing")) applyParams(t["smoothing"], c.smoothing, setSmoothParam, "smoothing");

	// Older documents. Read once, then gone on the next save.
	if(t.contains("overlay") && t["overlay"].is_object()){
		const auto & o = t["overlay"];
		if(!haveArrowScale) out.displayArrowScale = o.value("arrowScale", 1.0f);
		if(!(t.contains("tracker") && t["tracker"].is_object() && t["tracker"].contains("trailLen"))){
			c.tracker.trailLen = o.value("trailFrames", c.tracker.trailLen);
		}
		ofLogNotice("TrackingConfig") << "legacy key tracking.overlay read (now display.arrowScale / tracker.trailLen)";
	}
	for(const char * legacy : {"detector", "bgs", "yolo"}){
		if(t.contains(legacy)){
			ofLogNotice("TrackingConfig") << "legacy key tracking." << legacy << " ignored (flow is the only detector)";
		}
	}

	// canvas: {"width": 832, "height": 442} — output proportions
	if(t.contains("canvas") && t["canvas"].is_object()){
		c.canvas.width = std::max(1, t["canvas"].value("width", 832));
		c.canvas.height = std::max(1, t["canvas"].value("height", 442));
	}

	// corridors: {"train": {"label": "train", "placement": {"x0","x1","y0","y1"},
	//                       "flow": {...}, "tracker": {...}}, ...}
	c.corridors.clear();
	if(t.contains("corridors") && t["corridors"].is_object()){
		for(auto it = t["corridors"].begin(); it != t["corridors"].end(); ++it){
			CorridorConfig cc;
			cc.id = it.key();
			cc.label = it->value("label", cc.id);
			if(it->contains("placement") && (*it)["placement"].is_object()){
				const auto & p = (*it)["placement"];
				cc.placement.x0 = p.value("x0", 0.0f);
				cc.placement.x1 = p.value("x1", 1.0f);
				cc.placement.y0 = p.value("y0", 0.0f);
				cc.placement.y1 = p.value("y1", 1.0f);
			}
			if(it->contains("flow")) cc.flow = parseOverrides((*it)["flow"]);
			if(it->contains("tracker")) cc.tracker = parseOverrides((*it)["tracker"]);
			if(it->contains("shape")) cc.shape = parseOverrides((*it)["shape"]);
			c.corridors.push_back(cc);
		}
	}

	// layouts: [{"name","panes","recordings":[..],"lanes":[{"id","pane","corridor",
	//   "quad":[[x,y]x4], optional "top"/"bottom" polylines, "s0","s1"}]}]
	c.layouts.clear();
	if(t.contains("layouts") && t["layouts"].is_array()){
		for(const auto & lj : t["layouts"]){
			LayoutConfig l;
			l.name = lj.value("name", std::string("layout"));
			l.panes = lj.value("panes", 1);
			if(lj.contains("recordings") && lj["recordings"].is_array()){
				for(const auto & r : lj["recordings"]) l.recordings.push_back(r.get<std::string>());
			}
			if(lj.contains("lanes") && lj["lanes"].is_array()){
				for(const auto & lanej : lj["lanes"]){
					LaneConfig lane;
					lane.id = lanej.value("id", std::string("lane"));
					lane.pane = lanej.value("pane", 0);
					lane.corridor = lanej.value("corridor", lane.id);
					if(lanej.contains("quad")) parseQuad(lanej["quad"], lane.quad);
					// Optional finer border. Both polylines, same length, left
					// to right; the quad stays the four ends for older readers.
					if(lanej.contains("top") && lanej.contains("bottom")){
						std::vector<std::array<float, 2>> top, bottom;
						if(parsePolyline(lanej["top"], top) && parsePolyline(lanej["bottom"], bottom)
						   && top.size() == bottom.size()){
							lane.top = std::move(top);
							lane.bottom = std::move(bottom);
							lane.syncQuadEnds();
						}
					}
					lane.s0 = lanej.value("s0", 0.0f);
					lane.s1 = lanej.value("s1", 1.0f);
					// optional "yBand": [y0, y1] (lane fractions)
					if(lanej.contains("yBand") && lanej["yBand"].is_array() && lanej["yBand"].size() >= 2){
						lane.yBand0 = clamp01(lanej["yBand"][0].get<float>());
						lane.yBand1 = clamp01(lanej["yBand"][1].get<float>());
						if(lane.yBand1 < lane.yBand0) std::swap(lane.yBand0, lane.yBand1);
					}
					l.lanes.push_back(lane);
				}
			}
			c.layouts.push_back(l);
		}
	}
	// Anchors saved before frameFovDeg sit in the old 16:9 crop. Move them
	// once, with the lens this file already has; the next save writes the key.
	if(legacyFrame){
		int moved = 0;
		for(LayoutConfig & layout : c.layouts){
			for(LaneConfig & lane : layout.lanes){
				auto fix = [&](std::array<float, 2> & pt){
					float u = pt[0], v = pt[1];
					if(!migrateBalanceUv(c.lens, u, v)) return;
					pt[0] = u;
					pt[1] = v;
					moved++;
				};
				for(auto & pt : lane.quad.pts) fix(pt);
				for(auto & pt : lane.top) fix(pt);
				for(auto & pt : lane.bottom) fix(pt);
				lane.syncQuadEnds();
			}
		}
		if(moved){
			ofLogNotice("TrackingConfig")
				<< "moved " << moved << " lane anchors from the 16:9 crop into the sensor frame";
		}
	}
	return out;
}

ofJson displayJson(int decodeScale, float arrowScale){
	return {{"decodeScale", decodeScale}, {"arrowScale", arrowScale}};
}

ofJson trackingJson(const TrackingConfig & cfg){
	ofJson t = ofJson::object();
	t["enabled"] = cfg.enabled;
	t["analysisReduce"] = cfg.analysisReduce;
	t["laneMaxWidth"] = cfg.laneMaxWidth;
	t["laneMinHeight"] = cfg.laneMinHeight;
	t["lens"] = {
		{"enabled", cfg.lens.enabled},
		{"fovDeg", cfg.lens.fovDeg},
		{"k1", cfg.lens.k1},
		{"k2", cfg.lens.k2},
		{"k3", cfg.lens.k3},
		{"k4", cfg.lens.k4},
		{"frameFovDeg", cfg.lens.frameFovDeg},
		{"centerX", cfg.lens.centerX},
		{"centerY", cfg.lens.centerY},
	};
	t["flow"] = paramsJson(flowParams(cfg.flow));
	t["tracker"] = paramsJson(trackerParams(cfg.tracker));
	ofJson shape = paramsJson(shapeParams(cfg.shape));
	shape["model"] = cfg.shape.modelFile;
	t["shape"] = shape;
	t["smoothing"] = paramsJson(smoothParams(cfg.smoothing));

	t["canvas"]["width"] = cfg.canvas.width;
	t["canvas"]["height"] = cfg.canvas.height;

	ofJson corridors = ofJson::object();
	const auto flowSchema = flowParams(cfg.flow);
	const auto trackerSchema = trackerParams(cfg.tracker);
	const auto shapeSchema = shapeParams(cfg.shape);
	for(const auto & cc : cfg.corridors){
		ofJson cj = ofJson::object();
		cj["label"] = cc.label;
		cj["placement"] = {{"x0", cc.placement.x0}, {"x1", cc.placement.x1},
		                   {"y0", cc.placement.y0}, {"y1", cc.placement.y1}};
		if(!cc.flow.empty()) cj["flow"] = overridesJson(cc.flow, flowSchema);
		if(!cc.tracker.empty()) cj["tracker"] = overridesJson(cc.tracker, trackerSchema);
		if(!cc.shape.empty()) cj["shape"] = overridesJson(cc.shape, shapeSchema);
		corridors[cc.id] = cj;
	}
	t["corridors"] = corridors;

	ofJson layouts = ofJson::array();
	for(const auto & l : cfg.layouts){
		ofJson lj = ofJson::object();
		lj["name"] = l.name;
		lj["panes"] = l.panes;
		lj["recordings"] = l.recordings;
		ofJson lanes = ofJson::array();
		for(const auto & lane : l.lanes){
			LaneConfig stored = lane;
			stored.syncQuadEnds();
			ofJson lanej = {
				{"id", stored.id},
				{"pane", stored.pane},
				{"corridor", stored.corridor},
				{"quad", quadJson(stored.quad)},
				{"s0", stored.s0},
				{"s1", stored.s1},
			};
			if(stored.stationCount() > 2){
				lanej["top"] = polylineJson(stored.top);
				lanej["bottom"] = polylineJson(stored.bottom);
			}
			if(lane.hasYBand()) lanej["yBand"] = {lane.yBand0, lane.yBand1};
			lanes.push_back(lanej);
		}
		lj["lanes"] = lanes;
		layouts.push_back(lj);
	}
	t["layouts"] = layouts;
	return t;
}

} // namespace tracking
