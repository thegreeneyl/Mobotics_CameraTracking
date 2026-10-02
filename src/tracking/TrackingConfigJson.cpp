#include "TrackingConfigJson.h"

#include <algorithm>

#include "ofLog.h"

#include "TrackingParams.h"

namespace tracking {

namespace {
float clamp01(float v){ return std::min(1.0f, std::max(0.0f, v)); }

bool parseQuad(const ofJson & arr, FrameQuad & quad){
	if(!arr.is_array() || arr.size() != 4) return false;
	FrameQuad q;
	for(int i = 0; i < 4; i++){
		if(!arr[i].is_array() || arr[i].size() < 2) return false;
		q.pts[i][0] = clamp01(arr[i][0].get<float>());
		q.pts[i][1] = clamp01(arr[i][1].get<float>());
	}
	quad = q;
	return true;
}

ofJson quadJson(const FrameQuad & quad){
	ofJson arr = ofJson::array();
	for(const auto & p : quad.pts) arr.push_back({p[0], p[1]});
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
} // namespace

LoadedTrackingConfig loadTrackingConfig(const ofJson & root,
                                        const std::function<std::string(const std::string &)> & resolvePath){
	LoadedTrackingConfig out;
	out.recordingsDir = root.value("recordingsDir", std::string("recordings"));
	if(root.contains("display")){
		out.displayDecodeScale = root["display"].value("decodeScale", 2);
	}
	TrackingConfig & c = out.config;
	if(!root.contains("tracking")) return out;
	const auto & t = root["tracking"];

	c.enabled = t.value("enabled", true);
	c.detector = t.value("detector", std::string("flow"));
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
		c.lens.balance = l.value("balance", c.lens.balance);
		c.lens.centerX = l.value("centerX", c.lens.centerX);
		c.lens.centerY = l.value("centerY", c.lens.centerY);
	}

	if(t.contains("flow") && t["flow"].is_object()){
		for(auto it = t["flow"].begin(); it != t["flow"].end(); ++it){
			double v = 0;
			if(it->is_boolean()) v = it->get<bool>() ? 1 : 0;
			else if(it->is_number()) v = it->get<double>();
			else continue;
			if(!setFlowParam(c.flow, it.key(), v)){
				ofLogWarning("TrackingConfig") << "unknown flow parameter '" << it.key() << "'";
			}
		}
	}
	if(t.contains("bgs")){
		const auto & b = t["bgs"];
		c.bgs.history = b.value("history", 500);
		c.bgs.varThreshold = b.value("varThreshold", 16.0);
		c.bgs.learningRate = b.value("learningRate", -1.0);
		c.bgs.minAreaNorm = b.value("minAreaNorm", 0.0005f);
		c.bgs.maxAreaNorm = b.value("maxAreaNorm", 0.5f);
		c.bgs.morphOpenPx = b.value("morphOpen", 3);
		c.bgs.morphClosePx = b.value("morphClose", 9);
	}
	if(t.contains("tracker") && t["tracker"].is_object()){
		for(auto it = t["tracker"].begin(); it != t["tracker"].end(); ++it){
			double v = 0;
			if(it->is_boolean()) v = it->get<bool>() ? 1 : 0;
			else if(it->is_number()) v = it->get<double>();
			else continue;
			if(!setTrackerParam(c.tracker, it.key(), v)){
				ofLogWarning("TrackingConfig") << "unknown tracker parameter '" << it.key() << "'";
			}
		}
	}
	// shape: flat ShapeEstimator parameters plus "model" (ONNX path)
	c.shape.modelFile = "models/yolov8n-seg.onnx";
	if(t.contains("shape") && t["shape"].is_object()){
		for(auto it = t["shape"].begin(); it != t["shape"].end(); ++it){
			if(it.key() == "model"){
				if(it->is_string()) c.shape.modelFile = it->get<std::string>();
				continue;
			}
			double v = 0;
			if(it->is_boolean()) v = it->get<bool>() ? 1 : 0;
			else if(it->is_number()) v = it->get<double>();
			else continue;
			if(!setShapeParam(c.shape, it.key(), v)){
				ofLogWarning("TrackingConfig") << "unknown shape parameter '" << it.key() << "'";
			}
		}
	}
	c.shape.modelPath = c.shape.modelFile.empty() ? std::string() : resolvePath(c.shape.modelFile);
	if(t.contains("yolo")){
		const auto & y = t["yolo"];
		c.yolo.modelPath = resolvePath(y.value("model", std::string("models/yolov8n.onnx")));
		c.yolo.inputSize = y.value("inputSize", 640);
		c.yolo.confThreshold = y.value("confThreshold", 0.35f);
		c.yolo.nmsThreshold = y.value("nmsThreshold", 0.45f);
		c.yolo.classFilter.clear();
		if(y.contains("classes") && y["classes"].is_array()){
			for(const auto & cls : y["classes"]) c.yolo.classFilter.push_back(cls.get<std::string>());
		}
	}
	if(t.contains("overlay")){
		const auto & o = t["overlay"];
		out.overlayArrowScale = o.value("arrowScale", 1.0f);
		c.tracker.trailLen = o.value("trailFrames", c.tracker.trailLen);
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

	// layouts: [{"name","panes","recordings":[..],"lanes":[{"id","pane","corridor","quad":[[x,y]x4],"s0","s1"}]}]
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
					lane.s0 = lanej.value("s0", 0.0f);
					lane.s1 = lanej.value("s1", 1.0f);
					l.lanes.push_back(lane);
				}
			}
			c.layouts.push_back(l);
		}
	}
	return out;
}

void writeTrackingTuning(ofJson & root, const TrackingConfig & cfg, float overlayArrowScale){
	auto & t = root["tracking"];
	t["detector"] = cfg.detector;
	t["laneMaxWidth"] = cfg.laneMaxWidth;
	t["laneMinHeight"] = cfg.laneMinHeight;
	t["lens"] = {
		{"enabled", cfg.lens.enabled},
		{"fovDeg", cfg.lens.fovDeg},
		{"k1", cfg.lens.k1},
		{"k2", cfg.lens.k2},
		{"k3", cfg.lens.k3},
		{"k4", cfg.lens.k4},
		{"balance", cfg.lens.balance},
		{"centerX", cfg.lens.centerX},
		{"centerY", cfg.lens.centerY},
	};
	t["flow"] = paramsJson(flowParams(cfg.flow));
	t["bgs"]["history"] = cfg.bgs.history;
	t["bgs"]["varThreshold"] = cfg.bgs.varThreshold;
	t["bgs"]["learningRate"] = cfg.bgs.learningRate;
	t["bgs"]["minAreaNorm"] = cfg.bgs.minAreaNorm;
	t["bgs"]["maxAreaNorm"] = cfg.bgs.maxAreaNorm;
	t["bgs"]["morphOpen"] = cfg.bgs.morphOpenPx;
	t["bgs"]["morphClose"] = cfg.bgs.morphClosePx;
	ofJson tracker = paramsJson(trackerParams(cfg.tracker));
	tracker.erase("trailLen"); // lives under overlay.trailFrames
	t["tracker"] = tracker;
	ofJson shape = paramsJson(shapeParams(cfg.shape));
	shape["model"] = cfg.shape.modelFile;
	t["shape"] = shape;
	t["yolo"]["confThreshold"] = cfg.yolo.confThreshold;
	t["yolo"]["nmsThreshold"] = cfg.yolo.nmsThreshold;
	t["overlay"]["arrowScale"] = overlayArrowScale;
	t["overlay"]["trailFrames"] = cfg.tracker.trailLen;

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
			lanes.push_back({
				{"id", lane.id},
				{"pane", lane.pane},
				{"corridor", lane.corridor},
				{"quad", quadJson(lane.quad)},
				{"s0", lane.s0},
				{"s1", lane.s1},
			});
		}
		lj["lanes"] = lanes;
		layouts.push_back(lj);
	}
	t["layouts"] = layouts;
	t.erase("zones");
	t.erase("frames");
}

} // namespace tracking
