#include "TrackingParams.h"

#include <cmath>
#include <functional>

namespace tracking {

namespace {

template <typename Cfg>
struct Field {
	const char * key;
	ParamType type;
	std::function<double(const Cfg &)> get;
	std::function<void(Cfg &, double)> set;
};

#define F_FLOAT(Cfg, name) Field<Cfg>{#name, ParamType::Float, \
	[](const Cfg & c){ return static_cast<double>(c.name); }, \
	[](Cfg & c, double v){ c.name = static_cast<float>(v); }}
#define F_INT(Cfg, name) Field<Cfg>{#name, ParamType::Int, \
	[](const Cfg & c){ return static_cast<double>(c.name); }, \
	[](Cfg & c, double v){ c.name = static_cast<int>(std::lround(v)); }}
#define F_BOOL(Cfg, name) Field<Cfg>{#name, ParamType::Bool, \
	[](const Cfg & c){ return c.name ? 1.0 : 0.0; }, \
	[](Cfg & c, double v){ c.name = v >= 0.5; }}
#define F_DOUBLE(Cfg, name) Field<Cfg>{#name, ParamType::Float, \
	[](const Cfg & c){ return c.name; }, \
	[](Cfg & c, double v){ c.name = v; }}

const std::vector<Field<FlowDetectorConfig>> & flowFields(){
	static const std::vector<Field<FlowDetectorConfig>> f = {
		F_INT(FlowDetectorConfig, preset),
		F_FLOAT(FlowDetectorConfig, flowEma),
		F_FLOAT(FlowDetectorConfig, minFlowPx),
		F_FLOAT(FlowDetectorConfig, sparkleRatio),
		F_FLOAT(FlowDetectorConfig, busyAlpha),
		F_FLOAT(FlowDetectorConfig, busyThresh),
		F_BOOL(FlowDetectorConfig, useMog2),
		F_INT(FlowDetectorConfig, mogHistory),
		F_FLOAT(FlowDetectorConfig, mogVarThreshold),
		F_INT(FlowDetectorConfig, mogDilatePx),
		F_FLOAT(FlowDetectorConfig, fillWFrac),
		F_FLOAT(FlowDetectorConfig, fillHFrac),
		F_INT(FlowDetectorConfig, morphOpenPx),
		F_FLOAT(FlowDetectorConfig, closeWFrac),
		F_FLOAT(FlowDetectorConfig, closeHFrac),
		F_FLOAT(FlowDetectorConfig, minAreaFrac),
		F_FLOAT(FlowDetectorConfig, minWidthFrac),
		F_FLOAT(FlowDetectorConfig, minHeightFrac),
		F_FLOAT(FlowDetectorConfig, minCoherence),
		F_FLOAT(FlowDetectorConfig, minSpeed),
		F_FLOAT(FlowDetectorConfig, maxSpeed),
		F_FLOAT(FlowDetectorConfig, mergeGapFrac),
		F_FLOAT(FlowDetectorConfig, mergeAngleDeg),
		F_FLOAT(FlowDetectorConfig, mergeSpeedRatio),
		F_FLOAT(FlowDetectorConfig, mergeLateralOverlap),
		F_BOOL(FlowDetectorConfig, lockAxis),
		F_INT(FlowDetectorConfig, maxDetections),
		F_INT(FlowDetectorConfig, edgeMarginPx),
	};
	return f;
}

const std::vector<Field<TrackerConfig>> & trackerFields(){
	static const std::vector<Field<TrackerConfig>> f = {
		F_INT(TrackerConfig, confirmFrames),
		F_INT(TrackerConfig, maxMisses),
		F_INT(TrackerConfig, tentativeMaxMisses),
		F_FLOAT(TrackerConfig, minSpeedConfirm),
		F_FLOAT(TrackerConfig, gateGapFrac),
		F_FLOAT(TrackerConfig, gateLateral),
		F_FLOAT(TrackerConfig, velAngleDeg),
		F_FLOAT(TrackerConfig, velScale),
		F_FLOAT(TrackerConfig, absorbGapFrac),
		F_FLOAT(TrackerConfig, absorbLateral),
		F_BOOL(TrackerConfig, shadowSuppress),
		F_FLOAT(TrackerConfig, stopEdgeFrac),
		F_FLOAT(TrackerConfig, stopMinWidthFrac),
		F_FLOAT(TrackerConfig, sizeJumpTrust),
		F_FLOAT(TrackerConfig, sizeJumpNoiseGain),
		F_INT(TrackerConfig, clipFrames),
		F_FLOAT(TrackerConfig, shadowGapFrac),
		F_FLOAT(TrackerConfig, shadowLateral),
		F_FLOAT(TrackerConfig, stopSpeed),
		F_DOUBLE(TrackerConfig, stoppedHoldMs),
		F_FLOAT(TrackerConfig, mergeGapFrac),
		F_FLOAT(TrackerConfig, mergeVelTol),
		F_INT(TrackerConfig, mergeFrames),
		F_DOUBLE(TrackerConfig, reacquireMs),
		F_FLOAT(TrackerConfig, reacquireGapFrac),
		F_BOOL(TrackerConfig, lockAxis),
		F_FLOAT(TrackerConfig, posMeasNoise),
		F_FLOAT(TrackerConfig, velMeasNoise),
		F_FLOAT(TrackerConfig, posProcNoise),
		F_FLOAT(TrackerConfig, velProcNoise),
		F_FLOAT(TrackerConfig, clippedMeasNoise),
		F_INT(TrackerConfig, trailLen),
		F_DOUBLE(TrackerConfig, maxDtSec),
	};
	return f;
}

template <typename Cfg>
bool setByKey(const std::vector<Field<Cfg>> & fields, Cfg & c, const std::string & key, double v){
	for(const auto & f : fields){
		if(key == f.key){
			f.set(c, v);
			return true;
		}
	}
	return false;
}

template <typename Cfg>
std::vector<ParamInfo> listAll(const std::vector<Field<Cfg>> & fields, const Cfg & c){
	std::vector<ParamInfo> out;
	out.reserve(fields.size());
	for(const auto & f : fields) out.push_back({f.key, f.type, f.get(c)});
	return out;
}

} // namespace

bool setFlowParam(FlowDetectorConfig & c, const std::string & key, double v){
	return setByKey(flowFields(), c, key, v);
}
bool setTrackerParam(TrackerConfig & c, const std::string & key, double v){
	return setByKey(trackerFields(), c, key, v);
}
std::vector<ParamInfo> flowParams(const FlowDetectorConfig & c){ return listAll(flowFields(), c); }
std::vector<ParamInfo> trackerParams(const TrackerConfig & c){ return listAll(trackerFields(), c); }

void applyFlowOverrides(FlowDetectorConfig & c, const std::map<std::string, double> & o){
	for(const auto & kv : o) setFlowParam(c, kv.first, kv.second);
}
void applyTrackerOverrides(TrackerConfig & c, const std::map<std::string, double> & o){
	for(const auto & kv : o) setTrackerParam(c, kv.first, kv.second);
}

} // namespace tracking
