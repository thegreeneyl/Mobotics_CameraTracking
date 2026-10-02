#pragma once

#include <map>
#include <string>
#include <vector>

#include "FlowMotionDetector.h"
#include "MultiObjectTracker.h"
#include "ShapeEstimator.h"

// Name <-> field tables for the tunable detector / tracker parameters.
// One place that knows the JSON key names, used by the config loader, the
// config writer and the per-corridor override mechanism (a corridor may
// override any subset of the global "flow" / "tracker" values, e.g.
// lockAxis for the train corridor). No OF dependencies.
namespace tracking {

enum class ParamType { Bool, Int, Float };

struct ParamInfo {
	std::string key;
	ParamType type;
	double value;
};

// Set one parameter by key; returns false for unknown keys.
bool setFlowParam(FlowDetectorConfig & c, const std::string & key, double v);
bool setTrackerParam(TrackerConfig & c, const std::string & key, double v);
bool setShapeParam(ShapeConfig & c, const std::string & key, double v); // numeric keys only (not `model`)

// All parameters with their current values (for saving / GUI listing).
std::vector<ParamInfo> flowParams(const FlowDetectorConfig & c);
std::vector<ParamInfo> trackerParams(const TrackerConfig & c);
std::vector<ParamInfo> shapeParams(const ShapeConfig & c);

// Apply a flat override map (unknown keys are ignored).
void applyFlowOverrides(FlowDetectorConfig & c, const std::map<std::string, double> & o);
void applyTrackerOverrides(TrackerConfig & c, const std::map<std::string, double> & o);
void applyShapeOverrides(ShapeConfig & c, const std::map<std::string, double> & o);

} // namespace tracking
