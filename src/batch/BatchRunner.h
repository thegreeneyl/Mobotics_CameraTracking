#pragma once

#include <string>
#include <vector>

// Headless batch mode: runs the tracking pipeline over recordings as fast as
// the CPU allows (no window, no pacing) and writes evaluation artefacts:
//
//   <out>/<rec>.motion.json   motion-path.v1 recording (replayable in the
//                             FacadeEditor's CameraTrackingPanel)
//   <out>/<rec>.sheet.jpg     overlay contact sheet, one thumbnail every
//                             sheetIntervalSec
//   <out>/<rec>.summary.json  per-run timing + frame stats
//
// Configured from the environment by main():
//   CAMTRACK_BATCH=<rec>[,<rec>...] | all     recording base names
//   CAMTRACK_OUT=<dir>                        default bin/data/eval
//   CAMTRACK_SHEET_SEC=<sec>                  default 5
//   CAMTRACK_RANGE=<startSec>-<endSec>        optional time window
//   CAMTRACK_STRIDE=<n>                       process every n-th frame
//   CAMTRACK_CONFIG=<path>                    config file (default config/config.json)
//   CAMTRACK_DEBUG=1                          also write <rec>.debug.jsonl: one line
//                                             per frame with raw lane detections and
//                                             ALL tracks (incl. tentative) for tuning
namespace batch {

struct Options {
	std::vector<std::string> recordings; // base names, empty = all
	std::string outDir;                  // absolute
	std::string configPath;              // absolute
	std::string recordingsDir;           // absolute (overrides config)
	double sheetIntervalSec = 5.0;
	double startSec = 0;
	double endSec = -1; // < 0 = to the end
	int stride = 1;
	bool debugDump = false;
	double stripIntervalSec = 0; // > 0: dump rectified strip + detector mask per lane
};

// Fills options from the environment. Returns false when CAMTRACK_BATCH is
// not set (normal GUI start).
bool optionsFromEnv(Options & opts);

// Runs all requested recordings; returns a process exit code.
int run(const Options & opts);

} // namespace batch
