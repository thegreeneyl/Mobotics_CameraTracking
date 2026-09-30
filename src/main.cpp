#include "ofApp.h"
#include "ofMain.h"

#include "batch/BatchRunner.h"

//========================================================================
int main(){
	// Headless evaluation: CAMTRACK_BATCH=<rec>[,<rec>] runs the tracking
	// pipeline over recordings without a window and exits (see BatchRunner.h).
	batch::Options batchOpts;
	if(batch::optionsFromEnv(batchOpts)){
		ofSetLogLevel(OF_LOG_NOTICE);
		return batch::run(batchOpts);
	}

	ofGLFWWindowSettings settings;
	settings.setSize(1920, 1080);
	settings.windowMode = OF_WINDOW;
	settings.title = "YOUniverse_CameraTracking";

	auto window = ofCreateWindow(settings);

	ofRunApp(window, std::make_shared<ofApp>());
	ofRunMainLoop();
}
