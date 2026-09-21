#include "ofApp.h"
#include "ofMain.h"

//========================================================================
int main(){

	ofGLFWWindowSettings settings;
	settings.setSize(1920, 1080);
	settings.windowMode = OF_WINDOW;
	settings.title = "YOUniverse_CameraTracking";

	auto window = ofCreateWindow(settings);

	ofRunApp(window, std::make_shared<ofApp>());
	ofRunMainLoop();
}
