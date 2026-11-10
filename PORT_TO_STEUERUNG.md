# Port to YOUniverse_Steuerung

Task list for moving the camera tracking into `YOUniverse_Steuerung`. Tick the
boxes as you go.

## Boundary

That task moves `CameraTrackingSession` with its folders and rewrites the
operator window in Steuerung's own UI system (`UILayout` / `UIPanel` /
`UIControls`). It ends once `resultJson()` is being produced on Steuerung's
main thread. Wiring that JSON into the manifest is a further task.

Stays here, does not move: UDP output, ofxGui, the BGS and YOLO detectors (all
already deleted), the batch runner (`src/batch/`), `recording/RecordingReader.*`
and `tools/eval/`. Do not bring any of them back in Steuerung.

## Already in Steuerung

The port is done on macOS. The Tracking tab (between Brightness and Log) is
a three-pane split drawn by `CameraTrackingView`; the Brightness tab
(between Schedule and Tracking) drives the Art-Net gain from the MultiSense
readings. What is where, how the config was split, and what is still to be
exercised against a live camera is
[docs/CAMERA_TRACKING.md](../YOUniverse_Steuerung/docs/CAMERA_TRACKING.md).
Where the two differ, the Steuerung document wins.

## Checklist

- [x] **1. Copy the source folders** into `YOUniverse_Steuerung/src`, keeping
  the folder names so the existing relative includes still resolve:
  `mobotix/`, `recording/MjpegPlayer.*` and `recording/MjpegRecorder.*`,
  `tracking/`, `util/`, `output/MotionPathWriter.*`,
  `session/CameraTrackingSession.*`. None of these includes ofxGui, textures or
  OpenGL.
- [x] **2. Add `ofxOpenCv`** to Steuerung's `addons.make`. Do not add `ofxGui`.
  Both apps share this openFrameworks tree, so the OpenCV that already links
  `SegModel` (cv::dnn) is the one to use.
- [x] **3. Copy the shape model** `bin/data/models/yolov8n-seg-192x640.onnx`
  into Steuerung's `bin/data/models/`. `tracking.shape.model` in the config
  stays a data-relative path.
- [x] **4. Copy the recordings** that are then in CameraTracking's
  `bin/data/recordings/` into Steuerung's `bin/data/recordings/` — they are the
  debug library for playback. Do not copy `recordings-old/`. Gitignore the
  directory if the files are large, as CameraTracking does.
- [x] **5. Split the config** instead of copying it whole. Steuerung already
  has homes for two of its parts, so `bin/data/config/config.json` was taken
  apart: the camera (`ip`, `user`, `stream`, `fps`, `preview`, `previewSize`,
  `quality`, `cameraId`) is the `role: "tracking"` entry in Steuerung's
  `devices.json`; the password is `tracking.cameras.<cameraId>.password`
  in Steuerung's `config.json`, beside the API key that file already
  carries; `recordingsDir` is
  `tracking.recordingsDir` in the same `config.json`; `display` and
  `tracking` are the committed `config/trackingConfig.json`, loaded with
  `session.load("config/trackingConfig.json")` and the only thing
  `saveConfig()` writes. The lux curve for the facade level is a separate
  committed `config/brightness.json`. Nothing was merged into Steuerung's
  `config.json` beyond the one path. See Steuerung's
  `bin/data/CONFIGURATION.md`.
- [x] **6. Own a `CameraTrackingSession` on Steuerung's `ofApp`**, like
  `DeviceManager`. `load()` + `start()` at the end of `initialLoad()`,
  `update()` in `ofApp::update()` before `playbackController.update(...)`,
  `stop()` in the quit callback registered in `ofApp::setup`
  (`appState.setQuitCallback`), beside `syncEngine.shutdown()`.
  `ofApp::exit()` only intercepts the window close and must not stop the
  session. The tracking worker thread must never touch OpenGL —
  only the main thread uploads `takePreview()` pixels into a texture.
- [x] **7. Hand the session to `UILayout`** through a setter
  (`setCameraTrackingSession(CameraTrackingSession*)`), the same pattern as
  `setAppState`.
- [x] **8. Fill `src/ui/CameraTrackingView`.** The class, the
  `"cameraTracking"` branch in `UILayout::drawView`, the Tracking tab and
  `views.cameraTracking` already exist and draw an empty state. Replace
  that drawing with `UIPanel`, `UITheme` and `UIControls` (`drawButton`,
  `drawCheckbox`). No ofxGui, no `ofParameter`. Widget, zoom, scroll and
  keyboard rules are in Steuerung's `docs/CAMERA_TRACKING.md`.
- [x] **9. Rebuild the operator surface** on that view, calling the session
  only: lane anchors and `s0`/`s1` (hit-test in `UIPanel::toLocal`, keep the
  edited `LayoutConfig` in the view, re-seed it when `layoutRevision()`
  changes, commit with `setLayout()` on mouse-up); corridor placement (drag
  calls `setPlacement()`); save (`saveConfig()`); reconnect; the view toggles
  (overlay, motion mask, raw detections, shape pixels, outline, box, result vs
  raw, canvas vs lanes); the MultiSense brightness from `getSensors()`
  (top-right lux, live and playback — recordings keep it in the JPEG
  comment). Preview pixels and pane geometry come from
  `takePreview()`, overlay data from `takeResults()`. The drawing code in
  CameraTracking's `ofApp.cpp` (`drawVideo`, `drawCanvasView`,
  `drawTrackingOverlay`, `drawLaneEditor`, `drawTrackingResults`) is the
  reference for the geometry.
- [x] **10. Rebuild live / playback / record / export as a transport**, not
  checkboxes: play, pause, scrub, step, previous / next recording, record,
  export. They map to `setMode()`, `play()`, `pause()`, `seekTo()`,
  `stepFrame()`, `previousRecording()` / `nextRecording()`,
  `startRecording()` / `stopRecording()`, `startMotionExport()` /
  `stopMotionExport()`; position and duration from `positionSec()` /
  `durationSec()`.
- [x] **11. Poll `resultJson()` on the main thread** (read when
  `resultRevision()` changes) and leave it unused by the manifest. Do not open
  a socket for it.
- [ ] **12. Linux later.** macOS can ship first. `util/JpegDecode.*` is Apple
  ImageIO; before a Linux build of Steuerung, replace that one implementation
  with libjpeg-turbo. Callers (`MobotixMjpegClient`, `MjpegPlayer`,
  `TrackingManager`) stay as they are.
- [x] **13. Build Steuerung and check** (macOS; live stream and export still need a reachable camera): live stream, playback of a copied
  recording, an anchor edit saved back into `trackingConfig.json`, a motion
  export file in `recordingsDir`, and `resultJson()` changing when a new
  tracking revision arrives.
