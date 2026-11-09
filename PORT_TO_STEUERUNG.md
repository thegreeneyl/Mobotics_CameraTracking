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

## Checklist

- [ ] **1. Copy the source folders** into `YOUniverse_Steuerung/src`, keeping
  the folder names so the existing relative includes still resolve:
  `mobotix/`, `recording/MjpegPlayer.*` and `recording/MjpegRecorder.*`,
  `tracking/`, `util/`, `output/MotionPathWriter.*`,
  `session/CameraTrackingSession.*`. None of these includes ofxGui, textures or
  OpenGL.
- [ ] **2. Add `ofxOpenCv`** to Steuerung's `addons.make`. Do not add `ofxGui`.
  Both apps share this openFrameworks tree, so the OpenCV that already links
  `SegModel` (cv::dnn) is the one to use.
- [ ] **3. Copy the shape model** `bin/data/models/yolov8n-seg-192x640.onnx`
  into Steuerung's `bin/data/models/`. `tracking.shape.model` in the config
  stays a data-relative path.
- [ ] **4. Copy the recordings** that are then in CameraTracking's
  `bin/data/recordings/` into Steuerung's `bin/data/recordings/` — they are the
  debug library for playback. Do not copy `recordings-old/`. Gitignore the
  directory if the files are large, as CameraTracking does.
- [ ] **5. Copy the cleaned config** `bin/data/config/config.json` to
  Steuerung's `bin/data/config/trackingConfig.json` (the name can still
  change). Do not merge it into Steuerung's `config.json`. Load it with
  `session.load("config/trackingConfig.json")`; the same path is the save
  target. Revisit the `single_low` layout filter `recordings: ["14-18-08"]` if
  that take is no longer in the library.
- [ ] **6. Own a `CameraTrackingSession` on Steuerung's `ofApp`**, like
  `DeviceManager`. `load()` + `start()` at the end of `initialLoad()`,
  `update()` in `ofApp::update()` before `playbackController.update(...)`,
  `stop()` in `exit()`. The tracking worker thread must never touch OpenGL —
  only the main thread uploads `takePreview()` pixels into a texture.
- [ ] **7. Hand the session to `UILayout`** through a setter
  (`setCameraTrackingSession(CameraTrackingSession*)`), the same pattern as
  `setAppState`.
- [ ] **8. Add `src/ui/CameraTrackingView`** drawing with `UIPanel`, `UITheme`
  and `UIControls` (`drawButton`, `drawCheckbox`). No ofxGui, no
  `ofParameter`. Register `"cameraTracking"` in `UILayout::drawView` next to
  the existing view branches, add a View node plus a `views.cameraTracking`
  block to `bin/data/config/layout.json`.
- [ ] **9. Rebuild the operator surface** on that view, calling the session
  only: lane anchors and `s0`/`s1` (hit-test in `UIPanel::toLocal`, keep the
  edited `LayoutConfig` in the view, re-seed it when `layoutRevision()`
  changes, commit with `setLayout()` on mouse-up); corridor placement (drag
  calls `setPlacement()`); save (`saveConfig()`); reconnect; the view toggles
  (overlay, motion mask, raw detections, shape pixels, outline, box, result vs
  raw, canvas vs lanes). Preview pixels and pane geometry come from
  `takePreview()`, overlay data from `takeResults()`. The drawing code in
  CameraTracking's `ofApp.cpp` (`drawVideo`, `drawCanvasView`,
  `drawTrackingOverlay`, `drawLaneEditor`, `drawTrackingResults`) is the
  reference for the geometry.
- [ ] **10. Rebuild live / playback / record / export as a transport**, not
  checkboxes: play, pause, scrub, step, previous / next recording, record,
  export. They map to `setMode()`, `play()`, `pause()`, `seekTo()`,
  `stepFrame()`, `previousRecording()` / `nextRecording()`,
  `startRecording()` / `stopRecording()`, `startMotionExport()` /
  `stopMotionExport()`; position and duration from `positionSec()` /
  `durationSec()`.
- [ ] **11. Poll `resultJson()` on the main thread** (read when
  `resultRevision()` changes) and leave it unused by the manifest. Do not open
  a socket for it.
- [ ] **12. Linux later.** macOS can ship first. `util/JpegDecode.*` is Apple
  ImageIO; before a Linux build of Steuerung, replace that one implementation
  with libjpeg-turbo. Callers (`MobotixMjpegClient`, `MjpegPlayer`,
  `TrackingManager`) stay as they are.
- [ ] **13. Build Steuerung and check**: live stream, playback of a copied
  recording, an anchor edit saved back into `trackingConfig.json`, a motion
  export file in `recordingsDir`, and `resultJson()` changing when a new
  tracking revision arrives.
