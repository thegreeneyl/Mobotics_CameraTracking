# YOUniverse_CameraTracking

Standalone openFrameworks 0.12.1 test app for integrating the Mobotix Mx-S74A
cameras (motion + brightness sensors) into the YOUniverse system. It pulls the
authenticated live MJPEG stream, shows both optical modules side by side, and
displays the raw Mx-F-MSA (MultiSense) brightness value parsed from the JPEG
metadata of every frame.

This app is deliberately independent of `YOUniverse_Steuerung` — it exists to
validate camera ingest before any production integration. The
`src/mobotix/` classes have no UI dependencies so Steuerung can reuse them later.

## Hardware

| Unit | Modules | Purpose |
|---|---|---|
| Mx-S74A #1 (`192.168.178.230`) | M1 + M2: Mx-O-M7SA-8DN050 (4K Day/Night), M3: Mx-F-MSA MultiSense | Motion detection + brightness (north) |
| Mx-S74A #2 (later) | Mx-F-MSA MultiSense | Brightness (west) |

Camera #1 details (verified 2026-09): firmware **MX-V7.3.1.57**, live image
**BOTH** modules side by side at **3840×1080**.

## Build & run

```bash
cd apps/myApps/YOUniverse_CameraTracking
make -j Release
make RunRelease
```

Configuration: copy `bin/data/config/config.example.json` to
`bin/data/config/config.json` (gitignored) and fill in the camera password.
Only the first camera with `"enabled": true` is used in v1; the second CPU
entry is a stub for later.

### Keys / GUI

- `r` — reconnect (also GUI button)
- `p` — toggle preview (downscaled) stream
- `c` — toggle raw JPEG COM metadata dump
- `s` — save a screenshot to `bin/data/`
- `t` — toggle object tracking
- `o` — toggle the tracking debug overlay
- `m` — show the detector's motion mask behind the rectified lane views
- `d` — draw raw (pre-tracker) detections in grey
- `[` / `]` — select the previous / next lane for editing
- `v` — result view: output canvas (default) / rectified lane strips
- `,` / `.` — select the previous / next corridor for placing on the canvas
- `SPACE` — start/stop recording the incoming stream (live mode only)
- `TAB` — toggle LIVE / PLAYBACK mode
- `LEFT` / `RIGHT` — previous / next recording (playback mode, wraps)
- `UP` / `DOWN` — seek +/-1 s within the current recording (playback mode)
- GUI: preview toggle, JPEG quality (preview only), COM dump toggle,
  record toggle, playback-mode toggle, tracking/overlay/mask/detections
  toggles, result view switch (0 canvas / 1 lane strips), detector switch
  (0 flow / 1 bgs / 2 yolo), half-res display decode toggle, UDP publish
  toggle
- Tracking panel (below the stream panel): lane selector with the selected
  lane's corridor range `s0`/`s1`, corridor selector with the selected
  corridor's canvas placement `place x0/x1/y0/y1` (see *Canvas placement*
  below), live sliders for the flow detector
  (`minFlowPx`, `ema`, `busyThresh`, MOG2 on/off + `varThreshold`, closing
  kernel, `minAreaFrac`, `minCoherence`, `minSpeed`, `mergeGapFrac`), the
  tracker (`confirmFrames`, `maxMisses`, `gateGapFrac`, `absorbGapFrac`,
  `mergeFrames`, `reacquireMs`, `velMeasNoise`, trail length), YOLO
  thresholds and the overlay arrow scale. Changes reach the tracking worker
  within ~250 ms — tweak while watching the overlay. "save to config.json"
  writes the current values (plus the layouts, canvas and corridor
  placements) back into the `tracking` section of the loaded config file;
  nothing else in the file is touched. The sliders edit the *base*
  `flow`/`tracker` blocks; per-corridor parameter overrides
  (`tracking.corridors.<id>.flow/.tracker`) stay as they are in the file.
- Lane editor: every lane of the active layout is a draggable four-corner
  quad on its pane (the selected lane is highlighted; grabbing a corner
  selects that lane; overlay must be visible). Only pixels inside a lane are
  analysed — the quad is perspective-warped to an upright rectangle before
  detection, so it also corrects a skewed camera view. A crossed/collapsed
  quad turns red and is ignored until it is convex again; releasing a
  corner commits the lane (the worker rebuilds that layout's pipelines).
- Result view (below the video, `v` switches):
  - **canvas** (default) — the output canvas in the proportions of the
    Lines facade area (832×442) with a faint 10 % grid, every corridor's
    placement rectangle (dim where it lies outside the canvas — that part of
    the corridor is cut) and the tracked objects exactly as they are
    published: mapped through the placement and clipped at the canvas edge
    (the uncut box is drawn dim behind it). The selected corridor has corner
    handles; drag a corner to resize, drag the body to move — grabbing
    another corridor selects it. The `place` sliders follow the drag and can
    be used instead of it.
  - **lanes** — below each pane one rectified strip per lane shows what the
    detector sees (with the motion mask behind it when `m` is on), the raw
    detections (`d`) and the tracked corridor objects mapped into that lane.
  The overlay on the photo above projects the same data back through the
  lane quads; red ticks on a box mark an edge pinned to a corridor end
  (object still entering / already leaving).

### Recording & playback

Recording taps the raw bytes of every complete JPEG on the stream thread
(before decode) and appends them unchanged to
`bin/data/<recordingsDir>/rec_YYYY-MM-DD_HH-MM-SS.mjpeg` (`recordingsDir`
in `config.json`, default `recordings`, created on demand; each recording
gets a fresh timestamp). A JSON sidecar (`rec_<same-stamp>.json`) with
per-frame `{offset, size, tMs}` entries is written on stop, so playback is
paced exactly like the original stream. No re-encoding — the JPEG COM
sensor metadata stays intact, and the sensor panel keeps working during
playback.

Playback (`TAB`) starts with the most recent recording, plays it to the end,
then cycles onward through all recordings newest to oldest (wrapping).
Arrow keys jump within that cycle. While in playback mode the live stream
keeps running in the background but is ignored; `TAB` returns to it
instantly. Entering playback stops an active recording first.

Headless check: `CAMTRACK_AUTOSHOT=/tmp/shot.png ./YOUniverse_CameraTracking`
streams for ~8 s, saves a screenshot, and exits. Stats are logged every 5 s.
Add `CAMTRACK_AUTOPLAYBACK=1` to start in playback mode instead (verifies
recording playback + tracking without a reachable camera).

## Stream ingest (what was chosen and why)

**Chosen: authenticated HTTP MJPEG**

```text
http://<ip>/control/faststream.jpg?stream=full&fps=0
```

- `stream=full` — independent JPEGs, decodable natively by OF (FreeImage)
- `fps=0` — maximum frame rate
- `/control/` (user access) — the guest path `/cgi-bin/faststream.jpg` is
  capped at ~2 fps by camera policy
- Auth: camera accepts **Basic and Digest**; the client uses libcurl with
  `CURLAUTH_ANY` (libcurl is already linked by the OF core)

Measured on camera #1 (full 3840×1080 stream, quality 60):

| Path | Result |
|---|---|
| `/control/faststream.jpg?stream=full&fps=0` | **~18 fps decoded, ~5 MB/s, frame age ~50 ms** |
| same + `preview&size=1280x720` | ~5 fps (preview generation is slow on this unit — default **off**) |
| `/cgi-bin/faststream.jpg` (guest) | ~2.3 fps — do not use |

Alternatives considered and rejected for v1:

- **EventStream Client SDK (C++)** — no macOS package (Linux/Windows/RasPi
  only as of 1.2.2). The client is kept behind a small interface so an SDK
  backend can be added later (e.g. on Linux).
- **RTSP/H.264 (port 554)** — higher latency (GOP + client buffering), needs
  an extra decoder, and loses the JPEG comment metadata.
- **MxPEG** — lowest bandwidth but requires the Mobotix decoder; unnecessary
  on a LAN at these rates.

## Object tracking & detection

Goal: every train on the viaduct and every boat on the Spree is **one**
tracked object with a stable id, a tight bounding box, a center and a
velocity vector — from the moment it appears to the moment it leaves — as
smooth as the input allows. Cars and people are not of interest. There is
no appearance classification: the class of an object is the corridor it
travels in. Code lives in `src/tracking/` (no UI dependencies, reusable by
Steuerung); it runs in LIVE and PLAYBACK mode and headless in batch mode.

### Lanes and corridors

The geometry is configured, not detected. A **lane** is a four-corner quad
on one camera pane (`tracking.layouts[].lanes[]`, corners TL,TR,BR,BL in
pane UV) that is perspective-warped to an upright strip (at most
`laneMaxWidth` px wide, at least `laneMinHeight` px high) before analysis.
A **corridor** is the physical path an object follows (`train`, `boat`,
…); one or more lanes feed it, each covering the corridor range `s0..s1`
(lane x 0..1 maps linearly onto it, `s0 > s1` for a lane that sees the
path mirrored). Tracking itself runs in corridor-normalized coordinates:
`x` is the position along the corridor (0..1), `y` the lateral position
within the lane, sizes fractions of those, velocities fractions per second.
What gets *published* is those coordinates mapped onto the output canvas
(next section).

### Canvas placement

The output frame is a **canvas** in the proportions of the Lines facade
area (`tracking.canvas`, default 832×442; motion-path `size`), so the
consumer can lay the frame straight onto the lines. Each corridor is placed
on it with `tracking.corridors.<id>.placement {x0, x1, y0, y1}`: corridor
position 0 lands at canvas `x0`, position 1 at `x1`, lateral 0/1 at
`y0`/`y1` — so a corridor normally spans the full width (`x0` 0, `x1` 1) and
gets the vertical band the physical path occupies in the mirror image
(defaults: train `y` 0.15–0.40, boat 0.55–0.95). Values outside 0..1 are
allowed and *cut* the corridor: `x0 = -0.2` drops the first sixth of the
path, `x1 = 1.3` the last quarter; `x1 < x0` mirrors it. Published boxes are
clipped to the canvas, `clipped.left/right` is also set when the canvas edge
cuts the box, an object entirely outside the canvas is not published, and
`vx`/`vy` are scaled by the placement span (so `vx` stays "canvas widths
per second"; a mirrored placement flips its sign). The uncut corridor
values travel along as `corridor: {x, y, w, h, vx, vy}`. Placements are
applied when results are published — changing them never restarts the
detector or the tracks — and are edited on the canvas view (mouse) or the
`place` sliders; "save to config.json" stores them.

A **layout** is the set of lanes for one camera view: `panes` (1 = single
module, 2 = BOTH side by side) selects it, an optional `recordings` filter
(substring of the recording name) pins a layout to particular recordings.
Shipped layouts: `single` (one module, 1920×1080), `single_low` (the lower
camera angle of `rec_…14-18-08`) and `dual` (BOTH 3840×1080, where the
viaduct is seen by M1 and M2: `train_M1` covers `s` 0–0.58, `train_M2`
0.37–1.0, calibrated from a train visible in both — a train in the overlap
is one object, not two). Lane ends sit where the object physically becomes
(in)visible — the building left of the viaduct, the trees in front of the
S-Bahn platform, the tree line in M1 — so a train that fades out there is
treated as *left*, not *stopped*. "Visible" means visible *to the detector*:
in `single_low` the train lane ends at pane x 0.85, not at the last pixel of
track the eye can follow (0.906), because the last 5 % of the viaduct sit
behind the tree band whose foliage is in the busy map. With the lane end in
that band the head "vanished" mid-corridor and the box coasted; with the
end at 0.85 the head is pinned as *exiting* and released cleanly. Check the
end of a lane with `CAMTRACK_STRIPS` (below) — the mask, not the image,
tells you where the detector stops seeing.

Per-corridor parameter overrides (`tracking.corridors.<id>.flow` /
`.tracker`) adapt the shared pipeline: trains lock the heading to the
corridor axis and merge fragments aggressively; boats raise the MOG2
threshold and shrink the closing kernel (the wake is not the boat), and
suppress wake blobs behind the hull (`shadowGapFrac`/`shadowLateral`).

### Tuning notes (`rec_…14-18-08`, low camera angle)

What each shipped value fixes, measured on that recording — so that the
next change is made for a reason and not by feel:

- **Boat `minFlowPx` 0.6 → 0.3.** The distant boat moves ~1 px/frame in
  the strip; with the threshold at 0.6 the mask emptied at 41 s although
  the boat is visible until 56 s. This is the single most important knob
  for "the object disappears before it has left": the flow threshold is a
  *speed* threshold in strip pixels, and a receding object slows down in
  the image. Lowering it globally would admit water glitter and foliage
  everywhere, hence per corridor.
- **Boat `mogVarThreshold` 60 → 30.** The dark hull below the roof was not
  foreground; the box covered the roof only (h 0.29 → 0.37 of the lane).
- **Boat `minAreaFrac` 0.01, `minHeightFrac` 0.15.** Kills the glitter
  phantoms along the upper lane edge (h ≤ 0.14, area ≤ 0.0066); the boat
  never gets that small while inside the lane.
- **Boat `confirmFrames` 20** (base 9). A wake patch after the boat has
  gone moves coherently for ~14 frames; the boat itself is *entering* for
  seconds, so the later confirmation costs nothing visible.
- **Train `minHeightFrac` 0.25.** Removes the car glimpsed behind the
  viaduct (h 0.19); the entering train's first sliver is already h 0.26.
- **Train `preset` 2** (DIS medium). The fast preset underestimated the
  train's speed by ~45 % (flat, repetitive wagon texture); medium is
  within ~10 %, which the flow-velocity Kalman needs to place the free
  edge correctly (tail lag 0.20 → 0.03 of the corridor, x-jitter 2.2 →
  0.6 ‰). +2.3 ms/frame on this lane.
- **Fill 0.02×0.35 (train), 0.02×0.2 (boat).** Recovers roof / hull pixels
  that have foreground but no flow; the connected variant adds no phantoms
  (the box-shaped variant did — it picked up foreground trees above a car).
- **`posMeasNoise` 1e-3, `clipFrames` 3, `stopMinWidthFrac` 0.15,** see
  Tracker.

Limits of this pipeline (why the remaining gaps are structural, not a
matter of one more value):

1. *Motion is the only cue.* Flow ∧ foreground is decided per pixel, so a
   part of the object that has no texture (roof, hull in shadow) or moves
   too slowly is simply not there; the fill and the size-constancy logic
   paper over this but cannot invent the outline. A far-away, slowly
   receding boat will always shrink in the mask before it leaves.
2. *The busy map hides objects behind chronic motion.* Foliage in front of
   the viaduct is excluded permanently; a train behind it does not exist
   for the detector. Lane ends must be placed accordingly (see above).
3. *Velocity comes from flow, not from the box,* and the coupling is cut
   on purpose (a fragment must not flip the velocity). The price: a
   systematic flow bias (fast preset, repetitive texture) is never
   corrected from the box motion, and a track that has lost its
   detections coasts at the last flow estimate.
4. *Confirmation and stopping know size and motion, nothing about
   appearance.* Anything that moves coherently for `confirmFrames` and is
   large enough is an object; wake, glitter drift and cars behind the
   viaduct are held off by per-corridor size/speed limits, which is a
   dataset-specific fence, not a classifier.

Beyond that, the honest next steps are (a) a measured velocity from the
*box* (edge displacement over a few frames, weighted by how many edges
are free), fused with the flow velocity — this removes the bias and lets a
lost track coast on real motion; (b) a per-corridor appearance/size prior
for confirmation (a boat is this long and sits on the water line) so wake
and glitter are rejected on what they look like, not on how they move;
(c) if outlines matter more than cost, a light segmentation model on the
rectified strip instead of MOG2, seeded by the same flow blobs.

### Detector (`flow`, default)

`FlowMotionDetector` per lane: DIS optical flow on the rectified strip →
per-pixel EMA → a *moving* map (|flow| ≥ `minFlowPx`), a *sparkle* map
(fast, incoherent — water glitter, foliage) and a *busy* map (chronically
moving pixels, `busyThresh`, excluded) → optional MOG2 AND-mask
(`useMog2`, `mogVarThreshold`, dilated by `mogDilatePx`) → optional
**flow-seeded fill** (`fillWFrac`×`fillHFrac`, 0 = off): MOG2 foreground
that is *connected* to a moving blob joins the mask up to that reach
(morphological reconstruction, so the fill cannot jump across static ground
to an unrelated foreground patch). Flat, textureless parts of an object —
a train roof, a hull in shadow — give no usable flow but are foreground;
the flow blob is the evidence that something moves, the foreground draws
its outline. Velocity is still measured on the flow pixels only → opening +
anisotropic closing (`closeWFrac`×`closeHFrac` of the strip; wide along the
corridor to bridge wagon gaps) → connected components with **median flow
velocity and coherence** → directional union-find grouping
(`mergeGapFrac`, `mergeLateralOverlap`, `mergeAngleDeg`): blobs moving the
same way, close together, become one detection. Components touching a lane
end that is a corridor end are flagged clipped. `bgs` (plain MOG2) and
`yolo` (ONNX via OpenCV DNN, model in `bin/data/models/`, falls back to
flow when missing) are still selectable for comparison.

### Tracker

`MultiObjectTracker` per corridor: one Kalman filter per object over the
**box edges** `[x0, x1, y0, y1, vx, vy]`, with the flow velocity as a direct
measurement (position↔velocity cross-covariance is cut, so a box edge jump
can never flip the velocity). Edges pinned at a corridor end are held there
(clipping-aware transition: a pinned edge does not move with `v`; pin /
release re-seeds that edge, and needs `clipFrames` consecutive frames of
agreement — a single frame in which a fragment does not reach the lane end
no longer unpins and re-pins the edge, which was the dominant source of
box jitter). Association is greedy by cost (along-axis
gap, IoU, velocity difference) with a direction veto; leftover fragments
within `absorbGapFrac` are absorbed into the matched track (1:N); two
confirmed tracks travelling together for `mergeFrames` merge; a track that
dies and reappears where it was heading within `reacquireMs` inherits its
id (boats passing under the bridge). Size constancy: a detection much
shorter/longer than the predicted box (`sizeJumpTrust`) is treated as a
fragment — it cannot collapse the box or release a pinned edge in one step.
A confirmed, unclipped object that loses its detections at low speed far
from the corridor ends is held as `stopped` (v = 0) for `stoppedHoldMs`
and resumes with the same id only in the direction it arrived from. Only
an object at least `stopMinWidthFrac` of the corridor long may halt — a
car glimpsed behind the viaduct, a wake patch after the boat has gone, are
too short to be a train or a boat and must not be parked for two minutes.

Smoothing lives in the Kalman measurement noise: `posMeasNoise` (box
edges) and `velMeasNoise` (flow velocity). Raising `posMeasNoise` from
1e-4 to 1e-3 halved the vertical jitter of both objects without visible
lag; the along-axis position is dominated by the pinned edges and the
velocity anyway.

### Evaluation harness

Headless batch mode processes recordings unpaced (~80 fps single, ~40 fps
dual on an M-series Mac) and writes motion-path.v1 files plus a per-track
summary and a contact sheet:

```bash
CAMTRACK_BATCH=all CAMTRACK_OUT=$PWD/bin/data/eval/run1 \
  ./bin/YOUniverse_CameraTracking.app/Contents/MacOS/YOUniverse_CameraTracking
python3 tools/eval/score.py bin/data/eval/run1 --match class
```

`CAMTRACK_BATCH` is `all` or one recording name; `CAMTRACK_CONFIG`,
`CAMTRACK_RECORDINGS`, `CAMTRACK_RANGE=t0-t1`, `CAMTRACK_STRIDE`,
`CAMTRACK_SHEET_SEC`, `CAMTRACK_DEBUG=1` (per-frame `<rec>.debug.jsonl`
with raw detections and tracker state) and `CAMTRACK_STRIPS=<sec>` (every
`sec` seconds a `<rec>.strips/<rec>_<t>.jpg` with, per lane, the rectified
strip and the detector mask with detections drawn — the fastest way to see
*what the detector sees*, e.g. whether a roof is missing from the mask or a
lane end lies in foliage) refine a run. `tools/eval/events.json`
holds the hand-annotated pass-through windows; `score.py` reports per event
the ids seen (target 1), max simultaneous objects (target 1), coverage,
id switches and direction agreement, and per recording the false tracks,
box jitter and velocity roughness. Current state: all 18 events in the six
recordings pass with exactly one id each and zero false tracks
(`tools/eval/baseline_14-34-16.txt` keeps the pre-rewrite numbers:
14–37 ids per train, 73 false tracks).

### Output

`src/output/MotionPathWriter` turns a tracking snapshot into motion-path.v1
frames (the format of `B3S-FacadeEditor/packages/frontend/public/camera/
sample-motion-path.json`): frame `size` is the canvas (832×442 by default)
and all object coordinates are canvas-normalized (0..1, see *Canvas
placement*): per object `id`, `class`, `bbox` `[x, y, w, h]`, `x`/`y`
(center), `vx`/`vy`/`speed`, `heading`/`heading_deg`, `phase`
(`entering|crossing|exiting|moving|stopped`), `clipped`, `region`, `fill`, a
short `path`, and — extra — the uncut corridor-normalized `corridor {x, y,
w, h, vx, vy}` and, for overlays, the back-projected `panes[]` boxes.
`view` is the corridor id. With
`output.udp.enabled` the app sends one datagram per analysed frame to
`output.udp.host:port` (toggle in the GUI); the batch runner writes the
same frames under `{"kind": "recording"}`.

The debug overlay (`o`) draws per object, in a stable per-id color: bounding
box (per lane it is visible in), center point, velocity arrow (1 s
lookahead × `overlay.arrowScale`), fading trail, and `#id label phase
speed`. Tentative tracks are dimmed with a `?`.

### Threading & decode

The main thread only uploads textures and draws. Raw JPEG bytes are the
currency between threads; each consumer decodes what it needs:

| Thread | Work |
|---|---|
| stream thread | curl receive, JPEG framing, COM parse, display decode |
| playback thread | sidecar pacing, disk reads, display decode |
| tracking thread | own reduced decode (DCT-domain, `tracking.analysisReduce`, grayscale for flow/BGS, BGR for YOLO), lane warps, detectors + trackers |
| main thread | texture upload, UI, overlay |

All hand-offs are latest-only slots (drop stale, never queue). Display
decode uses `cv::imdecode` (libjpeg-turbo); `display.decodeScale: 2` decodes
at half size in the DCT domain (quarter cost, still larger than the
on-screen panes — overlay coordinates are normalized, so unaffected). The
stats bar shows per-stage timings (`dec`, `det`, `trk`).

## Brightness / sensor values

Mobotix embeds sensor data in the **JPEG COM comment** of every frame
(`SECTION SENSORS`), not in HTTP headers. Field mapping verified on this
firmware against `/control/camerainfo?text`:

| Field | Meaning | Conversion |
|---|---|---|
| `LA2` | **M3 MultiSense (Mx-F-MSA) illumination** — the brightness-control source | lux = LA2 / 10 |
| `LXL` / `LXR` | Left / right optical module illumination | lux = value / 10 |
| `PI2` | MultiSense PIR activity | percent 0–100 |
| `TC2` | MultiSense temperature | °C = TC2 / 10 |
| `TIN` | Camera board temperature | °C = TIN / 10 |
| `FRJ` | Camera-reported frame rate | fps = FRJ / 10 |

Every stream frame carries these values, so no separate polling is needed.
`JpegCommentParser` extracts them; toggle "show COM dump" to see the raw text.

## Notes / next steps

- The app never writes camera settings (`control?set…`) — per-stream
  parameters (`preview`, `size`, `quality`) only, so the camera's saved
  configuration stays untouched.
- Motion events (Video Motion on M1/M2, PIR on M3) are the next phase:
  camera-side events via IP Notify / MQTT / `event_env` (illumination source
  `cam2` = M3) rather than polling.
- Second S74 (west brightness): add its entry in `config.json` and extend the
  app to run one `MobotixMjpegClient` per camera.
