# YOUniverse_CameraTracking

Standalone openFrameworks 0.12.1 app for the Mobotix Mx-S74A camera on the
YOUniverse facade: it pulls the authenticated live MJPEG stream, tracks
trains and boats in configured corridors and publishes them as
motion-path.v1 frames on an output canvas.

Everything that does the work sits behind one GUI-free class,
`src/session/CameraTrackingSession` — stream, recording, playback, the
tracking worker, config load/save, the editor motion-file export and the
newest result as JSON (`resultJson()`). `ofApp` is only the window around
it (ofxGui, preview, lane editor, result views) and is temporary: the
production home is `YOUniverse_Steuerung`. The port is tracked in
[PORT_TO_STEUERUNG.md](PORT_TO_STEUERUNG.md).

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
Only the first camera with `"enabled": true` is used.

### Config document

`config.json` has exactly four top-level keys, and "save config.json" in
the window writes the whole document back from memory in that shape:

| Key | Content |
| --- | --- |
| `cameras[]` | `id`, `enabled`, `host`, `user`, `password`, `stream`, `fps`, `preview`, `previewSize`, `quality` |
| `recordingsDir` | recordings folder, relative to `bin/data` |
| `display` | `decodeScale` (1 full, 2 DCT half-size display decode), `arrowScale` (overlay velocity arrows) |
| `tracking` | `enabled`, `analysisReduce`, `laneMaxWidth`, `laneMinHeight`, `lens`, `flow`, `tracker` (incl. `trailLen`), `shape` (incl. `model`), `smoothing`, `canvas`, `corridors`, `layouts` |

Everything in it is edited in the file and takes effect on restart, with
two exceptions that the window edits and saves: the lane anchors
(`layouts[].lanes[]`) and the corridor placements
(`corridors.<id>.placement`). There are no tuning sliders. Older documents
with `output`, `tracking.detector`, `tracking.bgs`, `tracking.yolo` or
`tracking.overlay` still load (the overlay values fold into
`display.arrowScale` / `tracker.trailLen`); the next save drops them.

### Keys / GUI

- `r` — reconnect (also GUI button)
- `t` — toggle object tracking
- `o` — toggle the tracking debug overlay
- `m` — show the detector's motion mask behind the rectified lane views
- `d` — draw raw (pre-tracker) detections in grey
- `x` / `n` / `b` — the three result layers of a tracked object: the
  pixels currently attributed to it (`x`, off by default), its smoothed
  outline (`n`) and its bounding box (`b`). Each is independent; see
  *Object outline*
- `q` — result data (default) / raw tracker data. Result is what
  `resultJson()` and the motion export carry. For a boat the box and the
  center are the outline's bounds; the arrow is still the flow velocity,
  drawn from that center. Raw draws the flow box the tracker measures
  with. Trains stay on the flow box in both, because the outline covers
  only part of a train. See *Result and raw*
- `[` / `]` — select the previous / next lane for editing
- `v` — result view: output canvas (default) / rectified lane strips
- `,` / `.` — select the previous / next corridor for placing on the canvas
- `SPACE` — live: start/stop recording the incoming stream; playback:
  play/pause
- `e` — start/stop a motion-path export for the facade editor (both modes,
  see *Output*)
- `TAB` — toggle LIVE / PLAYBACK mode
- `LEFT` / `RIGHT` — previous / next recording (playback mode, wraps)
- `UP` / `DOWN` — seek +/-1 s within the current recording (playback mode)
- GUI "camera tracking" panel: playback-mode, record and export toggles,
  tracking/overlay toggles, `image scale`, the mask/detections toggles,
  the three result layers (`shape pixels`, `shape outline`, `bounding
  box`), the `result data (q)` toggle, result view switch (0 canvas / 1
  lane strips) and the reconnect button
- Playback panel (playback mode only, between the main and edit
  panels): `play` toggle, `playhead` slider to scrub, `prev frame` /
  `next frame` to step one frame (also while paused — the stepped frame
  goes through tracking and the export like any other), `prev recording` /
  `next recording`
- Edit panel: lane selector with the selected lane's corridor range
  `s0`/`s1`, corridor selector with the selected corridor's canvas
  placement `place x0/x1/y0/y1` (see *Canvas placement* below) and "save
  config.json", which writes the whole document (see *Config document*).
  Lens, detector, tracker, shape and smoothing values are config-only.
- Lane editor: every lane of the active layout is a strip on its pane
  (the selected lane is highlighted; grabbing a point selects that lane;
  overlay must be visible). The strip has a top border and a bottom border.
  With only the four corners it is the old quad. **Shift-click** a border
  (or double-click it) to add an anchor anywhere along that edge; the other
  border gets the matching point, and you drag either one. **Delete** or
  **Backspace** removes the selected anchor (not the four corners). Anchors
  stay inside the picture. Only pixels inside a lane are analysed. Each slice between anchors is
  perspective-warped into the next section of one upright rectangle before
  detection, so a lane can follow a curve and the tracking view below is
  still flat. A crossed/collapsed slice turns red and is ignored until it
  is convex again; releasing a point commits the lane (the worker rebuilds
  that layout's pipelines).
- Result view (below the video, `v` switches):
  - **canvas** (default) — the output canvas in the proportions of the
    Lines facade area (832×442) with a faint 10 % grid, every corridor's
    placement rectangle (dim where it lies outside the canvas — that part of
    the corridor is cut) and the tracked objects exactly as they are
    published: mapped through the placement and clipped at the canvas edge
    (the uncut box and outline are drawn dim behind them). Which layers are
    shown is up to `x`/`n`/`b`. The selected corridor has corner
    handles; drag a corner to resize, drag the body to move — grabbing
    another corridor selects it. The `place` sliders follow the drag and can
    be used instead of it.
  - **lanes** — below each pane one rectified strip per lane shows what the
    detector sees (with the motion mask behind it when `m` is on), the raw
    detections (`d`) and the tracked corridor objects mapped into that lane.
  The overlay on the photo above projects the same data back through the
  lane strip; red ticks on a box mark an edge pinned to a corridor end
  (object still entering / already leaving).

### Recording & playback

Recording taps the raw bytes of every complete JPEG on the stream thread
(before decode) and appends them unchanged to
`bin/data/<recordingsDir>/rec_YYYY-MM-DD_HH-MM-SS.mjpeg` (`recordingsDir`
in `config.json`, default `recordings`, created on demand; each recording
gets a fresh timestamp). A JSON sidecar (`rec_<same-stamp>.json`) with
per-frame `{offset, size, tMs}` entries is written on stop, so playback is
paced exactly like the original stream. No re-encoding — the JPEG COM
sensor metadata stays intact.

Playback (`TAB`) starts with the most recent recording, plays it to the end,
then cycles onward through all recordings newest to oldest (wrapping).
Arrow keys jump within that cycle; the playback panel pauses, scrubs and
steps single frames (a paused recording does not cycle on). While in playback mode the live stream
keeps running in the background but is ignored; `TAB` returns to it
instantly. Entering playback stops an active recording first.

Stats are logged every 5 s. `CAMTRACK_AUTOPLAYBACK=1` starts in playback
mode (recording playback + tracking without a reachable camera);
`CAMTRACK_RECORDINGS=<dir>` then picks the recordings folder, so a folder
holding one symlinked recording plays exactly that one.
`CAMTRACK_AUTOEXPORT=1` starts a motion export at launch.

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

### Lens correction

The camera lens is wide enough that a straight viaduct bows across the
frame, so a train's box changes shape as it passes. `tracking.lens`
undistorts that **before anything else** — before the picture is shown and
before the lane warp — with OpenCV's equidistant fisheye model. The image
in the window is the corrected one; lane corners are drawn and stored in
that same image, not in the raw camera frame.

A side-by-side BOTH frame is two lenses: each half is corrected about its
own center. One coefficient set is shared (the modules are the same lens).

| Key | Meaning |
| --- | --- |
| `enabled` | off copies the camera frame through |
| `fovDeg` | horizontal field of view of one pane, in degrees. This is the lens, not a crop: the Mx-O-M7SA-8DN050 modules are about 93°. Too high and the edges stretch |
| `k1`..`k4` | how the lens differs from an equidistant fisheye. These take out the bow `fovDeg` alone leaves |
| `frameFovDeg` | horizontal field of the picture, in degrees. 0 spans the whole sensor: the full width of the center row and the full height of the center column, square pixels, so the picture is wider than the sensor's own aspect. A smaller value narrows it and stays inside the sensor |
| `centerX`, `centerY` | optical center as a fraction of the pane, when the bow is not symmetric |

The picture is that rectangle and nothing around it. It does not keep the
sensor's aspect ratio: with this lens the center row is about a third
wider than a 16:9 crop of the same height, and the corners of the sensor
that flare past the rectangle are dropped. Lane UV `0..1` is this
rectangle. A config saved before `frameFovDeg` existed still has its
anchors in the old 16:9 crop; loading it moves them across, and the next
save writes the key. `balance` is no longer read.

The values are config-only (`tracking.lens`, read at startup) and reach
both the picture and the tracker. `fovDeg` 92.55 with `k1` 0.146 and
`k2` -0.143 is the fit for this lens: the viaduct anchors, which follow
the deck, sit on a straight line to about 1.5 px. What is left of the
slope is perspective — the track is not square to the camera — and no
lens model removes that. The main panel's `image scale` only changes how
large that picture is drawn (1 fits it in the video area). It does not
change the lens or the tracking.

### Lanes and corridors

The geometry is configured, not detected. A **lane** is a strip on one
lens-corrected camera pane (`tracking.layouts[].lanes[]`). `quad` is the
four ends, TL, TR, BR, BL in pane UV. To follow a curve, add
`top` and `bottom`: two polylines of the same length, left to right, pane
UV. The first and last points are the quad's ends (`top` TL→TR, `bottom`
BL→BR); the entries in between are the anchors you add in the editor.
Omit `top`/`bottom` when the lane is just the four corners. Each slice
between anchors is perspective-warped into the next section of one
upright rectangle (at most `laneMaxWidth` px wide, at least
`laneMinHeight` px high). Along-track position in that rectangle is the
centreline length, so a bend keeps its share of the width. The lane view
below the video, the detector and everything published (corridor `x`/`y`,
the canvas box, `resultJson()`, the motion file) stay on that flat rectangle. The
overlay on the photo maps them back through the same slices, so the box
follows the curve on screen and is rectangular in the result.
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
`place` sliders; "save config.json" stores them.

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
   on purpose (a fragment must not flip the velocity). The systematic
   flow bias (repetitive texture) is now corrected by the learnt flow
   gain (see *Tracker*), measured on free edges only — so an object that
   never shows a free edge in the lane (corridor-long, both ends pinned
   throughout) still runs on the corridor's gain from earlier objects,
   or on the manual `flowGain`. The larger speed error is geometric: a
   corridor unit is not the same distance everywhere along `s` (see *What
   the gain cannot fix*), so a hidden train coasts at the rate of where
   it was last seen.
4. *Confirmation and stopping know size and motion, nothing about
   appearance.* Anything that moves coherently for `confirmFrames` and is
   large enough is an object; wake, glitter drift and cars behind the
   viaduct are held off by per-corridor size/speed limits, which is a
   dataset-specific fence, not a classifier.

Beyond that, the honest next steps are (a) ~~a measured velocity from the
box fused with the flow velocity~~ — done as the flow gain, see
*Tracker*, which in turn showed that the remaining speed error is a rate
profile along `s`, to be measured from free edges per position and
applied to the coast; (b) a per-corridor appearance/size prior
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
end that is a corridor end are flagged clipped. **Direction split**
(`dirSplit`, default on): the components are labelled on an undirected
mask, so two trains passing each other are one component with one
(meaningless) median velocity. A component whose flow-seed pixels move
both ways along the lane — the minority sign a blob in its own right
(≥ `minAreaFrac`) and above the noise floor (≥ `dirSplitMinFrac`, 5 %, of
the signed pixels; a train entering behind a corridor-long one is only a
few percent) — is cut into a +x and a −x part — each sign's seed
pixels grown by the closing kernel inside the component and off the other
sign's pixels — and each part is measured on its own. Single-direction
components are untouched. Flow is the only detector; the experimental
plain-MOG2 and YOLO-as-detector variants were removed (the YOLO
segmentation model lives on as the shape estimator's silhouette source).

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
within `absorbGapFrac` that move at the track's speed (faster/slower
ratio ≤ `absorbSpeedRatio`) are absorbed into the matched track (1:N); two
confirmed tracks travelling together for `mergeFrames` merge, under the
same speed-ratio rule. The ratio is loose for trains (3: a low-texture
wagon fragment may read slow in the flow and must still join) and tight
for boats (1.4 in the shipped config): a wake travels at about half the
hull's speed, and a wake absorbed — or merged in as a tentative track,
bringing its lane-end pin along — puts the box's rear edge on the wash,
so the box stretched until the hull left (`rec_…15-48-29`); a track that
dies and reappears where it was heading within `reacquireMs` inherits its
id (boats passing under the bridge) — except one that coasted out past
the far end: it has left the corridor, and nothing at that lane end can
be it again. Size constancy: a detection much
shorter/longer than the predicted box (`sizeJumpTrust`) is treated as a
fragment — it cannot collapse the box or release a pinned edge in one step.
A confirmed, unclipped object that loses its detections at low speed far
from the corridor ends is held as `stopped` (v = 0) for `stoppedHoldMs`
and resumes with the same id only in the direction it arrived from. Only
an object at least `stopMinWidthFrac` of the corridor long may halt — a
car glimpsed behind the viaduct, a wake patch after the boat has gone, are
too short to be a train or a boat and must not be parked for two minutes.
Boats set `stopEnabled` false: a hull does not halt in the river, and the
blob that does is the wake. It is deleted with its misses instead of
being parked, and a slow death is not put in the reacquire graveyard, so
the wash cannot inherit the id. A settled track also refuses a detection
that is the wash behind it — opposite the direction it voted in, slower
than `absorbSpeedRatio` allows, or lying entirely behind its rear edge
(`rec_…16-04-00`, the boat that left and whose track then wandered back
with the wake and stopped mid-corridor).

Smoothing lives in the Kalman measurement noise: `posMeasNoise` (box
edges) and `velMeasNoise` (flow velocity). Raising `posMeasNoise` from
1e-4 to 1e-3 halved the vertical jitter of both objects without visible
lag; the along-axis position is dominated by the pinned edges and the
velocity anyway.

**Flow gain** (`flowGain`, `flowGainLearn`, `flowGainWindow`,
`flowGainSamples`, `flowGainMin` / `flowGainMax`, `flowGainAlpha`). The flow velocity is
biased *low* on repetitive texture: DIS matches a wagon side one period
short, the median over the blob is taken by those pixels, and the filter
cannot correct it from the box because the position↔velocity coupling is
cut on purpose (fast preset ≈ 45 % slow on trains, medium ≈ 10 %). A
hidden train coasted on that value and fell further behind the longer it
was covered. A *free* box edge, however, is placed by the mask and
crosses the lane at the object's true speed. Every flow velocity that
enters the filter is therefore multiplied by a gain: `flowGain` is the
manual value (per corridor via `corridors.train.tracker.flowGain`) and
the prior; with `flowGainLearn` the gain is measured. An edge counts as a
speed sample in a frame only if it is unpinned before and after the
update, not held or hidden by an oncoming object, not part of a fragment
or inflated blob, at least `edgeBirthFrac` from the lane end on its
side (the mask frays where the object fades into foliage), and more than
0.1 corridor from an *internal* lane boundary (`dual`: the `train_M1` /
`train_M2` ends at 0.58 and 0.37 — a blob edge there is cut by the lane
geometry and stalls while the train does not; in `rec_…14-43-02` those
stalls alone read as a 0.75 "bias"). The raw blob
edge is used, not the silhouette-trimmed one — a silhouette ageing in and
out would put a step into the window. Over `flowGainWindow` consecutive
clean frames the least-squares slope of the edge is the box velocity; its
ratio to the mean raw flow over the same window is one gain sample,
accepted if the fit's residual RMS is under 1 % of the corridor (a
fraying edge is not a speed), both agree on the sign, and both are
above `minSpeedConfirm`; clamped to `[flowGainMin, flowGainMax]`. The
track's gain follows the *median* of its last `flowGainSamples` accepted
ratios at rate `flowGainAlpha` (a plain EMA was dragged by stalled-edge
outliers) and a corridor-wide gain follows it at a quarter of that; a new
track starts with the corridor's gain, so a train entering behind another
one starts out corrected. `lastMatchVx`
and the occluded coast read the corrected velocity, so a hidden train
travels at its real speed. Nothing changes with `flowGainLearn` off and
`flowGain` 1. The boat corridor turns learning *off*: a hull is smooth,
its flow is not texture-biased, and the boat blob is not the hull — the
mask runs over the disturbed water ahead of the bow, whose front edge
moves ~20 % faster than the boat (measured on `rec_…15-48-29`), so the
learnt gain over-corrected boats to 0.86 of their real speed. Trains in
the single layouts measured 1.00–1.03 with learning on, from 1.02–1.09
without (the published edge is attenuated toward 1 by the silhouette
trim, so the raw flow under-read is somewhat larger than those numbers),
with all gates and jitter unchanged. `vbias` in
`tools/eval/score.py` reports both the ratio and the applied gain (`fg`
in the debug dump).

*What the gain cannot fix.* Measured on the raw blob edges, the ratio
depends far more on *where* the edge is than on the flow: in `single` the
same train reads 1.15–1.3 at its left edge and 0.8–0.95 at its right edge
in both directions, i.e. a given physical speed is ~25 % fewer corridor
units per second at `x ≈ 0.8` than at `x ≈ 0.4`; in `dual` both ends run
at ~0.6 of the middle (a train fully in view has both edges at 0.57 of
its flow). That is the lane mapping — the quad is a homography of the
track plane, the train is a tall body seen obliquely, and the lens
residual adds to it — not the flow. The learner therefore sees
rate(edge) / rate(blob mass): per-track gains swing 0.7–1.6 within one
pass as the clean edge changes, in `single` the net effect is the
+2–9 % above, in `dual` the clean edges are the compressed ends and the
gain settles at 0.82–0.95 (hidden trains coast slower; all gates still
pass, `omiss` rises only because the occluded phase lasts longer). This
position dependence is also the likely remaining cause of a hidden train
looking slow: it coasts at the corridor rate of where it was last seen
(often an end), while the visible train in front moves at the middle
rate. The remedy is a rate profile along `s` (or lane corners placed on
the train's own height), not a scalar gain.

**Lifecycle.** A track is *tentative* until it passes every confirmation
gate, *confirmed* while it is seen, *occluded* while it coasts unseen,
*stopped* while it is parked, and (from the output smoother only) *gone*
while it fades out. Only confirmed objects are published.

**Candidates and verified objects.** The two layers are that one flag.
A track starts as a *candidate* (`confirmed == false`, drawn with a ` ?`
suffix) and becomes a *verified object* only when every gate below holds
in the same frame. Candidates are what the water glitter is: a few frames
of motion, no hull, dead after `tentativeMaxMisses` misses (default 3).
They are not a second tracker — suppressing their creation would blind
`hiddenRear` (an object appearing from behind one already verified) and
id reacquisition. The window never draws them; the batch contact sheets
keep drawing them (a tuning artefact, with a ` ?` suffix). Nothing else
does: the output smoother, `resultJson()`, the editor export and the
motion file all drop `!confirmed`. Measured on all eleven annotated recordings (shipped
config, old layouts for the September set): 40–140 candidates per
recording, almost all of them in the boat corridor, and not one
confirmed outside an annotated window — the 31 confirmed tracks are
exactly the 31 annotated passes (`false=0` everywhere).

A candidate confirms only when all of these hold (`tracking.tracker`,
overridable per corridor):

- **Lifetime** — `confirmFrames` matched frames (boat corridor: 20, about
  0.7 s) with a settled direction (`dirVotes` ≥ min(`confirmFrames`, 3))
  and speed at least `minSpeedConfirm`.
- **Entry** — `edgeRule`: born clipped at a corridor end, or with its box
  within `edgeBirthFrac` of one (boat 0.08, train 0.2). Exempt when the
  track inherits an id, is born within `edgeGraceMs` of a reset, or
  (`hiddenRear`) appears from behind an already confirmed object.
- **Length** — `minConfirmLen` (train override 0.12 of the corridor).
- **Outline** — `modelGate`: the segmentation model has found the
  corridor's class on it at least once, and has not labelled another
  class at least `modelGateOtherHits` times and more than twice as often.
  No effect when the model is off.
- **Not a shadow** — `shadowSuppress`: not a fragment or wake travelling
  beside a confirmed object in the same direction (`shadowGapFrac`,
  `shadowLateral`); once flagged, it stays a shadow.

The gates in detail:

- **Edge rule** (`edgeRule`, `edgeBirthFrac`, `edgeGraceMs`). Trains and
  boats come in through a corridor end. A track confirms only if it was
  born clipped at an end or with its box within `edgeBirthFrac` of one;
  a track that inherits an id, or is born within `edgeGraceMs` of a reset
  (the object was already there), is exempt. Water glitter, a moored boat
  rocking, a wake fragment, a car glimpsed behind the viaduct — all of
  them appear in mid corridor and stay tentative forever. In the dual
  layout a train from the right is first detected at corridor x ≈ 0.85
  (the far lane end lies in foliage), hence the train override
  `edgeBirthFrac` 0.2; boats use the default 0.08.
- **Length gate** (`minConfirmLen`, train override 0.12). A track must be
  at least this long (corridor fraction) to confirm: a car on the road
  behind the viaduct is 0.03–0.05 of the corridor and never grows, a
  train has passed 0.12 after two or three seconds of entering — still
  off-canvas with the current train placement. (A permanent "let go of
  the end while short" flag was tried and dropped: a train whose nose
  flickers off the lane end for three frames must not be banned for its
  whole pass.)
- **Model gate** (`modelGate`, `modelGateOtherHits`; needs the
  segmentation model, see *Object outline*). The shape estimator runs the
  model on edge-born tentative tracks too (`shape.modelTentative`) and
  hands the tracker, one frame late, how often it found an instance of
  the corridor's class on the track and how often it saw only another
  class there. A track the model has never found does not confirm, nor
  one it has labelled another class at least `modelGateOtherHits` times
  and more than twice as often as its own class. The verdict is
  re-evaluated every frame — the nose of a train at the lane end reads
  "truck" for the first few calls, so an early verdict must not be final.
  The gate also works at the far end (`modelLostMs`): a confirmed object
  that is leaving (front at the end it heads for) on which the model has
  seen nothing at all — neither its own class nor another one — for that
  long is no longer fed by the motion blobs at that lane end; it ages out
  and coasts away as `occluded`. Sun glitter and wake at the lane edge
  kept a departed boat "exiting" for 45 s before this; a boat the model
  reads as "surfboard" is still something and keeps being tracked.
  Nothing changes when the model is off.
- A per-lane `yBand: [y0, y1]` (lane fractions, in the layout) drops
  detections whose vertical centre lies outside the band before tracking
  — for a quad that has to stay generous because of the lane height
  floor while the road band behind the viaduct should be ignored.

**Occlusion coasting** (`coastOccluded`, `occludedMaxMs`,
`occludedGateGrow`). A confirmed object that loses its detections while
moving is hidden, not gone: another train in front of it, a mast, the
sun on the water. After `maxMisses` it switches to `occluded` and coasts
at the velocity it was last seen with, size frozen (its rear pin is
released so the box does not grow without end — unless an oncoming object
still overlaps it, in which case the pin is the hidden rear and stays), until a detection moving
*its* way overlaps the prediction — then it continues under the same id,
corrected to the new box — or its trailing edge has passed the far
corridor end. The direction veto is unconditional while hidden: the
visible opposite-direction train passing in front can never drag the
hidden one along. The association gate widens by `occludedGateGrow` per
hidden second; a fresh tentative track that emerges where the hidden
object should be (same direction, lateral overlap, within
`reacquireGapFrac` plus the growth) is adopted by it. A hidden object
that is *leaving* (front pinned at the end it heads for, or within
`stopEdgeFrac` of it) only coasts out: it is neither adopted into nor
merged with a tentative track at that end, so water glitter at the lane
edge cannot keep a departed boat alive. An object that vanished *slowly*
is still parked as `stopped` as before; `occludedMaxMs` is the safety
cap. The stop rule is unchanged otherwise.

**Shape-driven box** (`shapeBox`, `shapeBoxMaxAge`, `shapeBoxTrim`,
`shapeShrink`, `shapeShrinkFrames`). While
a model silhouette no older than `shapeBoxMaxAge` frames exists for a
confirmed object, its bounding box *trims* the motion blob before the
blob becomes the Kalman position measurement: the tracker box is pulled
toward the hull, not the wake the closing glued to it (a boat box with wake was
~3× the hull), the roof, not the dilated foreground. The blob stays the
outer bound and the anchor — the silhouette lives in the box's own
(padded) frame, so taken as the measurement itself it could only ever
reproduce the box it was measured in and crept inward wherever the mask
fell short of the crop border; a corridor-long train lost two thirds of
its box that way. A silhouette edge therefore counts only inside the
blob and may take at most `shapeBoxTrim` of the blob's length / height
off each end; an edge the blob has at a corridor end stays pinned there.
The cap is 0.25 for trains and 0.45 for boats (shipped config): a boat is
short and the model finds the hull reliably, while the wash behind the
stern is often one blob with the hull and up to twice its length, so a
quarter of the blob is not enough to get back to the hull. What the cap
cannot undo is a single model frame that takes the white wash for boat:
the box jumps out at once and the size-constancy rule then holds the
inflated length (`rec_…15-48-29` at 23 s). `shapeShrink` is the way back
out: once the silhouette has ended where the blob ends — both sides,
within a fifth of the blob's length, both ends free of the corridor ends
and of any occluder — for `shapeShrinkFrames` running frames, the short
measurement is trusted and the box follows it instead of being held. A
silhouette that runs *past* the blob never qualifies (the model sees more
than the mask: a train whose flow mask broke up), so the hold stands for
trains even with it on; it ships off everywhere and on for boats. On that
recording the box stayed ~0.36 wide until the hull left; with the shrink it
narrows as soon as the wake separates (0.38 at 26 s, 0.25 at 31 s) and is
on the hull by the time the boat leaves (0.19 at 33 s, against 0.31 without
it). The convergence takes a few seconds
because the corrected measurement still goes through the filter.
Merge priority uses the same feedback: a track with a model outline is
never folded into one without — the survivor is the one the model found.
These are the only three couplings from the shape estimator back into the
tracker (`ShapeFeedback` in `TrackingTypes.h`); with the model off the
tracks are identical to before.

**Hidden rear / front** (`hiddenRear`). Two trains meeting on the
viaduct: the far one is partly or entirely behind the near one. A train
that is first detected in mid corridor with only its nose showing did
not appear out of nowhere — if the spot behind its rear edge is covered
by an oncoming visible confirmed object, it came in behind that one, so
its rear is pinned at the entry end it must have come through (and it is
edge-born). A pinned rear also stays pinned while the oncoming object
still covers the spot where the blob's rear edge is: the blob ends where
the occluder begins, not where the object does. The mirror case is the
nose disappearing behind the oncoming train: a blob front edge that lies
inside the oncoming object's box is ignored and the nose coasts at the
object's velocity (clamped to the corridor) instead of being pushed back
by the other train's tail. Once the occluder has moved on, the edge is
measured again and the output smoother glides the box to its real
extent. A detection right next to a confirmed same-direction object is a
fragment of it and never gets a hidden rear; a sliver sitting entirely
inside the oncoming box (or shorter than `minConfirmLen`) is the same.
The decision waits until the tentative track's direction has settled
(three agreeing frames), not at birth — a wake fragment's first velocity
sample can point the wrong way. It is not final while the blob is still a
short sliver sitting in the oncoming box: the nose of a train coming out
from behind another stays that way for about a second
(`rec_2026-10-12_16-29-11` near frame 1100, once the lane hugs the rails),
and giving up on that first look left the track unconfirmed for the whole
pass. The check is repeated until the blob is long enough to pin, or until
the rear is clearly not hidden (already in the lane, a same-direction
fragment, or a full-size blob with nothing oncoming behind it).

### Object outline

Besides the box, every confirmed object carries a silhouette: `ShapeEstimator`
(`tracking.shape`) keeps per object a `gridW × gridH` (96×32) occupancy
grid pinned to the tracked box (plus `padX`/`padY` margin), so the shape
rides along with the Kalman box and stays put when the object does. Each
frame the grid is refreshed from a *shape source* and blended with an EMA
(`ema` 0.25, the first observation seeds it); the outline is the largest
contour of the thresholded (`threshold`), 4× upsampled, morphologically
closed (`closeCells`) grid, simplified with `approxPolyDP` (`simplify`, in
cells). Pixels, outline and box are independent display layers (`x`/`n`/`b`)
and the outline is published (see *Output*). A `stopped` or `occluded`
object keeps its last shape. The tracker never reads the shape itself; it
only gets the narrow `ShapeFeedback` described under *Tracker* (model
gate, shape-driven box, merge priority) — with the model off, enabling or
disabling the estimator leaves the tracks identical.

The three sources (`source`):

| `source` | what fills the grid | character |
| --- | --- | --- |
| 0 `mask` | the detector's final motion mask (closed, dilated) | blobby, follows the box |
| 1 `foreground` | the raw MOG2 foreground, before closing | finer, but gaps where the object is untextured (roof, deck) |
| 2 `model` (default) | an instance segmentation model run on the colour crop of the tracked box | the actual silhouette — the stern cabin of a boat, the gap between two coupled cars |

The model is YOLOv8n-seg, run through the bundled OpenCV `dnn`
(`models/yolov8n-seg-192x640.onnx`, 13.7 MB). It is exported with a fixed
**rectangular** input (`modelInputW × modelInputH`, 640×192) because a
train in the lane is ~6:1 — letterboxed into a square it would be a 70 px
strip and the model finds nothing. The crop is the tracked box plus
`modelPad` (25 %) of margin, in the lane's own pane; all instances that
overlap the box (IoU ≥ `modelMinIou`, or mostly inside it and at least 3 %
of its area) are unioned, so a train split into two instances still yields
one shape, and the mask is warped back through the lane homography. With
`modelClassGate` (default on) only instances of the corridor's class count
— COCO `train` for the train corridor (every train in the evaluation
recordings is labelled `train` from its first partial sighting on; the one
`bus` hit was a vehicle on the road), `boat` for the boat corridor
(`ShapeEstimator::allowedClasses`); the strongest instance of any other
class on the box is reported to the tracker as an *other-class hit*
(the model gate rejects a track that is only ever "car"). With the gate
off, class labels are ignored as before. The forward pass costs ~25–30 ms on an M2
Max, so it runs every `modelEveryN` (2) frames, staggered by object id, at
most `modelMaxPerFrame` (2) objects per frame; the EMA bridges the frames
in between.

A crop wider than `modelMaxAspect` (6.5 — about twice the input's 3.3) is
cut into tiles with `modelTileOverlap` (15 %) overlap, and **one tile per
model call**, round-robin per object. Without this a full-length train
(crop ~1800×160) letterboxed into 640×192 is a 57 px ribbon and the model
returns nothing at all — the outline tracked fine while the train was
short and collapsed the moment it spanned the viaduct (`14-29-12` at
t ≈ 40 s). Two tiles make it ~110 px tall and the model finds it on both
(2/2 hits, 85 % of the length covered, versus 0/1); more tiles cost more
and lose the end cars at the seams. Cells outside the current tile are
*unobserved*, not empty, so the other half of the train keeps its last
state; the whole object is refreshed every `modelEveryN × tiles` frames.

If the model finds nothing for `modelHoldFrames` (45) frames the grid
falls back to the closed detector mask (the solid bar in the lane strip)
until the model returns. A grid that holds a model silhouette is blended
toward that bar at `fallbackEma` (0.06) rather than `ema`: at 0.25 the raw
MOG2 fallback used to erase a train in three frames and leave one blob
(the overlay label then reads `shape:model+mask`). If the model file is
missing or fails to load, the estimator logs once and uses the mask
throughout. Batch average with two objects at most: ~6 ms/frame
(`avg_shape_ms` in the summary, `shp` in the stats bar); tiling does not
change it — a call is still one forward pass.

Re-export the model (needs Python + `ultralytics`; the bundled `cv2` is not
required):

```bash
python3 -m venv /tmp/yoloseg && . /tmp/yoloseg/bin/activate && pip install ultralytics
yolo export model=yolov8n-seg.pt format=onnx imgsz=192,640 opset=12 simplify=True
# -> yolov8n-seg.onnx, copy to bin/data/models/ and set tracking.shape.model
```

`tracking.shape` is read at startup and can be overridden per corridor
(`tracking.corridors.<id>.shape`). Limits: at night (IR, the train is a row
of lit windows) the model mostly fails and the mask fallback gives the
detector's bar rather than a silhouette — the boat still segments well; the model sees
only what the tracked box covers, so when the motion detector lags behind
an untextured stern the outline can extend beyond the detections but not
beyond the box plus `padX`.

### Output smoothing

`OutputSmoother` (`tracking.smoothing`) sits between the tracker and the
publisher, per corridor. The tracker's job is to be right; this stage's
job is that whatever is published never jumps — nothing feeds back.

- **Box edges** slide toward the tracker's edges at most `boxRate` (0.25
  corridor fractions/s) on top of the object's own travel, so a merge, a
  pin/release re-seed or a silhouette trim is a slide, not a jump. An
  edge the tracker pins at a corridor end does not travel with the
  object (it sits at the end), but it glides there at the same rate when
  it is pinned — a nose reaching the lane end widens the box over a few
  frames, not in one.
- **Outline**: the tracker outline is resampled to `outlineVertices` (32)
  points by arc length — walked clockwise from the point where the
  boundary crosses the horizontal centre line on the right, so vertex k
  lies on the same part of the object frame after frame — and each
  vertex is low-passed in box-normalized coordinates (`outlineAlpha`,
  0.3). The outline is never empty: until the first silhouette arrives it
  is the box rectangle, while the model is silent it is the last
  silhouette, riding along on the smoothed box.
- **Presence**: a new object fades in over `fadeInMs` (300), an object the
  tracker dropped keeps travelling at its last velocity and fades out over
  `fadeOutMs` (400) with phase `gone`; `presence` (0..1) is published per
  object. An id that comes back while fading (reacquire) ramps up again.

### Result and raw

`q` (GUI: `result data`) switches every view between the published result
and the tracker's own box. Default is the result, which is also what
`resultJson()` and the motion export carry.

For a **boat**, once the drawn outline is a real silhouette and not the
rectangle stand-in, the published box is that outline's bounds and the
published center is the center of that box. The velocity stays the flow
velocity, so the arrow reports how the boat is moving but starts on the
hull. The flow box is not stretched back to a corridor end: while the
boat is only partly in frame the box is the visible hull, and `clipped`
still says the object continues past the end. Until the first silhouette
the outline is the flow rectangle, so result and raw match. The outline
eases onto the hull, so the published box follows over a few frames. The
path follows the result center. Smoothing of that box itself is the next
step; this view is the place it will show.

**Trains** keep the flow box as the result. The model outline usually
covers less of a train than the motion box, and publishing it would make
the train too short. Raw and result look the same for a train.

Raw (`q` off) draws the flow box, its center and its trail. The outline
is the same polygon in both modes, so the two can be compared directly.
The flow box is never fed by the outline. The stats bar shows `result` or
`raw`.

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
(center — for a boat, the center of the outline bounds; see *Result and raw*),
`vx`/`vy`/`speed`, `heading`/`heading_deg`, `phase`
(`entering|crossing|exiting|moving|stopped|occluded|gone`), `presence`
(0..1 fade ramp, see *Output smoothing*), `clipped`, `region`, `fill`, a
short `path`, and — extra — `outline` (the object's silhouette as a closed
polygon `[[x, y], …]` in canvas coordinates, clipped to the canvas like the
box; empty until the shape has its first observation or when
`tracking.shape.enabled` is false), the uncut corridor-normalized `corridor
{x, y, w, h, vx, vy}` and, for overlays, the back-projected `panes[]` boxes.
`view` is the corridor id.

**`resultJson()`** on `CameraTrackingSession` is the newest such frame
(`frame` counts up from 1 per session, `fps` is the camera's configured
rate or, when that is 0, the recording's frame spacing / the measured
stream rate, `camera` is `"live"` or the recording name). It is rebuilt
on the main thread each time the tracking worker publishes a new
revision — `resultRevision()` tells a consumer when to read it. There is
no network output; the host application decides what to do with the
object. The batch runner writes the same frames under
`{"kind": "recording"}` (`tools/eval/score.py` reads `view`, `class` and
`clipped` from them).

**Export motion (`e`)** writes a file for the facade editor instead: the
same canvas-space objects cut down to what its parser reads — root
`schema`, `source`, `fps`, `duration_s`, `size`, `frames`; per frame `t`
(seconds from the first exported frame) and `scene {vx, vy, speed}`; per
object `id`, `x`, `y`, `vx`, `vy`, `speed`, `phase`, `presence`, `fill`,
`bbox` and the `outline` polygon (`path` and the debug extras are left
out). Every start
opens a new `recordings/motion_<source>_<stamp>.json`, written on stop or
quit; it works in live and playback mode alike and is independent of the
MJPEG recording. A source change under an open export (live ↔ playback,
the player advancing to the next recording) closes the file and starts the
next one, so each file is one continuous take with stable ids.
`CAMTRACK_AUTOEXPORT=1` starts an export at launch (headless runs).

The debug overlay (`o`) draws per object, in a stable per-id color: the
shape pixels (`x`), the outline (`n`) and the bounding box (`b`) — each per
lane it is visible in — plus center point, velocity arrow (1 s
lookahead × `display.arrowScale`), fading trail (`tracker.trailLen`
frames), and `#id label phase speed shape:<source>`. With `result data`
on (the default) the box and the center are the published ones; `q`
shows the flow box instead. Only confirmed objects are drawn — see
*Candidates and verified objects*. The batch contact sheets draw the
outline together with the box, candidates included.

### Threading & decode

The main thread runs `CameraTrackingSession::update()` (poll the newest
frame, lens-correct it, poll the newest results, build `resultJson()`,
feed the export) and then draws. Raw JPEG bytes are the currency between
threads; each consumer decodes what it needs:

| Thread | Work |
|---|---|
| stream thread | curl receive, JPEG framing, COM parse, display decode |
| playback thread | sidecar pacing, disk reads, display decode |
| tracking thread | own reduced decode (DCT-domain, `tracking.analysisReduce`, grayscale for flow, BGR when the shape model is on), lane warps, detectors + trackers |
| main thread | session update, texture upload, UI, overlay |

All hand-offs are latest-only slots (drop stale, never queue). Display
decode uses Apple ImageIO (`util/JpegDecode`); `display.decodeScale: 2`
decodes at half size in the DCT domain (quarter cost, still larger than
the on-screen panes — overlay coordinates are normalized, so unaffected).
The stats bar shows per-stage timings (`dec`, `det`, `trk`, `shp`).

### Source layout

| Folder | Content | Moves to Steuerung |
|---|---|---|
| `src/session/` | `CameraTrackingSession` — the whole module behind one class | yes |
| `src/mobotix/` | MJPEG client, JPEG COM parser, camera types | yes |
| `src/recording/` | `MjpegRecorder`, `MjpegPlayer` (`RecordingReader` is batch-only) | player + recorder |
| `src/tracking/` | lens → lane warp → flow detector → grouping → tracker → shape → smoothing, config JSON | yes |
| `src/util/` | lens corrector, JPEG decode | yes |
| `src/output/` | motion-path.v1 builder + recording writer | yes |
| `src/batch/`, `tools/eval/` | headless evaluation harness | stays here |
| `src/ofApp.*` | the temporary ofxGui window | rewritten in Steuerung's UI |

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
`JpegCommentParser` extracts them into `SensorSnapshot`
(`CameraTrackingSession::getSensors()`); the stats bar shows the camera
fps and the MultiSense lux, and the 5 s log line carries the lux too.

## Notes / next steps

- The app never writes camera settings (`control?set…`) — per-stream
  parameters (`preview`, `size`, `quality`) only, so the camera's saved
  configuration stays untouched.
- Motion events (Video Motion on M1/M2, PIR on M3) are the next phase:
  camera-side events via IP Notify / MQTT / `event_env` (illumination source
  `cam2` = M3) rather than polling.
- Second S74 (west brightness): add its entry to `cameras[]` and extend the
  session to run one `MobotixMjpegClient` per camera.
- The port into `YOUniverse_Steuerung`: [PORT_TO_STEUERUNG.md](PORT_TO_STEUERUNG.md).
