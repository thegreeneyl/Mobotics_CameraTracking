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
- `SPACE` — start/stop recording the incoming stream (live mode only)
- `TAB` — toggle LIVE / PLAYBACK mode
- `LEFT` / `RIGHT` — previous / next recording (playback mode, wraps)
- GUI: preview toggle, JPEG quality (preview only), COM dump toggle,
  record toggle, playback-mode toggle

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
