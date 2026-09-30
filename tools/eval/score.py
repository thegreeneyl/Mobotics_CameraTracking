#!/usr/bin/env python3
"""Score motion-path.v1 batch output against the hand-annotated events.

Usage:
  python3 tools/eval/score.py <eval-dir> [--events tools/eval/events.json]
                              [--match class|any] [--min-track-s 0.5]
                              [--json <out.json>]

<eval-dir> holds <rec>.motion.json files written by the batch runner
(CAMTRACK_BATCH=...). For every recording that has events, the script prints:

  per event   ids       confirmed track ids seen in the window (target 1)
              maxsim    max simultaneous confirmed tracks of that class (target 1)
              cover     fraction of window frames with >=1 confirmed track
              switches  id changes of the dominant track sequence
              dir       fraction of track frames moving in the annotated direction
  per rec     false     confirmed tracks of an annotated class outside every
                        event window (+- tolerance), lasting >= --min-track-s
              jitter    mean per-track std of bbox edge residuals after
                        subtracting the published velocity (x1000, corridor units)
              vstd      mean per-track std of frame-to-frame velocity change
                        relative to mean |v|
  plus a <rec>.tracks.json next to the motion file with the per-track summary.

Pure Python (json only) so it runs anywhere.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
from collections import defaultdict


def load_events(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def load_motion(path):
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    frames = doc["frames"] if isinstance(doc, dict) else doc
    return doc, frames


def track_table(frames):
    """id -> list of (t, obj) for confirmed objects."""
    tracks = defaultdict(list)
    for fr in frames:
        t = float(fr.get("t", 0.0))
        for o in fr.get("objects", []):
            tracks[(o.get("view", ""), int(o["id"]))].append((t, o))
    return tracks


def track_summary(key, samples):
    view, tid = key
    ts = [t for t, _ in samples]
    cls = samples[0][1].get("class")
    vxs = [o["vx"] for _, o in samples]
    vys = [o["vy"] for _, o in samples]
    speeds = [math.hypot(vx, vy) for vx, vy in zip(vxs, vys)]
    mean_speed = statistics.fmean(speeds) if speeds else 0.0
    # bbox edge residuals after subtracting the published velocity
    res_x0, res_x1, dv = [], [], []
    for (t0, a), (t1, b) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0 or dt > 0.5:
            continue
        ax0, ax1 = a["bbox"][0], a["bbox"][0] + a["bbox"][2]
        bx0, bx1 = b["bbox"][0], b["bbox"][0] + b["bbox"][2]
        clip = a.get("clipped", {})
        # an edge pinned at the lane end is not expected to move with v
        if not clip.get("left"):
            res_x0.append(bx0 - (ax0 + a["vx"] * dt))
        if not clip.get("right"):
            res_x1.append(bx1 - (ax1 + a["vx"] * dt))
        dv.append(math.hypot(b["vx"] - a["vx"], b["vy"] - a["vy"]))

    def pstd(v):
        return statistics.pstdev(v) if len(v) > 1 else 0.0

    jitter = (pstd(res_x0) + pstd(res_x1)) / 2.0
    vstd = pstd(dv) / mean_speed if mean_speed > 1e-6 else 0.0
    sign_flips = 0
    last = 0
    for vx in vxs:
        s = 1 if vx > 0.002 else -1 if vx < -0.002 else 0
        if s and last and s != last:
            sign_flips += 1
        if s:
            last = s
    return {
        "view": view,
        "id": tid,
        "class": cls,
        "t_first": round(ts[0], 3),
        "t_last": round(ts[-1], 3),
        "frames": len(samples),
        "mean_speed": round(mean_speed, 4),
        "mean_vx": round(statistics.fmean(vxs), 4),
        "mean_w": round(statistics.fmean(o["bbox"][2] for _, o in samples), 4),
        "mean_h": round(statistics.fmean(o["bbox"][3] for _, o in samples), 4),
        "jitter": round(jitter, 5),
        "vstd": round(vstd, 4),
        "vx_sign_flips": sign_flips,
    }


def frames_in(frames, t0, t1):
    return [fr for fr in frames if t0 <= float(fr.get("t", 0.0)) <= t1]


def score_recording(name, frames, rec_events, cfg, match, min_track_s):
    tol_default = float(cfg.get("tolerance_s", 2.0))
    tol_approx = float(cfg.get("tolerance_approx_s", 4.0))
    tracks = track_table(frames)
    summaries = [track_summary(k, v) for k, v in tracks.items()]
    by_key = {(s["view"], s["id"]): s for s in summaries}

    def class_ok(obj, cls):
        return match == "any" or obj.get("class") == cls

    # Each (view, id) belongs to the event window where it has the most
    # frames, so a track exiting just as the next event's train enters is
    # not counted twice when the tolerance windows overlap.
    owner = {}
    owner_n = {}
    for ei, ev in enumerate(rec_events):
        tol = tol_approx if ev.get("approx") else tol_default
        t0, t1 = float(ev["t0"]), float(ev["t1"])
        counts = {}
        for fr in frames_in(frames, t0 - tol, t1 + tol):
            for o in fr.get("objects", []):
                if not class_ok(o, ev["class"]):
                    continue
                k = (o.get("view", ""), int(o["id"]))
                counts[k] = counts.get(k, 0) + 1
        for k, n in counts.items():
            if n > owner_n.get(k, 0):
                owner[k] = ei
                owner_n[k] = n

    event_rows = []
    all_gate_ok = True
    for ei, ev in enumerate(rec_events):
        cls = ev["class"]
        tol = tol_approx if ev.get("approx") else tol_default
        t0, t1 = float(ev["t0"]), float(ev["t1"])
        window = frames_in(frames, t0 - tol, t1 + tol)
        core = frames_in(frames, t0 + tol, t1 - tol) or window
        ids = {}
        maxsim = 0
        covered = 0
        dir_ok = dir_n = 0
        dominant_seq = []
        for fr in core:
            objs = [o for o in fr.get("objects", []) if class_ok(o, cls)]
            if objs:
                covered += 1
            maxsim = max(maxsim, len(objs))
            if objs:
                best = max(objs, key=lambda o: o["bbox"][2] * o["bbox"][3])
                if not dominant_seq or dominant_seq[-1] != best["id"]:
                    dominant_seq.append(best["id"])
        for fr in window:
            for o in fr.get("objects", []):
                if not class_ok(o, cls):
                    continue
                k = (o.get("view", ""), int(o["id"]))
                if owner.get(k) != ei:
                    continue
                ids[k] = ids.get(k, 0) + 1
                if "direction" in ev:
                    vx = o["vx"]
                    if abs(vx) > 0.002:
                        dir_n += 1
                        if (vx < 0) == (ev["direction"] == "left"):
                            dir_ok += 1
        # ids that lasted at least min_track_s inside the window count
        fps = max(1.0, len(window) / max(1e-6, (t1 - t0 + 2 * tol)))
        significant = [k for k, n in ids.items() if n >= min_track_s * fps]
        coverage = covered / len(core) if core else 0.0
        switches = max(0, len(dominant_seq) - 1)
        row = {
            "class": cls,
            "t0": t0,
            "t1": t1,
            "approx": bool(ev.get("approx")),
            "coarse": bool(ev.get("coarse")),
            "ids": sorted(k[1] for k in significant),
            "n_ids": len(significant),
            "maxsim": maxsim,
            "coverage": round(coverage, 3),
            "switches": switches,
            "dir_frac": round(dir_ok / dir_n, 3) if dir_n else None,
        }
        gate = row["n_ids"] == 1 and maxsim <= 1 and coverage >= 0.95
        row["ok"] = gate
        if not ev.get("coarse") and not gate:
            all_gate_ok = False
        event_rows.append(row)

    # false tracks: confirmed tracks of an annotated class outside all windows
    classes = {ev["class"] for ev in rec_events}
    false_tracks = []
    for s in summaries:
        if match != "any" and s["class"] not in classes:
            continue
        if s["t_last"] - s["t_first"] < min_track_s:
            continue
        inside = False
        for ev in rec_events:
            tol = tol_approx if ev.get("approx") else tol_default
            if match != "any" and ev["class"] != s["class"]:
                continue
            if s["t_first"] <= ev["t1"] + tol and s["t_last"] >= ev["t0"] - tol:
                inside = True
                break
        if not inside:
            false_tracks.append(s)

    long_tracks = [s for s in summaries if s["frames"] >= 15]
    jitter = statistics.fmean(s["jitter"] for s in long_tracks) if long_tracks else 0.0
    vstd = statistics.fmean(s["vstd"] for s in long_tracks) if long_tracks else 0.0

    return {
        "recording": name,
        "frames": len(frames),
        "tracks_total": len(summaries),
        "events": event_rows,
        "false_tracks": [(s["view"], s["id"], s["class"], s["t_first"], s["t_last"]) for s in false_tracks],
        "n_false": len(false_tracks),
        "jitter_x1000": round(jitter * 1000, 2),
        "vstd": round(vstd, 3),
        "gate_ok": all_gate_ok and not false_tracks,
        "tracks": summaries,
    }


def print_report(results):
    for r in results:
        print(f"\n== {r['recording']}  frames={r['frames']}  tracks={r['tracks_total']}  "
              f"false={r['n_false']}  jitter={r['jitter_x1000']}e-3  vstd={r['vstd']}  "
              f"{'OK' if r['gate_ok'] else 'FAIL'}")
        print(f"  {'class':6} {'window':>13} {'ids':>4} {'maxsim':>6} {'cover':>6} {'sw':>3} {'dir':>5}  ids")
        for e in r["events"]:
            flag = " (coarse)" if e["coarse"] else " (approx)" if e["approx"] else ""
            ids = ",".join(str(i) for i in e["ids"][:12]) + ("..." if len(e["ids"]) > 12 else "")
            d = "-" if e["dir_frac"] is None else f"{e['dir_frac']:.2f}"
            mark = "ok " if e["ok"] else "BAD"
            print(f"  {e['class']:6} {e['t0']:6.1f}-{e['t1']:6.1f} {e['n_ids']:4d} {e['maxsim']:6d} "
                  f"{e['coverage']:6.2f} {e['switches']:3d} {d:>5}  {mark} [{ids}]{flag}")
        if r["false_tracks"]:
            shown = r["false_tracks"][:8]
            more = "" if len(r["false_tracks"]) <= 8 else f" ... (+{len(r['false_tracks']) - 8})"
            print("  false tracks: " + ", ".join(f"{v}#{i} {c} {a:.0f}-{b:.0f}s" for v, i, c, a, b in shown) + more)


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("eval_dir")
    ap.add_argument("--events", default=os.path.join(os.path.dirname(__file__), "events.json"))
    ap.add_argument("--match", choices=["class", "any"], default="class",
                    help="match tracks to events by class (default) or ignore class")
    ap.add_argument("--min-track-s", type=float, default=0.5)
    ap.add_argument("--json", default=None, help="write the full result as JSON")
    args = ap.parse_args(argv)

    cfg = load_events(args.events)
    results = []
    for fn in sorted(os.listdir(args.eval_dir)):
        if not fn.endswith(".motion.json"):
            continue
        name = fn[: -len(".motion.json")]
        rec_cfg = cfg["recordings"].get(name)
        if not rec_cfg:
            print(f"(skipping {name}: no events annotated)")
            continue
        _, frames = load_motion(os.path.join(args.eval_dir, fn))
        r = score_recording(name, frames, rec_cfg["events"], cfg, args.match, args.min_track_s)
        with open(os.path.join(args.eval_dir, name + ".tracks.json"), "w", encoding="utf-8") as f:
            json.dump(r["tracks"], f, indent=1)
        results.append(r)

    if not results:
        print("no motion files found", file=sys.stderr)
        return 1
    print_report(results)
    ok = all(r["gate_ok"] for r in results)
    print(f"\nTOTAL: {sum(len(r['events']) for r in results)} events in {len(results)} recordings -> "
          f"{'ALL OK' if ok else 'FAILING'}")
    if args.json:
        slim = [{k: v for k, v in r.items() if k != "tracks"} for r in results]
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(slim, f, indent=1)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
