#!/usr/bin/env python3
"""Score motion-path.v1 batch output against the hand-annotated events.

Usage:
  python3 tools/eval/score.py <eval-dir> [--events tools/eval/events.json]
                              [--match class|any] [--min-track-s 0.5]
                              [--json <out.json>]

<eval-dir> holds <rec>.motion.json files written by the batch runner
(CAMTRACK_BATCH=...). For every recording that has events, the script prints:

  per event   ids       confirmed track ids owned by the event (target 1)
              maxsim    max simultaneous confirmed tracks of that class in the
                        event's core window; `allow` is the number of annotated
                        events of that class concurrent at that moment (two
                        trains meeting are two events), target maxsim <= allow
              cover     fraction of core-window frames with >=1 owned track
              switches  id changes of the dominant track sequence
              dir       fraction of track frames moving in the annotated direction
  per rec     false     confirmed tracks of an annotated class outside every
                        event window (+- tolerance), lasting >= --min-track-s
              jitter    mean per-track std of bbox edge residuals after
                        subtracting the published velocity (x1000, corridor units)
              vstd      mean per-track std of frame-to-frame velocity change
                        relative to mean |v|
  smoothness  nonedge   confirmed tracks (>= 1 s) of an annotated class that were
                        NOT born clipped at / within 0.20 of a corridor end;
                        judged on the tracker's birth frame (`eb` flag) when
                        <rec>.debug.jsonl is present, else on the first
                        published frame
              middeath  confirmed tracks (>= 1 s) that ended in mid corridor
                        (unclipped, 0.10 < x0, x1 < 0.90, not stopped/gone, and
                        not at the end of the recording)
              boxrate   95th percentile over tracks of the max corridor-width
                        change per second (how fast boxes resize)
              omiss     fraction of track frames without an outline
              ojit      mean per-vertex outline displacement between consecutive
                        frames in box-normalised units (x1000), after resampling
                        every outline to 32 vertices

Gate per recording: every non-coarse event ok (1 id, maxsim <= allow,
coverage >= 0.95), no false tracks, nonedge == 0 and middeath == 0.

A track belongs to the event window where it has the most frames among the
events whose class and direction it matches, so a track exiting just as the
next event's train enters is not counted twice and two trains meeting are
kept apart by direction.

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

EDGE_FRAC = 0.20          # born within this of a corridor end counts as an edge birth (train edgeBirthFrac)
MID_LO, MID_HI = 0.10, 0.90
RESAMPLE_N = 32


def load_events(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def load_motion(path):
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    frames = doc["frames"] if isinstance(doc, dict) else doc
    return doc, frames


def track_table(frames):
    """(view, id) -> list of (t, obj) for confirmed objects."""
    tracks = defaultdict(list)
    for fr in frames:
        t = float(fr.get("t", 0.0))
        for o in fr.get("objects", []):
            tracks[(o.get("view", ""), int(o["id"]))].append((t, o))
    return tracks


def corridor_edges(o):
    c = o.get("corridor")
    if not c:
        b = o["bbox"]
        return b[0], b[0] + b[2]
    return c["x"] - c["w"] / 2.0, c["x"] + c["w"] / 2.0


def resample(poly, n):
    """Closed polygon -> n points equally spaced by arc length, starting at the
    vertex nearest the polygon's leftmost point (stable anchor)."""
    if len(poly) < 3:
        return None
    start = min(range(len(poly)), key=lambda i: (poly[i][0], poly[i][1]))
    pts = poly[start:] + poly[:start]
    # orientation: make it clockwise (positive signed area in y-down coords)
    area = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        area += x0 * y1 - x1 * y0
    if area < 0:
        pts = [pts[0]] + pts[:0:-1]
    seg = []
    total = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        d = math.hypot(x1 - x0, y1 - y0)
        seg.append(d)
        total += d
    if total <= 1e-9:
        return None
    out = []
    step = total / n
    si = 0
    acc = 0.0
    for k in range(n):
        target = k * step
        while si < len(seg) - 1 and acc + seg[si] < target:
            acc += seg[si]
            si += 1
        x0, y0 = pts[si]
        x1, y1 = pts[(si + 1) % len(pts)]
        f = 0.0 if seg[si] <= 1e-9 else (target - acc) / seg[si]
        out.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f))
    return out


def box_normalised(poly, o):
    b = o["bbox"]
    w = max(b[2], 1e-6)
    h = max(b[3], 1e-6)
    return [((x - b[0]) / w, (y - b[1]) / h) for x, y in poly]


def load_births(debug_path):
    """(view, id) -> tracker-side birth record from <rec>.debug.jsonl, if present.

    The published stream starts at confirmation, a few frames after the
    tracker created the track (and after the silhouette box may already have
    shrunk it), so the edge rule is judged on the tracker's own birth frame
    when the debug dump is available: `eb` is the tracker's edgeBorn flag."""
    births = {}
    if not os.path.exists(debug_path):
        return births
    with open(debug_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            fr = json.loads(line)
            for tr in fr.get("tracks", []):
                k = (tr.get("c", ""), int(tr["id"]))
                if k not in births:
                    births[k] = {"t": fr.get("t", 0.0), "x": [tr["x0"], tr["x1"]],
                                 "eb": tr.get("eb"), "cl": tr.get("cl"), "cr": tr.get("cr")}
    return births


def track_summary(key, samples, rec_end_t, birth=None):
    view, tid = key
    ts = [t for t, _ in samples]
    cls = samples[0][1].get("class")
    vxs = [o["vx"] for _, o in samples]
    vys = [o["vy"] for _, o in samples]
    speeds = [math.hypot(vx, vy) for vx, vy in zip(vxs, vys)]
    mean_speed = statistics.fmean(speeds) if speeds else 0.0
    # bbox edge residuals after subtracting the published velocity
    res_x0, res_x1, dv = [], [], []
    box_rates = []
    outline_disp = []
    prev_norm = None
    outline_missing = 0
    for (t0, a), (t1, b) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0 or dt > 0.5:
            prev_norm = None
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
        ca0, ca1 = corridor_edges(a)
        cb0, cb1 = corridor_edges(b)
        box_rates.append(abs((cb1 - cb0) - (ca1 - ca0)) / dt)
    for _, o in samples:
        ol = o.get("outline") or []
        if len(ol) < 3:
            outline_missing += 1
            prev_norm = None
            continue
        rs = resample([tuple(p) for p in ol], RESAMPLE_N)
        if rs is None:
            prev_norm = None
            continue
        norm = box_normalised(rs, o)
        if prev_norm is not None:
            outline_disp.append(statistics.fmean(
                math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(prev_norm, norm)))
        prev_norm = norm

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

    first, lastobj = samples[0][1], samples[-1][1]
    bx0, bx1 = corridor_edges(first)
    dx0, dx1 = corridor_edges(lastobj)
    fclip = first.get("clipped", {})
    lclip = lastobj.get("clipped", {})
    edge_birth = bool(fclip.get("left") or fclip.get("right")
                      or bx0 <= EDGE_FRAC or bx1 >= 1.0 - EDGE_FRAC)
    if birth and birth.get("eb") is not None:
        # the tracker's own verdict on its birth frame (see load_births)
        edge_birth = bool(birth["eb"])
        bx0, bx1 = birth["x"]
    at_end = ts[-1] >= rec_end_t - 0.5
    mid_death = (not lclip.get("left") and not lclip.get("right")
                 and dx0 > MID_LO and dx1 < MID_HI
                 and lastobj.get("phase") not in ("stopped", "gone")
                 and not at_end)
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
        "birth_x": [round(bx0, 3), round(bx1, 3)],
        "death_x": [round(dx0, 3), round(dx1, 3)],
        "birth_phase": first.get("phase"),
        "death_phase": lastobj.get("phase"),
        "edge_birth": edge_birth,
        "mid_death": bool(mid_death),
        "box_rate_max": round(max(box_rates), 4) if box_rates else 0.0,
        "outline_missing_frac": round(outline_missing / len(samples), 3),
        "outline_jitter": round(statistics.fmean(outline_disp), 5) if outline_disp else 0.0,
    }


def frames_in(frames, t0, t1):
    return [fr for fr in frames if t0 <= float(fr.get("t", 0.0)) <= t1]


def score_recording(name, frames, rec_events, cfg, match, min_track_s, births=None):
    tol_default = float(cfg.get("tolerance_s", 2.0))
    tol_approx = float(cfg.get("tolerance_approx_s", 4.0))
    rec_end_t = float(frames[-1].get("t", 0.0)) if frames else 0.0
    tracks = track_table(frames)
    births = births or {}
    summaries = [track_summary(k, v, rec_end_t, births.get(k)) for k, v in tracks.items()]
    by_key = {(s["view"], s["id"]): s for s in summaries}

    def class_ok(obj, cls):
        return match == "any" or obj.get("class") == cls

    def tol_of(ev):
        return tol_approx if ev.get("approx") else tol_default

    def direction_ok(summary, ev):
        if "direction" not in ev:
            return True
        vx = summary["mean_vx"]
        if abs(vx) < 0.002:
            return True
        return (vx < 0) == (ev["direction"] == "left")

    # Each (view, id) belongs to the event window where it has the most
    # frames among the events whose class and direction it matches.
    owner = {}
    owner_n = {}
    for ei, ev in enumerate(rec_events):
        tol = tol_of(ev)
        t0, t1 = float(ev["t0"]), float(ev["t1"])
        counts = {}
        for fr in frames_in(frames, t0 - tol, t1 + tol):
            for o in fr.get("objects", []):
                if not class_ok(o, ev["class"]):
                    continue
                k = (o.get("view", ""), int(o["id"]))
                if k in by_key and not direction_ok(by_key[k], ev):
                    continue
                counts[k] = counts.get(k, 0) + 1
        for k, n in counts.items():
            if n > owner_n.get(k, 0):
                owner[k] = ei
                owner_n[k] = n

    def concurrent(cls, t):
        return sum(1 for ev in rec_events
                   if ev["class"] == cls and ev["t0"] - tol_of(ev) <= t <= ev["t1"] + tol_of(ev))

    event_rows = []
    all_gate_ok = True
    for ei, ev in enumerate(rec_events):
        cls = ev["class"]
        tol = tol_of(ev)
        t0, t1 = float(ev["t0"]), float(ev["t1"])
        window = frames_in(frames, t0 - tol, t1 + tol)
        core = frames_in(frames, t0 + tol, t1 - tol) or window
        ids = {}
        maxsim = 0
        allow_at_max = 1
        excess = 0
        covered = 0
        dir_ok = dir_n = 0
        dominant_seq = []
        for fr in core:
            t = float(fr.get("t", 0.0))
            objs = [o for o in fr.get("objects", []) if class_ok(o, cls)]
            owned = [o for o in objs if owner.get((o.get("view", ""), int(o["id"]))) == ei]
            if owned:
                covered += 1
            allow = max(1, concurrent(cls, t))
            if len(objs) > maxsim:
                maxsim = len(objs)
                allow_at_max = allow
            excess = max(excess, len(objs) - allow)
            if owned:
                best = max(owned, key=lambda o: o["bbox"][2] * o["bbox"][3])
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
            "allow": allow_at_max,
            "excess": excess,
            "coverage": round(coverage, 3),
            "switches": switches,
            "dir_frac": round(dir_ok / dir_n, 3) if dir_n else None,
        }
        gate = row["n_ids"] == 1 and excess == 0 and coverage >= 0.95
        row["ok"] = gate
        if not ev.get("coarse") and not gate:
            all_gate_ok = False
        event_rows.append(row)

    # false tracks: confirmed tracks outside all windows of their class. The
    # events list every train and boat, so a class without events in this
    # recording must not produce a confirmed track at all.
    classes = {ev["class"] for ev in rec_events}
    false_tracks = []
    for s in summaries:
        if s["t_last"] - s["t_first"] < min_track_s:
            continue
        inside = False
        for ev in rec_events:
            tol = tol_of(ev)
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

    # smoothness / lifecycle over confirmed tracks lasting >= 1 s
    life = [s for s in summaries if s["t_last"] - s["t_first"] >= 1.0]
    nonedge = [s for s in life if not s["edge_birth"]]
    middeath = [s for s in life if s["mid_death"]]
    rates = sorted(s["box_rate_max"] for s in life)
    box_rate_p95 = rates[min(len(rates) - 1, int(0.95 * len(rates)))] if rates else 0.0
    total_frames = sum(s["frames"] for s in life)
    omiss = (sum(s["outline_missing_frac"] * s["frames"] for s in life) / total_frames) if total_frames else 0.0
    ojit = statistics.fmean(s["outline_jitter"] for s in life if s["outline_jitter"] > 0) \
        if any(s["outline_jitter"] > 0 for s in life) else 0.0

    return {
        "recording": name,
        "frames": len(frames),
        "tracks_total": len(summaries),
        "events": event_rows,
        "false_tracks": [(s["view"], s["id"], s["class"], s["t_first"], s["t_last"]) for s in false_tracks],
        "n_false": len(false_tracks),
        "jitter_x1000": round(jitter * 1000, 2),
        "vstd": round(vstd, 3),
        "nonedge": [(s["view"], s["id"], s["class"], s["t_first"], s["birth_x"]) for s in nonedge],
        "middeath": [(s["view"], s["id"], s["class"], s["t_last"], s["death_x"]) for s in middeath],
        "box_rate_p95": round(box_rate_p95, 3),
        "outline_missing": round(omiss, 3),
        "outline_jitter_x1000": round(ojit * 1000, 2),
        "gate_ok": all_gate_ok and not false_tracks and not nonedge and not middeath,
        "tracks": summaries,
    }


def print_report(results):
    for r in results:
        print(f"\n== {r['recording']}  frames={r['frames']}  tracks={r['tracks_total']}  "
              f"false={r['n_false']}  jitter={r['jitter_x1000']}e-3  vstd={r['vstd']}  "
              f"{'OK' if r['gate_ok'] else 'FAIL'}")
        print(f"   nonedge={len(r['nonedge'])}  middeath={len(r['middeath'])}  "
              f"boxrate95={r['box_rate_p95']}/s  omiss={r['outline_missing']}  "
              f"ojit={r['outline_jitter_x1000']}e-3")
        print(f"  {'class':6} {'window':>13} {'ids':>4} {'maxsim':>6} {'allow':>5} {'cover':>6} {'sw':>3} {'dir':>5}  ids")
        for e in r["events"]:
            flag = " (coarse)" if e["coarse"] else " (approx)" if e["approx"] else ""
            ids = ",".join(str(i) for i in e["ids"][:12]) + ("..." if len(e["ids"]) > 12 else "")
            d = "-" if e["dir_frac"] is None else f"{e['dir_frac']:.2f}"
            mark = "ok " if e["ok"] else "BAD"
            print(f"  {e['class']:6} {e['t0']:6.1f}-{e['t1']:6.1f} {e['n_ids']:4d} {e['maxsim']:6d} {e['allow']:5d} "
                  f"{e['coverage']:6.2f} {e['switches']:3d} {d:>5}  {mark} [{ids}]{flag}")
        if r["false_tracks"]:
            shown = r["false_tracks"][:8]
            more = "" if len(r["false_tracks"]) <= 8 else f" ... (+{len(r['false_tracks']) - 8})"
            print("  false tracks: " + ", ".join(f"{v}#{i} {c} {a:.0f}-{b:.0f}s" for v, i, c, a, b in shown) + more)
        if r["nonedge"]:
            print("  non-edge births: " + ", ".join(f"{v}#{i} {c} t={t:.0f}s x={x}" for v, i, c, t, x in r["nonedge"][:8]))
        if r["middeath"]:
            print("  mid-corridor deaths: " + ", ".join(f"{v}#{i} {c} t={t:.0f}s x={x}" for v, i, c, t, x in r["middeath"][:8]))


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
        births = load_births(os.path.join(args.eval_dir, name + ".debug.jsonl"))
        r = score_recording(name, frames, rec_cfg["events"], cfg, args.match, args.min_track_s, births)
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
