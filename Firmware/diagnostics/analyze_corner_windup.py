"""Wheel wind-up mechanism analysis for corner (single-point) runs.

For every recorded corner attempt, pull the $FAST samples from the balancing
window and fit the wheel speed derivative against the wheel speed itself:

    dV/dt = a*V + b

Two candidate mechanisms:
  * constant gravity torque from a reference offset -> a ~= 0, b != 0
    (constant slope, direction fixed by the offset, identical signs on all
    wheels in proportions set by the geometry);
  * speed self-feedback from a wrong-signed wheel gain -> a > 0, |a| growing
    with |K_w| (each wheel feeds its own speed back into its own voltage).

With --settle it reconstructs the observed gravity-direction mean. This is
an experimental candidate, not an identified neutral COM pose: accelerating
wheels apply reaction torque even while the body attitude appears steady.
Closed-loop derivative regressions are descriptive correlations, not a
causal identification of gravity offset versus wheel feedback. Human/cable
intervention must also be excluded using the accompanying field record.

Usage: python diagnostics/analyze_corner_windup.py [glob] [--settle]
"""
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DAY = os.path.join(HERE, "2026-10-15")


def parse_fast(data):
    if not data.startswith("$FAST,"):
        return None
    body = data[len("$FAST,"):].rstrip("#\r\n")
    out = {}
    for item in body.split(","):
        if ":" not in item:
            continue
        k, v = item.split(":", 1)
        try:
            out[k] = float(v)
        except ValueError:
            pass
    return out


def gravity_from_pit_rol(pit_deg, rol_deg):
    """Inverse of the firmware mapping (lqr_balance.c attitude update):
    pitch = atan2(gy, sqrt(gx^2+gz^2)), roll = atan2(-gx, gz)."""
    p = math.radians(pit_deg)
    r = math.radians(rol_deg)
    return (
        -math.sin(r) * math.cos(p),
        math.sin(p),
        math.cos(r) * math.cos(p),
    )


def angle_between(a, b):
    d = sum(x * y for x, y in zip(a, b))
    d = max(-1.0, min(1.0, d / (math.sqrt(sum(x * x for x in a)) * math.sqrt(sum(x * x for x in b)))))
    return math.degrees(math.acos(d))


def normalized(v):
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v)


def load_attempts(path):
    base = path[:-6] if path.endswith(".jsonl") else path
    summary = None
    for cand in (base + "_summary.json", base + "-summary.json"):
        if os.path.exists(cand):
            with open(cand, "r", encoding="utf-8") as fh:
                summary = json.load(fh)
            break
    fast = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get("kind") != "rx":
                continue
            sample = parse_fast(rec.get("data", ""))
            if sample is not None:
                fast.append((rec["t"], sample))
    return summary, fast


def linfit(xs, ys):
    n = len(xs)
    if n < 3:
        return 0.0, 0.0, 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx == 0:
        return 0.0, my, 0.0
    slope = sxy / sxx
    inter = my - slope * mx
    syy = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - sum((y - (slope * x + inter)) ** 2 for x, y in zip(xs, ys)) / syy if syy > 0 else 1.0
    return slope, inter, r2


def analyze(path):
    summary, fast = load_attempts(path)
    if summary is None:
        return None
    settings = summary.get("settings", {})
    kw = settings.get("kw", [0, 0, 0])
    ki = settings.get("ki", 0.0)
    kti = settings.get("tilt_trim", 0.0)
    rows = []
    ref_used = None
    if summary.get("reference") and summary["reference"].get("gravity"):
        ref_used = summary["reference"]["gravity"]
    elif settings.get("reference_gravity"):
        ref_used = settings["reference_gravity"]
    for att in summary.get("attempts", []):
        t0, t1 = att["started_t"], att["ended_t"]
        win3 = [(t, s) for t, s in fast if t0 <= t <= t1 and s.get("Bal") == 3]
        if len(win3) < 10:
            continue
        pick = win3[0][0] + 1.8  # let the wheel-feedback path ramp in fully
        win = [(t, s) for t, s in win3 if t >= pick]
        r = {
            "file": os.path.basename(path),
            "kw": kw,
            "ki": ki,
            "kti": kti,
            "duration_s": att.get("duration_s", t1 - t0),
            "end_reason": att.get("end_reason"),
            "trim_last": att.get("trim_last"),
            "axes": [],
        }
        for axis, key in enumerate(("V1", "V2", "V3")):
            ts = [t for t, s in win if key in s]
            vs = [s[key] for t, s in win if key in s]
            if len(ts) < 10:
                continue
            # Overall linear ramp (constant-torque hypothesis).
            slope, _, r2_lin = linfit(ts, vs)
            # Directional feedback: dV/dt vs V.
            dts, dvs = [], []
            for (ta, sa), (tb, sb) in zip(win, win[1:]):
                if key not in sa or key not in sb or tb <= ta:
                    continue
                dt = tb - ta
                if dt > 0.2:
                    continue
                dts.append(0.5 * (sa[key] + sb[key]))
                dvs.append((sb[key] - sa[key]) / dt)
            a, b, r2_fb = linfit(dts, dvs)
            r["axes"].append(
                {
                    "axis": key,
                    "v_start": vs[0],
                    "v_end": vs[-1],
                    "slope_lin": slope,
                    "r2_lin": r2_lin,
                    "a_feedback": a,
                    "b_const": b,
                    "r2_fb": r2_fb,
                }
            )
        # Stable-pose estimate from the quasi-static samples of this attempt.
        st = [s for _, s in win3 if abs(s.get("Rate", 9.0)) <= 0.15 and "Pit" in s and "Rol" in s]
        if len(st) >= 20:
            gs = [gravity_from_pit_rol(s["Pit"], s["Rol"]) for s in st]
            gmean = normalized(tuple(sum(g[i] for g in gs) / len(gs) for i in range(3)))
            r["settle"] = {
                "n": len(st),
                "gravity": gmean,
                "angle_from_ref_deg": angle_between(gmean, ref_used) if ref_used else None,
                "mean_tilt_deg": sum(s.get("Tilt", 0.0) for s in st) / len(st),
            }
        rows.append(r)
    return rows


def main():
    settle = "--settle" in sys.argv
    argv = [a for a in sys.argv[1:] if a != "--settle"]
    pattern = argv[0] if argv else os.path.join(DAY, "corner_*online*.jsonl")
    files = sorted(glob.glob(pattern))
    print(f"{len(files)} files")
    best = None
    clean = []
    for path in files:
        rows = analyze(path)
        if not rows:
            continue
        for r in rows:
            dur = r["duration_s"] or 0.0
            print(
                f"\n== {r['file']}  Kw={r['kw']} Ki={r['ki']} Kti={r['kti']}  "
                f"dur={dur:.1f}s reason={r['end_reason']} trim={r['trim_last']}"
            )
            for ax in r["axes"]:
                print(
                    f"   {ax['axis']}: {ax['v_start']:8.1f} -> {ax['v_end']:8.1f}  "
                    f"slope={ax['slope_lin']:6.2f} rad/s^2 (R2={ax['r2_lin']:.3f})  "
                    f"dV/dt={ax['a_feedback']:+6.3f}*V{ax['b_const']:+7.1f} (R2={ax['r2_fb']:.3f})"
                )
            if settle and "settle" in r:
                s = r["settle"]
                ang = s["angle_from_ref_deg"]
                line = f"   settle: n={s['n']} meanTilt={s['mean_tilt_deg']:.3f}deg"
                if ang is not None:
                    line += f"  angle(ref->settled)={ang:.3f}deg"
                print(line)
                if best is None or dur > best[1]:
                    best = (r["file"], dur, s["gravity"], ang)
                quiet = (
                    s["n"] >= 300
                    and s["mean_tilt_deg"] <= 0.7
                    and all(abs(k + 0.03) < 0.005 for k in (r["kw"] if isinstance(r["kw"], (list, tuple)) else [r["kw"]]))
                    and abs(r["ki"]) < 1e-6
                )
                if quiet:
                    clean.append((r["file"], s["gravity"], ang))
    if settle and best:
        print('\nEXPERIMENTAL: attitude means do not identify the zero-torque COM pose; no convergence guarantee.')
        fname, dur, g, ang = best
        print(f"\nlongest-run settle candidate from {fname} ({dur:.1f}s):")
        print("--reference-gravity " + " ".join(f"{v:.9f}" for v in g))
        if ang is not None:
            print(f"(mean settled pose sits {ang:.3f} deg from the reference that run used)")
    if settle and clean:
        # A repeatable observed attitude is only a candidate for controlled tests.
        # Telemetry cannot certify that the run was hands-off or free of contact.
        mean = normalized(tuple(sum(g[i] for _, g, _ in clean) / len(clean) for i in range(3)))
        print(f"\nexperimental candidate = attitude mean of {len(clean)} filtered run(s), contact status unverified:")
        for fname, g, ang in clean:
            print(f"  {fname}: angle to this candidate = {angle_between(g, mean):.3f}deg" +
                  (f" (was {ang:.3f}deg from old ref)" if ang is not None else ""))
        print("--reference-gravity " + " ".join(f"{v:.9f}" for v in mean))


if __name__ == "__main__":
    main()
