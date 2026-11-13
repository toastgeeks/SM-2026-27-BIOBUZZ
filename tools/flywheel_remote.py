#!/usr/bin/env python3
"""Drive the Remote Flywheel Tuner OpMode over adb and summarize each run.

Start "Remote Flywheel Tuner" on the Driver Station first (INIT, then start). Then:

  flywheel_remote.py power --power 1.0 --duration 4
  flywheel_remote.py velocity --f 12 --to 1125 --duration 3
  flywheel_remote.py velocity --p 10 --f 12 --from 900 --settle 3 --to 1125 --duration 3
  flywheel_remote.py velocity --p 444 --i 0.6855 --f 22.2 --from 1125 --settle 3 --to 1125 --duration 20 --feed
  flywheel_remote.py status

--feed runs the feed and intake during the step (not the settle) and reports every shot.

Each run's CSV is saved in --out (default ./flywheel-runs) and summarized on stdout.
"""
import argparse
import csv
import os
import statistics
import subprocess
import sys
import tempfile
import time

DEFAULT_ADB = os.path.expanduser("~/Library/Android/sdk/platform-tools/adb")
REMOTE_DIR = "/sdcard/FIRST/claude-tune"


def adb(*args, check=True):
    exe = os.environ.get("ADB", DEFAULT_ADB)
    result = subprocess.run([exe, *args], capture_output=True, text=True)
    if check and result.returncode != 0:
        sys.exit(f"adb {' '.join(args)} failed: {result.stderr.strip() or result.stdout.strip()}")
    return result.stdout


def read_status():
    text = adb("shell", f"cat {REMOTE_DIR}/status.properties 2>/dev/null", check=False)
    status = {}
    for line in text.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            status[key.strip()] = value.strip()
    return status


def send(fields, timeout):
    seq = int(time.time() * 1000)
    body = f"seq={seq}\n" + "".join(f"{k}={v}\n" for k, v in fields.items())
    with tempfile.NamedTemporaryFile("w", suffix=".properties", delete=False) as tmp:
        tmp.write(body)
        local = tmp.name
    try:
        adb("push", local, f"{REMOTE_DIR}/cmd.tmp")
    finally:
        os.unlink(local)
    adb("shell", f"mv {REMOTE_DIR}/cmd.tmp {REMOTE_DIR}/cmd.properties")

    deadline = time.time() + timeout
    while time.time() < deadline:
        status = read_status()
        if status.get("seq") == str(seq) and status.get("state") in ("done", "aborted", "error"):
            return seq, status
        if status.get("state") == "stopped":
            sys.exit("The OpMode is stopped. Start Remote Flywheel Tuner on the Driver Station.")
        time.sleep(0.3)
    sys.exit(f"No result for command {seq} after {timeout:.0f} s. Is the OpMode running (not just INIT)? "
             f"Last status: {read_status()}")


def load(path):
    with open(path) as f:
        return [
            {k: (v if k == "phase" else float(v)) for k, v in row.items()}
            for row in csv.DictReader(f)
        ]


def window(rows, start, end):
    return [r for r in rows if start <= r["t"] <= end]


def summarize_velocity(rows, args):
    step = [r for r in rows if r["phase"] == "step"]
    if len(step) < 5:
        print("Too few samples to summarize.")
        return
    target = args.to
    v0 = step[0]["v1"]
    span = target - v0
    up = span >= 0
    dts = [b["t"] - a["t"] for a, b in zip(step, step[1:])]
    print(f"Samples: {len(step)}, loop {statistics.median(dts) * 1000:.1f} ms median")

    if abs(span) > 1:
        rise = next((r["t"] for r in step if (r["v1"] - v0) / span >= 0.9), None)
        peak = max(r["v1"] for r in step) if up else min(r["v1"] for r in step)
        overshoot = max(0.0, (peak - target) / span * 100 if up else (target - peak) / -span * 100)
        print(f"Step {v0:.0f} -> {target:.0f} ticks/s")
        print(f"  90% rise:  {f'{rise:.3f} s' if rise is not None else 'never'}")
        print(f"  overshoot: {overshoot:.1f}% of the step (peak {peak:.0f})")

    band = 0.02 * target
    outside = [r["t"] for r in step if abs(r["v1"] - target) > band]
    last = step[-1]["t"]
    if not outside:
        print("  settled ±2%: from the start")
    elif outside[-1] < last - 0.2:
        print(f"  settled ±2%: {outside[-1]:.3f} s")
    else:
        print("  settled ±2%: NOT settled by the end")

    tail = window(step, last - args.tail, last)
    v1 = [r["v1"] for r in tail]
    v2 = [r["v2"] for r in tail]
    mean1 = statistics.mean(v1)
    print(f"Last {args.tail:.1f} s:")
    print(f"  flywheel1 mean {mean1:.0f} (error {mean1 - target:+.0f}, {(mean1 - target) / max(target, 1) * 100:+.2f}%),"
          f" ripple ±{statistics.pstdev(v1):.1f}, range {min(v1):.0f}..{max(v1):.0f}")
    mean2 = statistics.mean(v2)
    print(f"  flywheel2 mean {mean2:.0f} (error {mean2 - target:+.0f}), ripple ±{statistics.pstdev(v2):.1f}")
    if target > 100 and abs(mean2) < 0.1 * target:
        print("  WARNING: flywheel2 reads about 0. Its encoder may be missing, so its velocity loop runs blind.")
    print_power(rows)


def position_velocity(rows, window=0.05):
    """Velocity from encoder position over about `window` seconds: finer than the hub's 20 ticks/s steps."""
    out = []
    j = 0
    for i, r in enumerate(rows):
        while j < i and r["t"] - rows[j]["t"] > window:
            j += 1
        k = max(j - 1, 0)
        dt = r["t"] - rows[k]["t"]
        out.append((r["pos1"] - rows[k]["pos1"]) / dt if dt > 0 else r["v1"])
    return out


def summarize_shots(rows, args):
    """Shots are found from feed current: a ball pushed through the feed raises it well above idle."""
    step = [r for r in rows if r["phase"] == "step" and r["t"] > 0.2]  # skip the feed switching on
    if not step or "feedAmps" not in step[0]:
        sys.exit("This CSV has no feed current. Redeploy the updated OpMode.")
    target = args.to
    vel = position_velocity(step, window=0.1)
    idle = statistics.median(r["feedAmps"] for r in step)
    threshold = idle + max(1.0, idle)
    band = 0.02 * target

    shots = []
    i = 0
    while i < len(step):
        if step[i]["feedAmps"] > threshold:
            t0 = step[i]["t"]
            # look from just before the feed spike to 0.8 s after it
            seg = [k for k, r in enumerate(step) if t0 - 0.1 <= r["t"] <= t0 + 0.8]
            low = min(seg, key=lambda k: vel[k])
            hub_low = min(step[k]["v1"] for k in seg)
            rec = next((step[k]["t"] - t0 for k in seg if k > low and vel[k] >= target - band), None)
            shots.append((t0, vel[low], hub_low, rec, max(step[k]["feedAmps"] for k in seg)))
            while i < len(step) and step[i]["t"] < t0 + 0.8:
                i += 1
        else:
            i += 1

    hold = [v for v in vel]
    print(f"Holding {target:.0f} ticks/s with feed and intake on for {step[-1]['t']:.1f} s")
    print(f"  overall: mean {statistics.mean(hold):.0f}, hub reading {statistics.mean(r['v1'] for r in step):.0f}")
    print(f"  feed idle current {idle:.1f} A; a shot is a spike above {threshold:.1f} A")
    if not shots:
        print("  No shots found (no feed current spike).")
    for n, (t0, lo, hub_lo, rec, amps) in enumerate(shots, 1):
        rec_text = f"{rec:.2f} s" if rec is not None else "not within 0.8 s"
        print(f"  Shot {n} at {t0:5.2f} s: low {lo:.0f} ({(lo - target) / target * 100:+.1f}%), hub low {hub_lo:.0f}, "
              f"back within 2% after {rec_text}, feed peak {amps:.1f} A")
    print_power(rows)


def summarize_power(rows, args):
    last = rows[-1]["t"]
    tail = window(rows, last - args.tail, last)
    mean1 = statistics.mean(r["v1"] for r in tail)
    mean2 = statistics.mean(r["v2"] for r in tail)
    print(f"Open loop at power {args.power}: last {args.tail:.1f} s")
    print(f"  flywheel1 {mean1:.0f} ticks/s, flywheel2 {mean2:.0f} ticks/s")
    if args.power > 0 and mean1 > 0:
        top = mean1 / args.power
        print(f"  top speed about {top:.0f} ticks/s, REV starting F = 32767 / top = {32767 / top:.2f}")
    print_power(rows)


def print_power(rows):
    print(f"Peak current: flywheel1 {max(r['amps1'] for r in rows):.1f} A, "
          f"flywheel2 {max(r['amps2'] for r in rows):.1f} A; "
          f"battery {min(r['volts'] for r in rows):.2f}..{max(r['volts'] for r in rows):.2f} V")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="flywheel-runs", help="where run CSVs are saved")
    parser.add_argument("--tail", type=float, default=1.0, help="seconds at the end used for steady-state stats")
    sub = parser.add_subparsers(dest="cmd", required=True)

    vel = sub.add_parser("velocity", help="PIDF velocity step")
    for gain in ("p", "i", "d", "f"):
        vel.add_argument(f"--{gain}", type=float, default=0.0)
    vel.add_argument("--from", dest="from_", type=float, default=0.0, help="ticks/s held during settle")
    vel.add_argument("--settle", type=float, default=0.0, help="seconds at --from before the step")
    vel.add_argument("--to", type=float, required=True, help="ticks/s after the step")
    vel.add_argument("--duration", type=float, default=3.0, help="seconds after the step")
    vel.add_argument("--feed", action="store_true", help="run feed and intake during the step; report shots")

    pwr = sub.add_parser("power", help="open-loop power (top speed)")
    pwr.add_argument("--power", type=float, default=1.0)
    pwr.add_argument("--duration", type=float, default=4.0)

    sub.add_parser("status", help="show the OpMode status")
    args = parser.parse_args()

    if args.cmd == "status":
        print(read_status() or "No status file. Start Remote Flywheel Tuner.")
        return

    if args.cmd == "velocity":
        fields = {"action": "velocity", "p": args.p, "i": args.i, "d": args.d, "f": args.f,
                  "from": args.from_, "settle": args.settle, "to": args.to, "duration": args.duration,
                  "feed": 1 if args.feed else 0}
        wait = args.settle + args.duration + 15
    else:
        fields = {"action": "power", "power": args.power, "duration": args.duration}
        wait = args.duration + 15

    seq, status = send(fields, wait)
    if status["state"] == "error":
        sys.exit(f"Robot rejected the command: {status.get('detail')}")
    os.makedirs(args.out, exist_ok=True)
    local = os.path.join(args.out, f"run-{seq}.csv")
    adb("pull", f"{REMOTE_DIR}/run-{seq}.csv", local)
    if status["state"] == "aborted":
        print("Run ABORTED on the robot (gamepad X or OpMode stop). Partial data follows.")
    print(f"Saved {local}")
    rows = load(local)
    if not rows:
        print("No samples.")
        return
    if args.cmd == "velocity" and args.feed:
        summarize_shots(rows, args)
    else:
        (summarize_velocity if args.cmd == "velocity" else summarize_power)(rows, args)


if __name__ == "__main__":
    main()
