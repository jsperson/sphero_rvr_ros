"""Continuous ToF capture: one CSV row per frame, wall-clock stamped.

Columns: epoch, iso, seq, z00..z63 (millimetres, row-major 8x8 as the sensor
returns them). Nothing is filtered or rescaled here -- a capture that "cleans" its
own data cannot be re-analysed when the cleaning turns out to be wrong.

    python3 diagnostics/tof_capture.py out.csv      # run (foreground or setsid)
    python3 diagnostics/tof_capture.py --stop out.csv

WHY THERE IS A --stop. The 2026-08-13 tilt session tried to end a capture with
`kill <pid>` THREE times and hit the bash wrapper instead of the python child every
time; the sensor kept reading and the operator believed it had stopped. So the
process writes its OWN pid -- `os.getpid()`, which cannot be a wrapper -- beside the
CSV, and --stop signals that pid and waits for the file to actually go away. It also
closes the CSV on the way out, because the same session had a capture die silently
while Scott held a pose in front of a dead recorder.
"""
import datetime
import os
import signal
import sys
import time

DRIVER_PATH = os.environ.get("TOF_DRIVER_PATH", "/home/jsperson/tof_smoke")


def pidfile_for(csv_path):
    return csv_path + ".pid"


def stop(csv_path):
    """Signal the capture that owns this CSV, and report what actually happened."""
    pf = pidfile_for(csv_path)
    try:
        pid = int(open(pf).read().strip())
    except (OSError, ValueError):
        print(f"no pidfile at {pf} -- nothing to stop (check for a stray process yourself)")
        return 1
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        os.unlink(pf)
        print(f"pid {pid} was already gone; removed stale {pf}")
        return 0
    for _ in range(50):                       # up to 5 s for a clean close
        time.sleep(0.1)
        if not os.path.exists(pf):
            print(f"pid {pid} stopped, {csv_path} closed")
            return 0
    print(f"pid {pid} did NOT exit within 5 s -- STILL RUNNING, do not archive yet")
    return 2


def main():
    args = [a for a in sys.argv[1:]]
    if args and args[0] == "--stop":
        return stop(args[1])
    out = args[0]

    sys.path.insert(0, DRIVER_PATH)
    from DFRobot_matrixLidar import DFRobot_matrixLidar_i2c

    pf = pidfile_for(out)
    with open(pf, "w") as f:
        f.write(str(os.getpid()))             # OUR pid, never a wrapper's

    running = {"go": True}
    signal.signal(signal.SIGTERM, lambda *_: running.update(go=False))
    signal.signal(signal.SIGINT, lambda *_: running.update(go=False))

    tof = DFRobot_matrixLidar_i2c(0x33)
    tof.begin()
    while tof.set_Ranging_Mode(8) != 0:
        print("mode set failed, retrying", flush=True)
        time.sleep(1)

    seq = 0
    try:
        with open(out, "w", buffering=1) as f:
            f.write("epoch,iso,seq," + ",".join(f"z{i:02d}" for i in range(64)) + "\n")
            while running["go"]:
                try:
                    data = tof.get_all_data()
                except Exception as exc:      # a bad read must not end the run
                    print(f"read error: {exc}", flush=True)
                    time.sleep(0.05)
                    continue
                t = time.time()
                vals = [(data[i + 1] << 8) | data[i] for i in range(0, len(data) - 1, 2)]
                if len(vals) != 64:
                    print(f"short frame: {len(vals)} zones", flush=True)
                    continue
                f.write(
                    f"{t:.3f},"
                    f"{datetime.datetime.fromtimestamp(t).isoformat(timespec='milliseconds')},"
                    f"{seq}," + ",".join(str(v) for v in vals) + "\n"
                )
                seq += 1
                if seq % 50 == 0:
                    print(f"{seq} frames", flush=True)
    finally:
        # The pidfile is the stop signal's handshake: --stop waits for it to vanish,
        # so it must go away on EVERY exit path -- including the exception that
        # killed the 500-frame capture without saying anything.
        try:
            os.unlink(pf)
        except OSError:
            pass
        print(f"capture ended after {seq} frames -> {out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
