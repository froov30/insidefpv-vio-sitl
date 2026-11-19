#!/usr/bin/env python3
"""Automated acceptance test for Task 2: takeoff, hover, land, disarm.

This is the test the brief asks for: no GUI, no human watching a video. It
starts a fresh Gazebo server (gz sim -s, headless, camera still rendered),
starts a fresh ArduCopter SITL, flies takeoff_land.py once, and then asserts
on the files that flight produced.

What is asserted, and against which file
    1. The flight script exited 0                          terminal.log
    2. The CSV has >= MIN_SAMPLES rows                     flight.csv
    3. Phases climb, hover, land appear, in that order      flight.csv
    4. Peak relative_alt is inside [4.5, 5.5] m            flight.csv
    5. The final relative_alt is below LANDED_MAX_ALT_M    flight.csv
    6. The autopilot disarmed at the end                   summary + heartbeat log
    7. A DataFlash .BIN was written and is non-empty       logs/sitl/logs

Every check reads the CSV itself rather than trusting the summary JSON, and
then cross-checks the summary against the CSV. Two independent readings of
the same evidence is what makes "it passed" mean something.

Usage
    python3 scripts/test_task2.py                 # two runs, the brief's minimum
    python3 scripts/test_task2.py --repeat 1      # one run, for a quick check

Output layout (one directory per run, all under results/task2/):
    flight.csv, flight.summary.json   the flight evidence
    terminal.log                      supervisor + ground station stdout
    gazebo.log, sitl_console.log      the pair's own logs
    <timestamp>.BIN                   DataFlash log, if SITL wrote one

Exit status is 0 only if every run passed every check.
"""

import argparse
import csv
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# The flight the test demands. These are the brief's numbers, not the
# script's defaults: takeoff to 5 m, accept +-0.5 m, and require the
# aircraft to be back on the ground at the end. A change to
# takeoff_land.py's TARGET_ALTITUDE_M that is not reflected here makes the
# test fail, which is the intended behaviour.
EXPECTED_ALTITUDE_M = 5.0
ALTITUDE_TOLERANCE_M = 0.5
PEAK_MIN_M = EXPECTED_ALTITUDE_M - ALTITUDE_TOLERANCE_M
PEAK_MAX_M = EXPECTED_ALTITUDE_M + ALTITUDE_TOLERANCE_M
LANDED_MAX_ALT_M = 0.5

# A 10 Hz log over an 8 s hover plus climb and descent is well over 100
# rows. Requiring 50 still fails immediately if telemetry was never
# requested, without becoming flaky on a slow machine.
MIN_SAMPLES = 50

# Phase order. "climb" first, then "hover", then "land" - if any phase is
# missing or out of order the flight did not do what Task 2 describes.
EXPECTED_PHASES = ["climb", "hover", "land"]

# Wall-clock budgets. Gazebo cold start with rendering is ~10-20 s on this
# machine; SITL adds a few more. The flight itself is ~40 s of simulated
# time, which at real-time factor 1 is ~40 s of wall clock.
SIM_READY_TIMEOUT_S = 180
FLIGHT_TIMEOUT_S = 420

# Where the supervised pair writes its logs (scripts/run_sim.sh).
SIM_LOG_DIR = ROOT / "logs" / "sim"
SITL_LOG_DIR = ROOT / "logs" / "sitl" / "logs"


def shell(script, *args, timeout=None):
    """Run a project script from the repository root and return CompletedProcess.

    Every child is run through bash with the script path relative to ROOT,
    so the scripts see the same working directory a human would use.
    """
    command = ["bash", f"scripts/{script}", *args]
    return subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        capture_output=True,
        timeout=timeout,
    )


def stop_pair():
    """Kill any leftover Gazebo/SITL pair.

    This is not optional hygiene: SITL's SERIAL0 accepts one client ever
    (see scripts/run_sitl.sh), so a survivor from an earlier attempt makes
    the next run fail with "No heartbeat" for a reason that has nothing to
    do with the code under test.
    """
    shell("stop_sim.sh")


def start_pair(run_dir):
    """Start run_sim.sh in its own session and wait for SIM_READY.

    The supervisor's output goes to a file, not a pipe: a pipe would block
    the supervisor once the buffer filled, and the file is also the
    terminal.log evidence for this run. start_new_session=True puts the
    whole process group (supervisor, Gazebo, SITL) in a new session so
    teardown can signal all three at once.
    """
    supervisor_log = run_dir / "supervisor.log"
    handle = supervisor_log.open("w")
    process = subprocess.Popen(
        ["bash", "scripts/run_sim.sh", "--headless"],
        cwd=ROOT,
        stdout=handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    deadline = time.time() + SIM_READY_TIMEOUT_S
    while time.time() < deadline:
        # A dead supervisor will never print SIM_READY; fail fast with its
        # log instead of waiting out the whole timeout.
        if process.poll() is not None:
            handle.close()
            raise RuntimeError(
                f"run_sim.sh exited with {process.returncode} before SIM_READY.\n"
                + supervisor_log.read_text()[-4000:]
            )
        if "SIM_READY" in supervisor_log.read_text(errors="replace"):
            return process, handle
        time.sleep(0.5)
    handle.close()
    terminate(process)
    raise RuntimeError(
        f"Simulation not ready within {SIM_READY_TIMEOUT_S} s.\n"
        + supervisor_log.read_text(errors="replace")[-4000:]
    )


def terminate(process):
    """Stop the supervisor and everything it started.

    SIGTERM to the process group reaches the supervisor, which has its own
    trap that terminates Gazebo and SITL in that order. SITL is given a
    few seconds to flush its DataFlash log; only then is SIGKILL used.
    """
    if process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=10)
    # The supervisor's trap runs asynchronously; stop_sim.sh is the
    # backstop that guarantees nothing survives into the next run.
    stop_pair()


def collect_logs(run_dir):
    """Copy the pair's logs and the newest DataFlash file into run_dir.

    Copied rather than moved so logs/sim/ still shows the most recent run
    for someone reading the terminal after the test.
    """
    for name in ("gazebo.log", "sitl_console.log"):
        source = SIM_LOG_DIR / name
        if source.exists():
            shutil.copy2(source, run_dir / name)
    if SITL_LOG_DIR.is_dir():
        bins = sorted(SITL_LOG_DIR.glob("*.BIN"), key=lambda p: p.stat().st_mtime)
        if bins:
            shutil.copy2(bins[-1], run_dir / bins[-1].name)


def read_csv(csv_path):
    """Return (rows, phases_in_order, peak_alt, final_alt, duration_s).

    phases_in_order keeps first-appearance order, so a land that happened
    before takeoff would be visible as the wrong sequence rather than as
    the right set.
    """
    rows = []
    phases = []
    peak = 0.0
    final = 0.0
    first_ms = None
    last_ms = None
    with csv_path.open() as handle:
        for row in csv.DictReader(handle):
            rows.append(row)
            phase = row["phase"]
            if not phases or phases[-1] != phase:
                phases.append(phase)
            altitude = float(row["relative_alt_m"])
            peak = max(peak, altitude)
            final = altitude
            boot_ms = int(row["time_boot_ms"])
            if first_ms is None:
                first_ms = boot_ms
            last_ms = boot_ms
    duration = 0.0
    if first_ms is not None and last_ms is not None:
        duration = (last_ms - first_ms) / 1000.0
    return rows, phases, peak, final, duration


def check_run(run_dir, flight_result):
    """Run every acceptance check and return a list of (name, ok, detail).

    Each check is independent so one failure does not mask another; the
    details string carries the observed number, which is what ends up in
    the evidence table.
    """
    checks = []

    def record(name, ok, detail):
        checks.append((name, bool(ok), detail))

    record("ground station exited 0", flight_result.returncode == 0,
           f"exit {flight_result.returncode}")

    csv_path = run_dir / "flight.csv"
    summary_path = run_dir / "flight.summary.json"
    if not csv_path.exists():
        record("flight.csv exists", False, f"missing {csv_path}")
        return checks
    rows, phases, peak, final, duration = read_csv(csv_path)
    record("flight.csv readable", len(rows) > 0, f"{len(rows)} rows")
    record("samples >= minimum", len(rows) >= MIN_SAMPLES,
           f"{len(rows)} >= {MIN_SAMPLES}")
    record("phases in order", phases == EXPECTED_PHASES,
           f"{phases} vs {EXPECTED_PHASES}")
    record("peak altitude in band", PEAK_MIN_M <= peak <= PEAK_MAX_M,
           f"{peak:.2f} m in [{PEAK_MIN_M}, {PEAK_MAX_M}]")
    record("landed below threshold", final <= LANDED_MAX_ALT_M,
           f"{final:.2f} m <= {LANDED_MAX_ALT_M}")
    record("flight duration plausible", duration > 0, f"{duration:.2f} s autopilot time")

    disarmed = False
    if summary_path.exists():
        summary = json.loads(summary_path.read_text())
        disarmed = bool(summary.get("disarmed"))
        # The summary is derived from this CSV. If the two disagree, one
        # of them is being computed from something else, and both stop
        # being evidence.
        agrees = abs(float(summary["peak_relative_alt_m"]) - peak) < 0.01
        record("summary agrees with csv", agrees,
               f"summary {summary['peak_relative_alt_m']} csv {peak:.3f}")
    else:
        record("flight.summary.json exists", False, "missing")
    record("disarmed at end", disarmed, "MAV_MODE_FLAG_SAFETY_ARMED cleared")

    bins = sorted(run_dir.glob("*.BIN"))
    record("DataFlash log present", bool(bins) and bins[-1].stat().st_size > 0,
           f"{bins[-1].name} ({bins[-1].stat().st_size} bytes)" if bins else "none")

    return checks


def run_once(index, results_root):
    """Fly one complete mission and return (run_dir, checks, ok)."""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = results_root / f"run{index}_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"\n=== Run {index} -> {run_dir.relative_to(ROOT)} ===")

    stop_pair()
    pair, handle = start_pair(run_dir)
    print("Simulation is ready; flying")
    try:
        flight_result = subprocess.run(
            [sys.executable, "scripts/takeoff_land.py",
             "--log", str(run_dir / "flight.csv")],
            cwd=ROOT,
            text=True,
            capture_output=True,
            timeout=FLIGHT_TIMEOUT_S,
        )
    except subprocess.TimeoutExpired as error:
        # A hung flight is still evidence: keep whatever it printed.
        (run_dir / "terminal.log").write_text(
            f"takeoff_land.py timed out after {FLIGHT_TIMEOUT_S} s\n"
            f"stdout:\n{error.stdout}\nstderr:\n{error.stderr}\n"
        )
        terminate(pair)
        collect_logs(run_dir)
        return run_dir, [("flight completed", False, "timeout")], False

    (run_dir / "terminal.log").write_text(
        flight_result.stdout + "\n--- stderr ---\n" + flight_result.stderr
    )
    terminate(pair)
    handle.close()
    collect_logs(run_dir)

    checks = check_run(run_dir, flight_result)
    ok = all(passed for _, passed, _ in checks)
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}: {detail}")
    return run_dir, checks, ok


def main():
    parser = argparse.ArgumentParser(description="Headless acceptance test for Task 2")
    parser.add_argument("--repeat", type=int, default=2,
                        help="Number of consecutive runs; the brief requires 2")
    parser.add_argument("--results", default=str(ROOT / "results" / "task2"),
                        help="Directory for per-run evidence")
    args = parser.parse_args()

    results_root = Path(args.results)
    results_root.mkdir(parents=True, exist_ok=True)

    outcomes = []
    for index in range(1, args.repeat + 1):
        run_dir, checks, ok = run_once(index, results_root)
        outcomes.append((index, run_dir, checks, ok))
        if not ok:
            # Stop at the first failing run: a later pass would not make
            # the pair consecutive, which is what "passes twice in a row"
            # means.
            print(f"\nRun {index} FAILED. Fix the cause before re-running.")
            break

    print("\n=== Summary ===")
    all_ok = len(outcomes) == args.repeat and all(ok for *_, ok in outcomes)
    for index, run_dir, checks, ok in outcomes:
        failed = [name for name, passed, _ in checks if not passed]
        state = "PASS" if ok else "FAIL"
        print(f"run {index}: {state}  {run_dir.relative_to(ROOT)}"
              + (f"  failed: {', '.join(failed)}" if failed else ""))
    print("RESULT:", "PASS" if all_ok else "FAIL")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
