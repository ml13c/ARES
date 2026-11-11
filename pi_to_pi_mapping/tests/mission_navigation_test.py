#!/usr/bin/env python3
"""Two-process mission simulation, actual TCP, no live GPS or actuation."""
import csv
import pathlib
import signal
import socket
import subprocess
import sys
import tempfile
import time


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_for_server(port, process):
    for _ in range(100):
        if process.poll() is not None:
            raise RuntimeError("operator exited")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.05):
                return
        except OSError:
            time.sleep(0.02)
    raise RuntimeError("operator not listening")


def rows(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def main(operator, rover, mission):
    # Invalid missions must be rejected before attempting any network connection.
    with tempfile.TemporaryDirectory() as tmp:
        folder = pathlib.Path(tmp)
        invalid = folder / "invalid.csv"
        invalid.write_text("BOUNDARY,37,-122\nBOUNDARY,37,-121.999\n"
                           "BOUNDARY,37.001,-121.999\nBOUNDARY,37.001,-122\n"
                           "WAYPOINT,38,-122\nWAYPOINT,38,-121.999\n")
        fail = subprocess.run([rover, "127.0.0.1", "5000", "--mission-sim", str(invalid)],
                              capture_output=True, text=True, timeout=3)
        if fail.returncode == 0 or "UNSAFE MISSION REJECTED" not in fail.stderr:
            raise RuntimeError("unsafe mission was not rejected")

        logs = folder / "operator.log"
        hazards = folder / "hazards.csv"
        sightings = folder / "sightings.csv"
        track = folder / "track.csv"
        port = free_port()
        with logs.open("w+") as output:
            op = subprocess.Popen([operator, str(port), str(hazards), str(sightings), str(track)],
                                  stdout=output, stderr=subprocess.STDOUT)
            try:
                wait_for_server(port, op)
                run = subprocess.run([rover, "127.0.0.1", str(port),
                                      "--mission-sim", mission,
                                      "--speed-mps", "30", "--interval-ms", "10",
                                      "--mock-detections"],
                                     capture_output=True, text=True, timeout=12)
                if run.returncode:
                    raise RuntimeError("mission rover failed: " + run.stderr + run.stdout)
                for expected in ["Sent: BOUNDARY,", "Sent: WAYPOINT,",
                                 "Sent: GPS,", "Sent: SIGHTING,", "Sent: NAV,COMPLETE,"]:
                    if expected not in run.stdout:
                        raise RuntimeError("missing " + expected)
                if "Sent: HAZARD," in run.stdout:
                    raise RuntimeError("mission mode invented hazard range/object GPS")
                time.sleep(0.2)
            finally:
                if op.poll() is None:
                    op.send_signal(signal.SIGINT)
                try:
                    op.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    op.kill()
                    op.wait()
            output.flush()
        actual_track = rows(track)
        actual_sightings = rows(sightings)
        if len(actual_track) < 15 or len(actual_sightings) < 1:
            raise RuntimeError("missing GPS track or simulated hazard sightings")
        if len(rows(hazards)):
            raise RuntimeError("sightings incorrectly recorded as localized hazards")
        console = logs.read_text(errors="replace")
        if "GPS FIELD VIEW" not in console or "Simulation navigation: COMPLETE" not in console:
            raise RuntimeError("operator did not render completed GPS field mission")
        if "object location UNKNOWN" not in console:
            raise RuntimeError("operator falsely localized an object")
        print(f"PASS: GPS geofence + route display, {len(actual_track)} fixes, "
              f"{len(actual_sightings)} sightings, mission complete")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        raise SystemExit("usage: mission_navigation_test.py operator rover mission.csv")
    try:
        main(*sys.argv[1:])
    except Exception as exc:
        raise SystemExit("MISSION INTEGRATION FAILED: " + str(exc))
