#!/usr/bin/env python3
"""Replay GPS NMEA through rover -> TCP -> operator, without GPS hardware."""
import csv
import pathlib
import signal
import socket
import subprocess
import sys
import tempfile
import time


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_listening(port, process):
    for _ in range(100):
        if process.poll() is not None:
            raise RuntimeError("operator exited before listen")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.05):
                return
        except OSError:
            time.sleep(0.02)
    raise RuntimeError("operator did not start")


def rows(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def main(operator, rover, nmea):
    with tempfile.TemporaryDirectory() as tmp:
        base = pathlib.Path(tmp)
        hazard = base / "hazards.csv"
        sightings = base / "sightings.csv"
        track = base / "gps_track.csv"
        log = base / "operator.log"
        port = free_port()
        with log.open("w+") as output:
            op = subprocess.Popen([operator, str(port), str(hazard),
                                   str(sightings), str(track)],
                                  stdout=output, stderr=subprocess.STDOUT)
            try:
                wait_listening(port, op)
                run = subprocess.run([rover, "127.0.0.1", str(port),
                                      "--gps-replay", nmea,
                                      "--steps", "15", "--interval-ms", "10",
                                      "--mock-detections"],
                                     capture_output=True, text=True, timeout=8)
                if run.returncode != 0:
                    raise RuntimeError("rover failed: " + run.stdout + run.stderr)
                if run.stdout.count("Sent: GPS,") != 15:
                    raise RuntimeError("expected 15 valid GPS records: " + run.stdout)
                if "Sent: SIGHTING," not in run.stdout:
                    raise RuntimeError("no sightings transmitted")
                if "Sent: HAZARD," in run.stdout:
                    raise RuntimeError("GPS mode must not invent object coordinates")
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
        fixes = rows(track)
        observed = rows(sightings)
        if len(fixes) != 15:
            raise RuntimeError(f"expected 15 recorded GPS fixes, got {len(fixes)}")
        if len(observed) < 1:
            raise RuntimeError("no sighting rows")
        if any(row["type"] not in {"ROCK", "WEED", "OTHER"} for row in observed):
            raise RuntimeError("unknown sighting class")
        if any(abs(float(row["rover_latitude_deg"]) - 37) > 0.01 for row in observed):
            raise RuntimeError("incorrect rover GPS in sighting")
        if any("object" in key or "range" in key for key in observed[0]):
            raise RuntimeError("sighting CSV must not invent object location")
        if len(rows(hazard)) != 0:
            raise RuntimeError("GPS sightings must not be logged as localized hazards")
        if "LIVE GPS rover" not in log.read_text(errors="replace"):
            raise RuntimeError("operator did not display live GPS position")
        print(f"PASS GPS replay: {len(fixes)} fixes, {len(observed)} approximate sightings")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        raise SystemExit("usage: gps_replay_test.py operator rover sample.nmea")
    try:
        main(*sys.argv[1:])
    except Exception as error:
        raise SystemExit(f"GPS INTEGRATION FAILED: {error}")
