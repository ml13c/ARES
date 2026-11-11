#!/usr/bin/env python3
"""End-to-end smoke test using two C++ processes and a local TCP connection.

Uses only the Python standard library. Mock sensors and rover positions are
simulated; passing this test does NOT validate hardware or NFR3 latency.
"""
import csv
import pathlib
import signal
import socket
import subprocess
import sys
import tempfile
import time


def find_free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_for_server(port, proc):
    for _ in range(100):
        if proc.poll() is not None:
            raise RuntimeError("Operator exited before accepting a connection")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.05):
                return
        except OSError:
            time.sleep(0.02)
    raise RuntimeError("Operator did not start within 2 seconds")


def main():
    if len(sys.argv) != 3:
        print("usage: integration_test.py <operator-path> <rover-path>", file=sys.stderr)
        return 2
    operator_path, rover_path = sys.argv[1:]
    with tempfile.TemporaryDirectory() as tmp:
        temp = pathlib.Path(tmp)
        csv_path = temp / "hazards.csv"
        log_path = temp / "operator.log"
        port = find_free_port()
        with log_path.open("w+") as output:
            op = subprocess.Popen([operator_path, str(port), str(csv_path)],
                                  stdout=output, stderr=subprocess.STDOUT)
            try:
                wait_for_server(port, op)
                rover = subprocess.run(
                    [rover_path, "127.0.0.1", str(port), "14", "15"],
                    capture_output=True, text=True, timeout=8, check=False)
                if rover.returncode != 0:
                    raise RuntimeError("Rover failed: " + rover.stdout + rover.stderr)
                if "HAZARD" not in rover.stdout or "POS" not in rover.stdout:
                    raise RuntimeError("Rover did not send positions and hazards")
                # Allow operator to drain the TCP stream after rover closes.
                time.sleep(0.1)
            finally:
                if op.poll() is None:
                    op.send_signal(signal.SIGINT)
                try:
                    op.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    op.kill()
                    op.wait()
            output.flush()
        with csv_path.open(newline="") as file:
            rows = list(csv.DictReader(file))
        if not rows:
            raise RuntimeError("Operator did not save any hazards to CSV")
        for row in rows:
            if row["type"] not in ("ROCK", "WEED", "OTHER"):
                raise RuntimeError("Unexpected hazard type " + str(row))
            if not (36.9 < float(row["latitude_deg"]) < 37.1 and
                    -122.1 < float(row["longitude_deg"]) < -121.9):
                raise RuntimeError("Invalid simulated GPS coordinates " + str(row))
            if float(row["range_m"]) != 1.0:
                raise RuntimeError("Unexpected simulated range " + str(row))
        log = log_path.read_text(errors="replace")
        if "GPS (" not in log or "Saved hazards:" not in log:
            raise RuntimeError("Operator map did not display GPS hazards")
        print(f"PASS: rover/operator TCP, GPS display, {len(rows)} logged hazards")
        return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("INTEGRATION TEST FAILED:", exc, file=sys.stderr)
        sys.exit(1)
