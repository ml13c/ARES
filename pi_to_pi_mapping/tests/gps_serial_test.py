#!/usr/bin/env python3
"""Exercise the Linux serial-NMEA path with a pseudo-terminal (no receiver)."""
import csv
import os
import pathlib
import pty
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


def sentence(body):
    checksum = 0
    for byte in body.encode("ascii"):
        checksum ^= byte
    return f"${body}*{checksum:02X}\r\n".encode("ascii")


def main(operator, rover):
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    with tempfile.TemporaryDirectory() as temp:
        destination = pathlib.Path(temp)
        track = destination / "gps_track.csv"
        port = free_port()
        with (destination / "operator.log").open("w+") as out:
            op = subprocess.Popen([operator, str(port), str(destination / "hazards.csv"),
                                   str(destination / "sightings.csv"), str(track)],
                                  stdout=out, stderr=subprocess.STDOUT)
            rover_proc = None
            try:
                time.sleep(0.2)
                rover_proc = subprocess.Popen(
                    [rover, "127.0.0.1", str(port), "--gps-serial", path,
                     "--baud", "9600", "--steps", "4"],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                time.sleep(0.25)
                for n in range(4):
                    body = (f"GNRMC,1235{n:02d},A,3700.0000,N,"
                            f"12200.{n:04d},W,3.0,90.0,031126,,,A")
                    os.write(master, sentence(body))
                    time.sleep(0.15)
                stdout, _ = rover_proc.communicate(timeout=5)
                if rover_proc.returncode != 0:
                    raise RuntimeError("serial rover failed: " + stdout)
                if stdout.count("Sent: GPS,") != 4:
                    raise RuntimeError("missing serial GPS fixes: " + stdout)
                time.sleep(0.1)
            finally:
                if rover_proc and rover_proc.poll() is None:
                    rover_proc.kill()
                    rover_proc.wait()
                if op.poll() is None:
                    op.send_signal(signal.SIGINT)
                op.wait(timeout=3)
            out.flush()
        with track.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        if len(rows) != 4:
            raise RuntimeError(f"operator saved {len(rows)} fixes, expected 4")
    os.close(slave)
    os.close(master)
    print("PASS: Linux pseudo-serial GPS input, rover transport, 4 logged fixes")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: gps_serial_test.py operator rover")
    try:
        main(*sys.argv[1:])
    except Exception as exc:
        raise SystemExit(f"GPS SERIAL TEST FAILED: {exc}")
