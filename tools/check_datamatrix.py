"""Check the live OV9281 DataMatrix result over UVC and CDC.

Requires opencv-python, cv2-enumerate-cameras, zxing-cpp, and pyserial.
Usage: python tools/check_datamatrix.py [expected_payload]
"""

import json
import pathlib
import sys
import threading
import time

import cv2
import serial
import zxingcpp
from cv2_enumerate_cameras import enumerate_cameras
from serial.tools import list_ports


expected = sys.argv[1] if len(sys.argv) > 1 else "012345"
output = pathlib.Path(__file__).resolve().parents[1] / "docs"
output.mkdir(exist_ok=True)
ports = [p.device for p in list_ports.comports()
         if p.vid == 0x303A and p.pid == 0x8001]
cameras = [c for c in enumerate_cameras(cv2.CAP_DSHOW)
           if c.vid == 0x303A and c.pid == 0x8001]
if len(ports) != 1 or len(cameras) != 1:
    raise RuntimeError(f"Expected one ESP32 CDC and UVC device: {ports}, {cameras}")

camera = cameras[0]
port = serial.Serial(ports[0], 115200, timeout=0.1)
port.dtr = True
stop = threading.Event()
received = bytearray()


def read_cdc():
    while not stop.is_set():
        data = port.read(4096)
        if data:
            received.extend(data)


thread = threading.Thread(target=read_cdc)
thread.start()
cap = cv2.VideoCapture(camera.index, cv2.CAP_DSHOW)
try:
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        raise RuntimeError("ESP32 UVC camera could not be opened")
    frames = 0
    matches = []
    first_bounds = None
    start = time.monotonic()
    while time.monotonic() - start < 8:
        ok, frame = cap.read()
        if not ok or frame.shape[:2] != (720, 1280):
            continue
        frames += 1
        if frames % 10:
            continue
        for code in zxingcpp.read_barcodes(
                frame, formats=zxingcpp.BarcodeFormat.DataMatrix):
            if code.text != expected:
                continue
            corners = code.position
            xs = [corners.top_left.x, corners.top_right.x,
                  corners.bottom_left.x, corners.bottom_right.x]
            ys = [corners.top_left.y, corners.top_right.y,
                  corners.bottom_left.y, corners.bottom_right.y]
            bounds = [min(xs), min(ys), max(xs), max(ys)]
            matches.append(bounds)
            if first_bounds is None:
                first_bounds = bounds
                cv2.imwrite(str(output / "dm_verified_uvc.png"), frame)
    elapsed = time.monotonic() - start
finally:
    cap.release()
    stop.set()
    thread.join()
    port.close()

(output / "dm_verified_cdc.log").write_bytes(received)
results = []
for raw in received.split(b"\r"):
    fields = raw.decode("utf-8", errors="replace").split(",")
    if len(fields) < 12 or fields[0] != expected:
        continue
    try:
        corners = list(map(int, fields[1:9]))
        sequence, capture_us, age_us = map(int, fields[9:12])
    except ValueError:
        continue
    bounds = [min(corners[::2]), min(corners[1::2]),
              max(corners[::2]), max(corners[1::2])]
    if not (0 <= bounds[0] < bounds[2] < 1280 and
            0 <= bounds[1] < bounds[3] < 720 and
            0 <= age_us < 100000):
        continue
    results.append({"bounds": bounds, "sequence": sequence,
                    "capture_us": capture_us, "age_us": age_us})

if not matches:
    raise RuntimeError(f"UVC did not show DataMatrix {expected!r}")
if not results:
    raise RuntimeError(f"CDC did not report DataMatrix {expected!r}")
if not any(all(abs(a - b) <= 12 for a, b in zip(first_bounds, result["bounds"]))
           for result in results):
    raise RuntimeError("UVC and CDC DataMatrix coordinates disagree")

report = {
    "payload": expected, "camera": camera.path, "cdc_port": ports[0],
    "uvc_frames": frames, "seconds": round(elapsed, 3),
    "uvc_fps": round(frames / elapsed, 2),
    "uvc_decoded_samples": len(matches), "cdc_results": len(results),
    "uvc_bounds": first_bounds, "cdc_example": results[0],
}
(output / "dm_check.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2))
