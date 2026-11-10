"""Verify the ESP32 UVC identity, decoded image, and observed frame rates.
Requires: opencv-python, cv2-enumerate-cameras, pyserial.
"""
import json
import pathlib
import threading
import time
import zlib

import cv2
import serial
from serial.tools import list_ports
from cv2_enumerate_cameras import enumerate_cameras

root = pathlib.Path(__file__).resolve().parents[1]
log_dir = root / 'docs'
log_dir.mkdir(exist_ok=True)
ports = [p.device for p in list_ports.comports() if p.vid == 0x303A and p.pid == 0x1001]
if len(ports) != 1:
    raise RuntimeError(f'Expected one ESP32 Serial/JTAG port; found {ports}')
port = serial.Serial(ports[0], 115200, timeout=0.1)
stop = threading.Event()

def drain():
    with (log_dir / 'uvc_board_check.log').open('wb') as log:
        while not stop.is_set():
            data = port.read(4096)
            if data:
                log.write(data)
                log.flush()

thread = threading.Thread(target=drain)
thread.start()
cap = None
try:
    port.dtr = False
    port.rts = False
    port.dtr = True
    time.sleep(0.2)
    port.dtr = False
    time.sleep(6)
    cameras = [c for c in enumerate_cameras(cv2.CAP_DSHOW)
               if c.vid == 0x303A and c.pid == 0x8001]
    if len(cameras) != 1:
        raise RuntimeError(f'Expected one ESP32 UVC device; found {cameras}')
    camera = cameras[0]
    # The index is resolved from VID/PID on every run, never assumed.
    print('Checking:', camera.path, flush=True)
    cap = cv2.VideoCapture(camera.index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        raise RuntimeError('Could not open the ESP32 camera')
    for _ in range(5):
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError('Could not decode the initial UVC frames')
    checksums = set()
    start = time.monotonic()
    for index in range(90):
        ok, frame = cap.read()
        if not ok or frame is None or frame.shape[:2] != (720, 1280):
            raise RuntimeError(f'Invalid decoded frame {index}')
        checksums.add(zlib.crc32(frame))
        if index == 0 and not cv2.imwrite(str(log_dir / 'uvc_verified.png'), frame):
            raise RuntimeError('Could not save the verified image')
    elapsed = time.monotonic() - start
    if len(checksums) < 2:
        raise RuntimeError('Only a repeated image was received')
    report = {
        'device': camera.path, 'serial_port': ports[0],
        'width': frame.shape[1], 'height': frame.shape[0],
        'requested_fps': 30, 'frames': 90, 'seconds': round(elapsed, 3),
        'received_fps': round(90 / elapsed, 2),
        'distinct_frames': len(checksums),
        'distinct_fps': round(len(checksums) / elapsed, 2),
        'mean': round(float(frame.mean()), 2),
        'stddev': round(float(frame.std()), 2),
    }
    (log_dir / 'uvc_check.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)
finally:
    if cap is not None:
        cap.release()
    stop.set()
    thread.join()
    port.close()
