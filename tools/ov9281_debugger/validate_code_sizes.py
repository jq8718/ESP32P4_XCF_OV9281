"""Verify the real window's size selection against the actual P4 pose C code.

Runs without opening a serial port. Writes screenshots/report under build/.
"""
import ctypes
import json
import os
from argparse import Namespace
from pathlib import Path
import sys

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import numpy as np
import cv2
from PySide6.QtWidgets import QApplication

from app import DebugWindow
from geometry import code_side_mm, load_calibration
from protocol import Snapshot

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from validate_scan_pose import compile_probe


def main():
    output = ROOT / 'build/ui_size_sync/source_validation'
    output.mkdir(parents=True, exist_ok=True)
    p4 = compile_probe(output / 'pose_probe')
    application = QApplication([])
    window = DebugWindow(Namespace(port=None, auto_connect=False, smoke_seconds=0))
    window.smooth_angles.setChecked(False)
    window.show()
    application.processEvents()
    points = np.array([[521.288, 287.283], [574.915, 294.349],
                       [569.467, 346.858], [515.192, 340.729]], dtype=np.float32)
    matrix_before = window.calibration['camera_matrix'].copy()
    # Import a legacy JSON with a conflicting side to ensure prefix wins.
    calibration_path = output / 'legacy_calibration.json'
    calibration_path.write_text(json.dumps(dict(
        camera_matrix=matrix_before.tolist(), image_size=[1280, 720],
        dist_coeffs=window.calibration['dist_coeffs'].tolist(), qr_size_mm=999.)), encoding='utf-8')
    window.calibration = load_calibration(calibration_path)
    names = ('yaw_y_deg', 'pitch_x_deg', 'roll_z_deg', 'range_m', 'optical_depth_m',
             'distance_to_plane_m', 'lateral_offset_x_m')
    rows = []
    encoded, jpeg = cv2.imencode('.jpg', np.full((720, 1280), 220, dtype=np.uint8))
    assert encoded
    for prefix, side in (('0', 30), ('1', 50), ('2', 70)):
        payload = (prefix+'12345').encode('ascii')
        assert code_side_mm(payload) == code_side_mm(payload.decode()) == side
        for coordinates_only in (False, True):
            frame = Snapshot(1, 1000000, 1280, 720, True, 20000,
                points.tolist(), payload, coordinates_only=coordinates_only,
                jpeg=jpeg.tobytes(),
                geometry_quality=2, geometry_edge_mask=15, geometry_support=64)
            window.display_frame(frame)
            application.processEvents()
            assert window.pose['available'], window.pose
            assert window.pose['qr_size_mm'] == side
            assert f'{side:.1f} mm' in window.pose_details.text()
            assert window.payload.toPlainText() == payload.decode()
            assert window.calibration['qr_size_mm'] == 999.
            assert np.array_equal(window.calibration['camera_matrix'], matrix_before)
            values = np.zeros(9, dtype=np.float32)
            assert p4.probe(points.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), ord(prefix),
                values.ctypes.data_as(ctypes.POINTER(ctypes.c_float))) == 1
            expected = np.array([window.pose[name] for name in names])
            difference = np.abs(expected-values[:7])
            assert max(difference[:3]) < .1, difference
            assert max(difference[3:]) < .0005, difference
            rows.append(dict(prefix=prefix, side_mm=side, coordinates_only=coordinates_only,
                             ui_values=expected.tolist(), p4_values=values[:7].tolist()))
        window.grab().save(str(output / f'prefix_{prefix}.png'))
    # An unsupported prefix must clear the previous valid distances.
    for payload in (b'312345', b'912345', b'', b'A12345'):
        assert code_side_mm(payload) is None
        window.display_frame(Snapshot(2, 2000000, 1280, 720, True, 20000,
            points.tolist(), payload, coordinates_only=True, geometry_quality=2))
        assert not window.pose['available']
        assert '不可计算' in window.tx_offset.text()
        assert '不可计算' in window.plane_distance.text()
    report = dict(status='passed', version=window.windowTitle(),
                  checks=['prefix 0/1/2 in image and coordinate-only paths',
                          'P4 C pose agreement', 'legacy JSON cannot override size',
                          'unknown prefix clears pose', 'calibration is not mutated'], cases=rows)
    (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    window.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
