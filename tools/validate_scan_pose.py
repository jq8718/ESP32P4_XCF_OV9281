"""Compile the actual P4 pose C code on Windows and compare it with OpenCV.

Requires Python numpy/opencv and Visual Studio C tools. ESP/NVS stubs replace
platform services only; all pose arithmetic is compiled from scan_pose.c.
Run: py -3.12 tools/validate_scan_pose.py
"""
import ctypes
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/ov9281_debugger'))
from geometry import default_ov9281_calibration, estimate_pose


def compile_probe(folder):
    folder.mkdir(parents=True, exist_ok=True)
    stubs = {
        'esp_err.h': '''#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_ARG 1
#define ESP_ERR_INVALID_STATE 2
#define ESP_ERR_NVS_NOT_FOUND 3
static inline const char *esp_err_to_name(int err) { return "host stub"; }
''',
        'esp_log.h': '#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\n',
        'esp_timer.h': '#include <time.h>\nstatic inline long long esp_timer_get_time(void) { return (long long)(clock()*1000000.0/CLOCKS_PER_SEC); }\n',
        'freertos/FreeRTOS.h': '''typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(...) ((void)0)
#define portEXIT_CRITICAL(...) ((void)0)
''',
        'nvs_flash.h': 'static inline int nvs_flash_init(void) { return 0; }\n',
        'nvs.h': '''#include <stddef.h>
typedef int nvs_handle_t;
#define NVS_READONLY 0
#define NVS_READWRITE 1
static inline int nvs_open(const char *s, int mode, int *h) { *h=0; return 0; }
static inline void nvs_close(int h) {}
static inline int nvs_get_blob(int h, const char *k, void *v, size_t *s) { return 3; }
static inline int nvs_set_blob(int h, const char *k, const void *v, size_t s) { return 0; }
static inline int nvs_commit(int h) { return 0; }
''',
    }
    for name, content in stubs.items():
        target = folder / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content, encoding='utf-8')
    source = ROOT / 'port/standalone_idf_test/main/scan_pose.c'
    wrapper = folder / 'probe.c'
    wrapper.write_text('''#include "SOURCE"
__declspec(dllexport) int probe(const float *points, int prefix, float *values) {
    XcrBarcodeResult barcode = {0};
    barcode.geometry_valid=1; barcode.geometry_quality=2; barcode.subpixel_valid=1;
    barcode.length=1; barcode.data[0]=(char)prefix;
    memcpy(barcode.subpixel_corners, points, sizeof(barcode.subpixel_corners));
    scan_pose_result_t result;
    if (!scan_pose_estimate(&barcode, &result)) return 0;
    values[0]=result.yaw_y_deg; values[1]=result.pitch_x_deg; values[2]=result.roll_z_deg;
    values[3]=result.range_m; values[4]=result.optical_depth_m;
    values[5]=result.distance_to_plane_m; values[6]=result.tx_m;
    values[7]=result.reprojection_px;
    return 1;
}
'''.replace('SOURCE', source.as_posix()), encoding='utf-8')
    vswhere = Path('C:/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe')
    vs = subprocess.check_output([str(vswhere), '-latest', '-property', 'installationPath'], text=True).strip()
    vcvars = Path(vs) / 'VC/Auxiliary/Build/vcvars64.bat'
    batch = folder / 'compile.cmd'
    batch.write_text(f'@echo off\ncall "{vcvars}" >nul\n'
                     f'cl /nologo /O2 /std:c11 /LD /W3 /I"{folder}" '
                     f'/I"{ROOT / "port/components/xcr_decoder/include"}" '
                     f'"{wrapper}" /link /OUT:"{folder / "pose_probe.dll"}"\n', encoding='utf-8')
    result = subprocess.run(['cmd', '/c', str(batch)], cwd=folder, text=True, errors='replace', capture_output=True)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    library = ctypes.CDLL(str(folder / 'pose_probe.dll'))
    library.probe.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
    library.probe.restype = ctypes.c_int
    return library


def main():
    folder = ROOT / 'build/pose_validation'
    library = compile_probe(folder)
    calibration = default_ov9281_calibration()
    names = ['yaw_y_deg', 'pitch_x_deg', 'roll_z_deg', 'range_m', 'optical_depth_m',
             'distance_to_plane_m', 'lateral_offset_x_m']

    def p4(points, prefix):
        values = np.zeros(8, dtype=np.float32)
        points = np.ascontiguousarray(points, dtype=np.float32)
        ok = library.probe(points.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), prefix,
                           values.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
        return values if ok else None

    screenshot = np.array([[521.288, 287.283], [574.915, 294.349],
                           [569.467, 346.858], [515.192, 340.729]])
    cases = [('screenshot', screenshot, 30., None)]
    rng = np.random.default_rng(20261117)
    for index in range(800):
        side = 30. if index % 2 == 0 else 50.
        half = side / 2
        obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
        # Neutral pose is diag(1,-1,-1); include off-axis, rotated and tilted labels.
        angles = rng.uniform([-55, -45, -150], [55, 45, 150])
        perturbation, _ = cv2.Rodrigues(np.deg2rad(angles))
        matrix = perturbation @ np.diag([1., -1., -1.])
        rotation, _ = cv2.Rodrigues(matrix)
        depth = rng.uniform(180., 1200.)
        translation = np.array([rng.uniform(-.35, .35)*depth,
                                rng.uniform(-.2, .2)*depth, depth])
        points, _ = cv2.projectPoints(obj, rotation, translation,
                                     calibration['camera_matrix'], calibration['dist_coeffs'])
        points = points.reshape(4, 2)
        if np.any(points < 0) or np.any(points[:, 0] >= 1280) or np.any(points[:, 1] >= 720):
            continue
        noise = (0., .05, .2, .5)[index % 4]
        points = points + rng.normal(0, noise, points.shape)
        cases.append((f'synthetic_{index}_noise_{noise}', points, side, matrix if noise == 0 else None))

    failures, rows, elapsed = [], [], []
    for label, points, side, truth in cases:
        cal = dict(calibration, qr_size_mm=side)
        reference = estimate_pose(points, 1280, 720, cal)
        started = time.perf_counter()
        actual = p4(points, ord('0' if side == 30 else '1'))
        elapsed.append((time.perf_counter()-started)*1e6)
        if not reference['available'] or actual is None:
            failures.append({'case': label, 'reason': 'solver rejected', 'pc': reference, 'p4': None if actual is None else actual.tolist()})
            continue
        expected = np.array([reference[key] for key in names])
        difference = np.abs(actual[:7]-expected)
        difference[:3] = np.abs((actual[:3]-expected[:3]+180) % 360-180)
        rows.append({'case': label, 'p4': actual[:7].tolist(), 'opencv': expected.tolist(), 'difference': difference.tolist(),
                     'p4_error_px': float(actual[7]), 'pc_error_px': reference['reprojection_px'], 'points': points.tolist()})
        if max(difference[:3]) > .1 or max(difference[3:]) > .0002:
            failures.append(rows[-1])

    bad_inputs = [screenshot*np.nan, np.zeros((4, 2)), screenshot-2000]
    for index, bad in enumerate(bad_inputs):
        if p4(bad, ord('0')) is not None:
            failures.append({'case': f'invalid_{index}', 'reason': 'accepted invalid input'})
    if p4(screenshot, ord('2')) is not None:
        failures.append({'case': 'invalid_prefix', 'reason': 'accepted unsupported size'})
    report = {
        'source_sha256': hashlib.sha256((ROOT / 'port/standalone_idf_test/main/scan_pose.c').read_bytes()).hexdigest(),
        'cases': len(cases), 'failures': len(failures), 'screenshot': rows[0],
        'maximum_difference': np.max([row['difference'] for row in rows], axis=0).tolist(),
        'host_mean_us': float(np.mean(elapsed)), 'host_p95_us': float(np.percentile(elapsed, 95)),
        'timing_note': 'Windows host timings include ctypes; these are not ESP32-P4 timings.',
        'failure_details': failures,
    }
    (folder / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: value for key, value in report.items() if key != 'failure_details'}, indent=2))
    if failures:
        print('First failures:', json.dumps(failures[:3], ensure_ascii=False, indent=2))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
