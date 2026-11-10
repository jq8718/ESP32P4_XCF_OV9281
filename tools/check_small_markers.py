"""Same-frame marker regression; run with WSL python3 after building both traces.

dm_stage_trace_baseline is built from the pre-fix source saved under build/.
Host timings do not represent ESP32 timing. Synthetic shifts test grid phase,
not independent camera captures.
"""
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 1280, 720
OUT = ROOT / 'docs/failure_diagnosis'
CASES = ROOT / 'build/small_marker_cases'


def decode(binary, path):
    result = subprocess.run([str(ROOT / 'docs/decode_stages' / binary), str(path)],
                            env={**os.environ, 'XCR_DM_PREPARED': '1'},
                            capture_output=True, text=True)
    if result.returncode not in (0, 1) or not result.stdout.strip():
        raise RuntimeError(result.stderr)
    return json.loads(result.stdout)


def main():
    CASES.mkdir(parents=True, exist_ok=True)
    report = []
    for name in ('fail01', 'sample02', 'fail03', 'fail04', 'current06'):
        source = (OUT / f'{name}.gray8').read_bytes()
        # All 6x6 phases of the original grid, translating the complete frame.
        for dy in range(6):
            for dx in range(6):
                shifted = bytearray([128]) * (WIDTH * HEIGHT)
                for y in range(HEIGHT - dy):
                    start = (y + dy) * WIDTH + dx
                    shifted[start:start + WIDTH - dx] = source[y * WIDTH:(y + 1) * WIDTH - dx]
                path = CASES / 'shift.gray8'
                path.write_bytes(shifted)
                before = decode('dm_stage_trace_baseline', path)
                after = decode('dm_stage_trace', path)
                report.append(dict(source=name, dx=dx, dy=dy, before=before, after=after))
        subset = [r for r in report if r['source'] == name]
        print(name, 'before', sum(r['before']['geometry_quality'] == 2 for r in subset),
              'after', sum(r['after']['geometry_quality'] == 2 for r in subset), '/36', flush=True)
    for path in sorted((ROOT / 'build/parallel_host_cases').glob('*.gray8')):
        before, after = (decode(binary, path) for binary in ('dm_stage_trace_baseline', 'dm_stage_trace'))
        report.append(dict(source=path.stem, before=before, after=after))
    (OUT / 'same_frame_regression.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    wrong = [r for r in report if r['after']['decoded'] and r['after']['payload'] != '012345']
    lost = [r for r in report if r['before']['geometry_quality'] == 2 and r['after']['geometry_quality'] != 2]
    negative = [r for r in report if r['source'] in ('blank', 'covered') and r['after']['decoded']]
    if wrong or lost or negative:
        raise SystemExit(f'wrong={len(wrong)} lost={len(lost)} negative={len(negative)}')


if __name__ == '__main__':
    main()
