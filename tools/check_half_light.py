"""Replay captured shadow frames at all 6x6 sampling phases (WSL Python)."""

import json
import os
from pathlib import Path
import subprocess


def main():
    root = Path(__file__).resolve().parents[1]
    output = root / 'docs/failure_diagnosis'
    work = root / 'build/half_light_cases'
    work.mkdir(parents=True, exist_ok=True)
    binary = root / 'docs/decode_stages/dm_stage_trace'
    width, height = 1280, 720
    rows = []
    for name in ('half_light01', 'half_light02', 'half_restored01', 'half_restored_fail01',
                 'half_restored_fail03'):
        source = (output / (name + '.gray8')).read_bytes()
        assert len(source) == width * height
        for dy in range(6):
            for dx in range(6):
                shifted = bytearray([128]) * len(source)
                for y in range(height - dy):
                    start = (y + dy) * width + dx
                    shifted[start:start + width - dx] = source[y * width:(y + 1) * width - dx]
                path = work / 'shift.gray8'
                path.write_bytes(shifted)
                result = subprocess.run(
                    [str(binary), str(path)], capture_output=True, text=True,
                    env={**os.environ, 'XCR_DM_PREPARED': '1', 'XCR_DM_TRACE_REPEATS': '4'})
                if result.returncode not in (0, 1):
                    raise RuntimeError(result.stderr)
                decoded = json.loads(result.stdout)
                rows.append(dict(source=name, dx=dx, dy=dy, result=decoded))
        print(name, sum(r['result']['geometry_quality'] == 2 for r in rows if r['source'] == name),
              '/36', flush=True)
    (output / 'half_light_final_regression.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
    failed = [r for r in rows if r['result']['decoded'] != 6 or
              r['result']['payload'] != '012345' or r['result']['geometry_quality'] != 2]
    if failed:
        raise SystemExit(f'{len(failed)}/{len(rows)} cases failed; see half_light_final_regression.json')
    print(f'Passed {len(rows)} cases; host timings are not ESP32 timings.')


if __name__ == '__main__':
    main()
