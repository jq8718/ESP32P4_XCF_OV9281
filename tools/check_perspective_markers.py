"""Replay current perspective failures with all 6x6 grid phases (WSL Python)."""

import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', help='Optional pre-fix dm_stage_trace executable')
    parser.add_argument('--include-stutter', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = root / 'docs/failure_diagnosis'
    work = root / 'build/perspective_marker_cases'
    work.mkdir(parents=True, exist_ok=True)
    binaries = {'after': root / 'docs/decode_stages/dm_stage_trace'}
    if args.baseline:
        binaries['before'] = Path(args.baseline).resolve()
    rows = []
    sources = ['current_1111_fail01', 'current_1111_fail02', 'current_1111_restored01']
    if args.include_stutter:
        sources += ['stutter_restored01', 'stutter_restored_fail01']
    for name in sources:
        source = (output / (name + '.gray8')).read_bytes()
        assert len(source) == 1280 * 720
        for dy in range(6):
            for dx in range(6):
                pixels = bytearray([128]) * len(source)
                for y in range(720 - dy):
                    pixels[(y + dy) * 1280 + dx:(y + dy + 1) * 1280] = source[y * 1280:(y + 1) * 1280 - dx]
                path = work / 'shift.gray8'
                path.write_bytes(pixels)
                row = dict(source=name, dx=dx, dy=dy)
                for label, binary in binaries.items():
                    result = subprocess.run(
                        [str(binary), str(path)], capture_output=True, text=True,
                        env={**os.environ, 'XCR_DM_PREPARED': '1', 'XCR_DM_TRACE_REPEATS': '4'})
                    if result.returncode not in (0, 1):
                        raise RuntimeError(result.stderr)
                    row[label] = json.loads(result.stdout)
                rows.append(row)
        subset = [r for r in rows if r['source'] == name]
        print(name, {label: sum(r[label]['geometry_quality'] == 2 for r in subset)
                     for label in binaries}, '/36', flush=True)
    (output / ('stutter_perspective_regression.json' if args.include_stutter else 'current_1111_regression.json')).write_text(json.dumps(rows, indent=2), encoding='utf-8')
    for row in rows:
        actual = row['after']
        assert actual['payload'] == '012345' and actual['geometry_quality'] == 2, row
        reference = next(r['after']['corners'] for r in rows if r['source'] == row['source'])
        assert all(abs(value - reference[i] - (row['dx'] if i % 2 == 0 else row['dy'])) <= 2
                   for i, value in enumerate(actual['corners'])), row
    print(f'Passed {len(rows)} refined decodes and corner translation checks.')


if __name__ == '__main__':
    main()
