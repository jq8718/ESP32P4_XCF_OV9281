"""Compare dense marker windows on saved frames and translated sampling phases.

Run in WSL after tools/build_dm_stage_trace.sh. The baseline binary must be
built from the pre-change decoder at build/captures_20261118/baseline.
JPEG reconstructions and translations are regression inputs, not live trials.
"""
import json
import os
from pathlib import Path
import subprocess


def main():
    root = Path(__file__).resolve().parents[1]
    work = root / 'build/captures_20261118'
    captures = sorted(work.glob('20261118*.gray8'))
    existing = root / 'docs/failure_diagnosis'
    cases = [(p, 0, 0, False) for p in sorted(existing.glob('*.gray8'))]
    for source in captures:
        cases.extend((source, dx, dy, True) for dy in range(4) for dx in range(4))
    for name in ('fail01', 'current06', 'half_light01', 'half_light02',
                 'half_restored_fail01', 'light_on01', 'slow_current01'):
        cases.extend((existing / (name + '.gray8'), dx, dy, False)
                     for dy in range(6) for dx in range(6))
    negatives = list((root / 'build/parallel_host_cases').glob('*.gray8'))
    cases.extend((p, 0, 0, False) for p in negatives)
    baseline = work / 'baseline'
    current = root / 'docs/decode_stages/dm_stage_trace'
    results = []
    shifted = work / 'regression_shift.gray8'
    cached = {}
    for index, (source, dx, dy, require) in enumerate(cases):
        if str(source) not in cached:
            cached[str(source)] = source.read_bytes()
        data = cached[str(source)]
        assert len(data) == 1280 * 720
        if dx or dy:
            pixels = bytearray([128]) * len(data)
            for y in range(720 - dy):
                start = (y+dy)*1280+dx
                pixels[start:start+1280-dx] = data[y*1280:(y+1)*1280-dx]
            shifted.write_bytes(pixels)
            path = shifted
        else:
            path = source
        pair = []
        for binary in (baseline, current):
            run = subprocess.run([str(binary), str(path)], capture_output=True, text=True,
                                 env={**os.environ, 'XCR_DM_TWO_STAGE': '1'})
            if run.returncode not in (0, 1):
                raise RuntimeError(run.stderr)
            pair.append(json.loads(run.stdout))
        before, after = pair
        lost = before['decoded'] > 0 and (after['decoded'] <= 0 or
            after['payload'] != before['payload'] or after['geometry_quality'] < before['geometry_quality'])
        failed_new = require and (after['payload'] != '012348' or after['geometry_quality'] != 2)
        false_positive = source.stem in ('blank', 'covered') and after['decoded'] > 0
        results.append(dict(source=source.stem, dx=dx, dy=dy, before=before, after=after,
                            lost=lost, failed_new=failed_new, false_positive=false_positive))
        if (index+1) % 50 == 0:
            print(f'{index+1}/{len(cases)}', flush=True)
    output = root / 'docs/failure_diagnosis/captures_20261118/window_regression.json'
    output.write_text(json.dumps(results, indent=2), encoding='utf-8')
    failures = [r for r in results if r['lost'] or r['failed_new'] or r['false_positive']]
    print(json.dumps(dict(cases=len(results), lost=sum(r['lost'] for r in results),
                          new_failed=sum(r['failed_new'] for r in results),
                          false_positive=sum(r['false_positive'] for r in results)), indent=2))
    if failures:
        print([(r['source'], r['dx'], r['dy'], r['lost'], r['failed_new']) for r in failures])
        raise SystemExit(1)


if __name__ == '__main__':
    main()
