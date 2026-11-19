"""Compare current L-finder recovery with HEAD on saved Gray8 failures (WSL)."""
import json
import os
from pathlib import Path
import subprocess

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT / 'build/dist_captures_20261119/regression'


def main():
    WORK.mkdir(parents=True, exist_ok=True)
    baseline = WORK / 'baseline.c'
    baseline.write_bytes(subprocess.check_output(
        ['git', 'show', 'HEAD:port/components/xcr_decoder/src/xcr_dmtx.c'], cwd=ROOT))
    common = ['gcc', '-O2', '-std=c11', '-fno-strict-aliasing',
              '-DCONFIG_XCR_DATAMATRIX_LIBDMTX=0', '-DXCR_DM_TRACE_STAGES=1',
              '-Iport/components/xcr_decoder/include',
              '-Iport/components/xcr_decoder/vendor/include',
              '-Iport/components/xcr_decoder/vendor/quirc']
    has_body = b'xcr_dm_body' in baseline.read_bytes()
    baseline_flags = ['-I' + str(WORK), '-DXCR_DMTX_BODY_RECOVERY=1',
                      '-Iport/components/xcr_decoder/src'] if has_body else []
    # Compile HEAD's C++ wrapper too: linking baseline C to the working tree
    # wrapper hides regressions in candidate routing and early rejection.
    if has_body:
        for name in ('xcr_dm_body.h', 'xcr_dm_body.cpp'):
            (WORK / name).write_bytes(subprocess.check_output(
                ['git', 'show', 'HEAD:port/components/xcr_decoder/src/' + name], cwd=ROOT))
        subprocess.run(['g++', '-O3', '-std=c++20', '-fsingle-precision-constant',
                        '-DZXING_INTERNAL', '-Iport/components/xcr_decoder/vendor/zxing_dm',
                        '-Iport/components/xcr_decoder/include',
                        '-Iport/components/xcr_decoder/vendor/include', '-c',
                        str(WORK / 'xcr_dm_body.cpp'), '-o', str(WORK / 'baseline_body.o')],
                       cwd=ROOT, check=True)
    subprocess.run(common + baseline_flags + ['-c', str(baseline), '-o', str(WORK / 'baseline.o')], cwd=ROOT, check=True)
    objects = [str(ROOT / 'build/dm_stage_trace' / f'trace_{n}.o') for n in ('main', 'port', 'core')]
    body_names = ('xcr_dm_body', 'BitMatrix', 'GridSampler', 'LocalGrid',
                  'PerspectiveTransform', 'WhiteRectDetector', 'DMDetector', 'DMVersion')
    body_objects = [str(WORK / 'baseline_body.o') if n == 'xcr_dm_body' else
                    str(ROOT / 'build/dm_stage_trace' / (n + '.o'))
                    for n in body_names] if has_body else []
    subprocess.run(['g++' if has_body else 'gcc', str(WORK / 'baseline.o'), *objects,
                    *body_objects, '-lm', '-o', str(WORK / 'baseline')], check=True)
    paths = sorted((ROOT / 'docs/failure_diagnosis').glob('*.gray8'))
    extra = sorted((ROOT / 'build/parallel_host_cases').glob('*.gray8'))
    cases = [(p, 'phase', dx, dy) for p in paths for dy in range(6) for dx in range(6)]
    for p in (paths[0], ROOT / 'docs/failure_diagnosis/stutter_final01.gray8'):
        cases += [(p, 'rotate', a, 1.) for a in (-15, -7, 7, 15)]
        cases += [(p, 'scale', 0, s) for s in (.65, .8, 1.2, 1.4)]
    cases += [(p, 'negative' if p.stem in ('blank', 'covered') else 'extra', 0, 0) for p in extra]
    env = {k: v for k, v in os.environ.items() if not k.startswith('XCR_DM_')}
    env['XCR_DM_TWO_STAGE'] = '1'
    source = WORK / 'case.gray8'
    cache, rows = {}, []
    for i, (path, kind, a, b) in enumerate(cases):
        if path not in cache:
            cache[path] = np.fromfile(path, np.uint8).reshape(720, 1280)
        image = cache[path]
        if kind == 'phase':
            image = np.full_like(image, 128)
            image[b:, a:] = cache[path][:720-b, :1280-a]
        elif kind in ('rotate', 'scale'):
            image = cv2.warpAffine(image, cv2.getRotationMatrix2D((640, 360), a, b),
                                  (1280, 720), borderValue=128)
        image.tofile(source)
        row = dict(name=path.stem, kind=kind, a=a, b=b)
        for name, binary in (('baseline', WORK / 'baseline'),
                             ('current', ROOT / 'docs/decode_stages/dm_stage_trace')):
            run = subprocess.run([str(binary), str(source)], capture_output=True,
                                 text=True, env=env, timeout=15)
            assert run.returncode in (0, 1), (row, name, run.stderr)
            row[name] = json.loads(run.stdout)
        rows.append(row)
        if (i + 1) % 100 == 0:
            print(i + 1, '/', len(cases), flush=True)
    good = lambda r: r['decoded'] > 0 and r['geometry_quality'] == 2
    lost = [r for r in rows if good(r['baseline']) and
            (not good(r['current']) or r['current']['payload'] != r['baseline']['payload'])]
    negatives = [r for r in rows if r['kind'] == 'negative' and r['current']['decoded'] > 0]
    report = dict(cases=len(rows), lost=lost, false_positive=negatives,
                  baseline_refined=sum(good(r['baseline']) for r in rows),
                  current_refined=sum(good(r['current']) for r in rows), rows=rows)
    (WORK / 'results.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: len(v) if isinstance(v, list) else v
                      for k, v in report.items() if k != 'rows'}, indent=2))
    return int(bool(lost or negatives))


if __name__ == '__main__':
    raise SystemExit(main())
