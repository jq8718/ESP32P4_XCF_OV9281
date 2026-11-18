"""Offline marker-threshold comparison; never edits or flashes product firmware.

Windows Python needs OpenCV/Numpy/Pillow; WSL needs gcc and Python 3.
Only marker masks are substituted in a temporary copy of the real C decoder.
All subsequent candidate, geometry, ECC and corner refinement code is unchanged.
"""
import hashlib
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT / 'build/binarization_comparison_20261118'
OUTPUT = ROOT / 'docs/failure_diagnosis/binarization_20261118'
CAPTURES = Path(r'C:\Users\jq163\Downloads\captures')
METHODS = ('current', 'mean13', 'mean5', 'gaussian13', 'median13',
           'sauvola13', 'sauvola5', 'bernsen9', 'otsu')


def masks(gray):
    g = gray.astype(np.float64)
    ones = np.ones(g.shape, dtype=np.float64)

    def mean(n, data=g):
        # Truncated windows match the firmware's border handling.
        return (cv2.boxFilter(data, -1, (n, n), normalize=False, borderType=cv2.BORDER_CONSTANT) /
                cv2.boxFilter(ones, -1, (n, n), normalize=False, borderType=cv2.BORDER_CONSTANT))

    result = {'mean13': g < mean(13)*.75, 'mean5': g < mean(5)*.75,
              'gaussian13': g < cv2.GaussianBlur(g, (13, 13), 2)*.75,
              'median13': g < cv2.medianBlur(gray, 13)*.75}
    for n, k in ((13, .5), (5, .34)):
        m = mean(n)
        std = np.sqrt(np.maximum(mean(n, g*g)-m*m, 0))
        result[f'sauvola{n}'] = g < m*(1+k*(std/128-1))
    kernel = np.ones((9, 9), np.uint8)
    lo = cv2.erode(gray, kernel).astype(np.float64)
    hi = cv2.dilate(gray, kernel).astype(np.float64)
    result['bernsen9'] = (hi-lo >= 24) & (g < (lo+hi)/2)
    threshold, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    result['otsu'] = g <= threshold
    return {key: value.astype(np.uint8) for key, value in result.items()}


def linux(path):
    path = Path(path).resolve()
    return '/mnt/'+path.drive[0].lower()+path.as_posix()[2:]


def build_probe():
    source = ROOT / 'port/components/xcr_decoder/src/xcr_dmtx.c'
    text = source.read_text(encoding='utf-8')
    needle = '    trace_marker_pgm("03_marker_binary.pgm", mask, width, height, 1);'
    assert text.count(needle) == 1
    hook = '''
    /* Offline experiment only: substitute a precomputed marker grid. */
    const char *mask_directory = getenv("XCR_BIN_MASK_DIR");
    if (mask_directory && *mask_directory) {
        char mask_path[1024];
        if (sample_left != 0 || width != (image->width + step - 1) / step) abort();
        snprintf(mask_path, sizeof(mask_path), "%s/mask_%d.gray8", mask_directory, step);
        FILE *file = fopen(mask_path, "rb");
        if (!file || fread(mask, 1, count, file) != count) abort();
        fclose(file);
    }
'''
    (WORK / 'probe_marker.c').write_text(text.replace(needle, hook+needle), encoding='utf-8')
    build = '''#!/bin/bash
set -euo pipefail
cd "'''+linux(ROOT)+'''"
work=build/binarization_comparison_20261118
common=(-O2 -std=c11 -fno-strict-aliasing -DCONFIG_XCR_DATAMATRIX_LIBDMTX=0 -DXCR_DM_TRACE_STAGES=1 -Iport/components/xcr_decoder/include -Iport/components/xcr_decoder/vendor/include -Iport/components/xcr_decoder/vendor/quirc)
gcc "${common[@]}" -c tools/dm_stage_trace.c -o "$work/main.o"
gcc "${common[@]}" -c "$work/probe_marker.c" -o "$work/marker.o"
gcc "${common[@]}" -c port/components/xcr_decoder/src/xcr_port.c -o "$work/port.o"
gcc "${common[@]}" -DXCR_DATAMATRIX_P4=1 -DXCR_PORT_REMAP_ALLOC=1 -c port/components/xcr_decoder/vendor/symbologies/datamatrix.c -o "$work/core.o"
gcc "$work/main.o" "$work/marker.o" "$work/port.o" "$work/core.o" -lm -o "$work/probe"
'''
    (WORK / 'build.sh').write_text(build, encoding='utf-8', newline='\n')
    subprocess.run(['wsl', '-e', 'bash', linux(WORK/'build.sh')], check=True)
    return hashlib.sha256(source.read_bytes()).hexdigest()


def main():
    WORK.mkdir(parents=True, exist_ok=True)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    source_hash = build_probe()
    originals = sorted(CAPTURES.glob('*.jpg'))
    assert len(originals) == 5
    cases = []
    for i, path in enumerate(originals):
        image = cv2.imread(str(path), 0)
        assert image.shape == (720, 1280)
        for dy in range(4):
            for dx in range(4):
                shifted = np.full_like(image, 128)
                shifted[dy:, dx:] = image[:720-dy, :1280-dx]
                cases.append((f'capture_{i}_{dx}_{dy}', shifted, 'capture', i, dx, dy))
    historical = sorted((ROOT/'docs/failure_diagnosis').glob('*.gray8'))
    for path in historical:
        cases.append((path.stem, np.fromfile(path, np.uint8).reshape(720, 1280), 'historical', None, 0, 0))
    for name in ('blank', 'covered'):
        path = ROOT/'build/parallel_host_cases'/f'{name}.gray8'
        if path.exists():
            cases.append((name, np.fromfile(path, np.uint8).reshape(720, 1280), 'negative', None, 0, 0))
    manifest = []
    for name, image, group, index, dx, dy in cases:
        directory = WORK/'cases'/name
        directory.mkdir(parents=True, exist_ok=True)
        image.tofile(directory/'source.gray8')
        for step in (4, 6):
            for method, mask in masks(image[::step, ::step]).items():
                target = directory/method
                target.mkdir(exist_ok=True)
                mask.tofile(target/f'mask_{step}.gray8')
        manifest.append(dict(name=name, directory=linux(directory), group=group, index=index, dx=dx, dy=dy))
    (WORK/'manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
    runner = '''import json, os, subprocess
from pathlib import Path
work = Path(__file__).resolve().parent
manifest = json.loads((work/'manifest.json').read_text())
methods = '''+repr(METHODS)+'''
rows = []
for i, case in enumerate(manifest):
    for method in methods:
        env = dict(os.environ, XCR_DM_TWO_STAGE='1')
        env.pop('XCR_DM_TRACE_DIR', None)
        if method != 'current':
            env['XCR_BIN_MASK_DIR'] = case['directory']+'/'+method
        else:
            env.pop('XCR_BIN_MASK_DIR', None)
        run = subprocess.run([str(work/'probe'), case['directory']+'/source.gray8'],
                             env=env, capture_output=True, text=True, timeout=10)
        if run.returncode not in (0, 1):
            raise RuntimeError((case['name'], method, run.stderr))
        rows.append(dict(case, method=method, result=json.loads(run.stdout)))
    if (i+1)%10 == 0: print(f'{i+1}/{len(manifest)} cases', flush=True)
(work/'results.json').write_text(json.dumps(rows, indent=2))
print('Finished', len(rows), 'full decoder replays', flush=True)
'''
    (WORK/'run.py').write_text(runner, encoding='utf-8')
    subprocess.run(['wsl', '-e', 'python3', linux(WORK/'run.py')], check=True)
    rows = json.loads((WORK/'results.json').read_text())
    baseline = {r['name']: r['result'] for r in rows if r['method'] == 'current'}
    summary = []
    for method in METHODS:
        subset = [r for r in rows if r['method'] == method]
        captures = [r for r in subset if r['group'] == 'capture']
        good = lambda r: r['decoded'] > 0 and r['geometry_quality'] == 2
        originals_rows = [r for r in captures if r['dx'] == r['dy'] == 0]
        lost = [r['name'] for r in subset if r['group'] == 'historical' and good(baseline[r['name']]) and
                (not good(r['result']) or r['result']['payload'] != baseline[r['name']]['payload'])]
        summary.append(dict(method=method, originals_ok=sum(good(r['result']) and r['result']['payload']=='012348' for r in originals_rows),
            originals_total=len(originals_rows), phases_ok=sum(good(r['result']) and r['result']['payload']=='012348' for r in captures),
            phases_total=len(captures), historical_lost=lost,
            historical_recovered=[r['name'] for r in subset if r['group']=='historical' and
                                  not good(baseline[r['name']]) and good(r['result'])],
            negative_false_positive=[r['name'] for r in subset if r['group']=='negative' and r['result']['decoded']>0]))
    report = dict(input='5 saved JPEG images; not CSI RAW8', source_sha256=source_hash,
                  note='Only the marker-search binary mask is replaced; actual C candidate search, decoding and refinement are unchanged. Host timing includes file I/O and is not a P4 timing benchmark.',
                  methods=summary, historical_count=len(historical), rows=rows)
    (OUTPUT/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    draw_comparison(cv2.imread(str(originals[0]), 0), summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def draw_comparison(image, summary):
    names = dict(mean13='旧方法：13×13 均值 × 0.75', mean5='小窗口：5×5 均值 × 0.75',
                 gaussian13='高斯加权：13×13，σ=2', median13='中值背景：13×13 × 0.75',
                 sauvola13='Sauvola：13×13，k=0.50', sauvola5='Sauvola：5×5，k=0.34',
                 bernsen9='局部极值：9×9，反差门限24', otsu='整幅网格 Otsu')
    fontpath = r'C:\Windows\Fonts\msyh.ttc'
    title_font = ImageFont.truetype(fontpath, 27)
    font = ImageFont.truetype(fontpath, 19)
    canvas = Image.new('RGB', (1500, 1690), '#eaf0f6')
    draw = ImageDraw.Draw(canvas)
    draw.text((20, 12), '同一帧定位标记二值化对比（4 像素网格；完整 C 解码验证）', font=title_font, fill='#18334c')
    choices = [('source', image)] + [(m, (1-mask)*255) for m, mask in masks(image[::4, ::4]).items()]
    for i, (method, pixels) in enumerate(choices):
        x, y = (i%3)*500, 60+(i//3)*540
        draw.rectangle((x+8, y, x+490, y+526), fill='white')
        if method == 'source':
            crop = pixels[240:392, 512:656]
            title = '原始灰度图（标签局部）'
            caption = '码体约64×65 px；外部标记约12～14 px'
        else:
            crop = pixels[60:98, 128:164]
            title = names[method]
            item = next(r for r in summary if r['method']==method)
            caption = f"原图 {item['originals_ok']}/5；平移 {item['phases_ok']}/80；历史退化 {len(item['historical_lost'])}"
        draw.text((x+18, y+12), title, font=font, fill='#18334c')
        enlarged = Image.fromarray(crop).resize((432,456), Image.Resampling.NEAREST).convert('RGB')
        canvas.paste(enlarged, (x+28,y+46))
        draw.text((x+18,y+504), caption, font=font, fill='#18334c')
    canvas.save(OUTPUT/'comparison.png')


if __name__ == '__main__':
    main()
