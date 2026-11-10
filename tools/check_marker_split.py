"""Compare full-frame and left/right marker searches on shifted Gray8 samples."""

import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'docs/decode_stages/source.gray8'
CASES = ROOT / 'build/parallel_host_cases'
WIDTH, HEIGHT = 1280, 720


def decode(binary: str, sample: Path) -> dict:
    result = subprocess.run(
        ['wsl', '-e', f'docs/decode_stages/{binary}', str(sample.relative_to(ROOT)).replace('\\', '/')],
        cwd=ROOT, text=True, capture_output=True, check=False,
    )
    if result.returncode not in (0, 1) or not result.stdout.strip():
        raise RuntimeError(f'{binary} failed on {sample.name}: {result.stderr}')
    return json.loads(result.stdout)


def main() -> None:
    source = SOURCE.read_bytes()
    if len(source) != WIDTH * HEIGHT:
        raise ValueError('Expected a 1280x720 Gray8 sample')
    CASES.mkdir(parents=True, exist_ok=True)
    report = []
    for shift in (0, 100, 150, 200, 250, 300, 350, 400, 450, 500, 600):
        sample = CASES / f'shift_{shift:03d}.gray8'
        if shift:
            padding = bytes([128]) * shift
            sample.write_bytes(b''.join(padding + source[row * WIDTH:(row + 1) * WIDTH - shift]
                                        for row in range(HEIGHT)))
        else:
            sample.write_bytes(source)
        full = decode('dm_stage_trace', sample)
        split = decode('dm_stage_trace_split', sample)
        match = (full['decoded'] == split['decoded'] and
                 full['payload'] == split['payload'] and
                 full['marker_roi'] == split['marker_roi'] and
                 full['corners'] == split['corners'])
        report.append({'shift_x': shift, 'match': match, 'full': full, 'split': split})
        print(f'shift={shift:3d} match={match} full_roi={full["marker_roi"]} split_roi={split["marker_roi"]}')
    for name, pixels in (
        ('blank', bytes([128]) * (WIDTH * HEIGHT)),
        ('covered', b''.join(bytes([128]) * WIDTH if 120 <= row < 410 else
                            source[row * WIDTH:(row + 1) * WIDTH] for row in range(HEIGHT))),
    ):
        sample = CASES / f'{name}.gray8'
        sample.write_bytes(pixels)
        full = decode('dm_stage_trace', sample)
        split = decode('dm_stage_trace_split', sample)
        match = full['decoded'] == split['decoded'] == 0
        report.append({'case': name, 'match': match, 'full': full, 'split': split})
        print(f'{name}: match={match} full={full["decoded"]} split={split["decoded"]}')
    # Enlarge the marked symbol beyond the centre overlap. Neither half can
    # contain all three markers, so this case exercises the full-frame fallback.
    crop_x, crop_y, crop_w, crop_h, scale = 234, 156, 228, 216, 3
    enlarged = bytearray(bytes([128]) * (WIDTH * HEIGHT))
    out_x = (WIDTH - crop_w * scale) // 2
    out_y = (HEIGHT - crop_h * scale) // 2
    for y in range(crop_h):
        row = source[(crop_y + y) * WIDTH + crop_x:(crop_y + y) * WIDTH + crop_x + crop_w]
        expanded = bytes(value for value in row for _ in range(scale))
        for repeat in range(scale):
            offset = (out_y + y * scale + repeat) * WIDTH + out_x
            enlarged[offset:offset + len(expanded)] = expanded
    sample = CASES / 'large_centre.gray8'
    sample.write_bytes(enlarged)
    full = decode('dm_stage_trace', sample)
    split = decode('dm_stage_trace_split', sample)
    match = (full['decoded'] > 0 and full['decoded'] == split['decoded'] and
             full['payload'] == split['payload'] and full['marker_roi'] == split['marker_roi'] and
             split['full_fallback'] == 1)
    report.append({'case': 'large_centre', 'match': match, 'full': full, 'split': split})
    print(f'large_centre: match={match} full_roi={full["marker_roi"]} split_roi={split["marker_roi"]}')
    output = ROOT / 'docs/decode_stages/marker_split_host_results.json'
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    if not all(item['match'] for item in report):
        raise SystemExit('Marker split differs from full-frame search')


if __name__ == '__main__':
    main()
