"""Check the direct ECC200 path against independently encoded ZXing grids.

Run on Windows after building the WSL host tools; requires numpy/zxing-cpp.
"""
import json
from pathlib import Path
import subprocess

import numpy as np
import zxingcpp


def main():
    root = Path(__file__).resolve().parents[1]
    work = root / 'build/grid10_cases'
    work.mkdir(parents=True, exist_ok=True)
    rows = []
    for text in ('012345', '000000', '999999', '123456', 'ABC', 'AGV', 'Z9'):
        image = np.asarray(zxingcpp.write_barcode(zxingcpp.BarcodeFormat.DataMatrix, text,
                                                 quiet_zone=0))
        assert image.shape == (10, 10), (text, image.shape)
        bits = (image < 128).astype('uint8')
        for turn in range(4):
            rotated = np.rot90(bits, turn)
            if rotated[0].sum() == 10 and rotated[:, 0].sum() == 10:
                bits = rotated.copy()
                break
        else:
            raise AssertionError('No solid corner')
        for mirror in (False, True):
            normal = bits.T.copy() if mirror else bits.copy()
            for corruption in ('none', 'one_bit', 'two_bits', 'bad_border', 'non_binary'):
                modules = normal.copy()
                if corruption in ('one_bit', 'two_bits'):
                    modules[3, 3] ^= 1
                if corruption == 'two_bits':
                    modules[6, 6] ^= 1
                if corruption == 'bad_border':
                    modules[0, 1:4] = 0
                if corruption == 'non_binary':
                    modules[4, 4] = 2
                path = work / 'modules.bin'
                modules.tofile(path)
                result = subprocess.run(
                    ['wsl', '-e', 'build/dm_grid10_probe', path.relative_to(root).as_posix()],
                    cwd=root, capture_output=True, text=True)
                expected = text if corruption not in ('bad_border', 'non_binary') else None
                actual = bytes.fromhex(result.stdout.strip()).decode('ascii') if result.returncode == 0 else None
                assert result.returncode in (0, 1) and actual == expected, (text, mirror, corruption, result)
                rows.append(dict(text=text, mirror=mirror, corruption=corruption, decoded=actual))
    for value in (0, 1):
        path = work / 'modules.bin'
        path.write_bytes(bytes([value]) * 100)
        result = subprocess.run(['wsl', '-e', 'build/dm_grid10_probe', path.relative_to(root).as_posix()], cwd=root)
        assert result.returncode == 1
        rows.append(dict(uniform=value, decoded=None))
    (root / 'docs/failure_diagnosis/grid10_regression.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
    print(f'Passed {len(rows)} independently encoded, corrected, mirrored and negative grid cases.')


if __name__ == '__main__':
    main()
