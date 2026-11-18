"""Cross-check the STM32 C parser and Python parser, without a serial port."""
import ctypes
import json
from pathlib import Path
import random
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/stm32_uart_debugger'))
from uart_protocol import frame, parse_scan, calibration_command, parse_reply, FACTORY_CAL


class Packet(ctypes.Structure):
    _fields_ = [('sequence', ctypes.c_uint32), ('compact', ctypes.c_uint8),
        ('pose_valid', ctypes.c_uint8), ('payload_length', ctypes.c_uint16),
        ('payload', ctypes.c_uint8*255), ('corners_x10', ctypes.c_int32*8)] + [
        (n, ctypes.c_int32) for n in ('yaw_mdeg', 'pitch_mdeg', 'roll_mdeg', 'range_mm',
                                     'depth_mm', 'plane_mm', 'tx_mm', 'along_mm')]


def main():
    out = ROOT/'build/uart_integer_validation'
    out.mkdir(parents=True, exist_ok=True)
    vswhere = Path('C:/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe')
    vs = subprocess.check_output([str(vswhere), '-latest', '-property', 'installationPath'], text=True).strip()
    source = ROOT/'examples/stm32_uart/p4_scan_protocol.c'
    numeric = ROOT/'port/standalone_idf_test/main/scan_uart_numeric.h'
    wrapper = out/'numeric.c'
    wrapper.write_text(f'#include "{numeric.as_posix()}"\n'
        'int checked_scale(float value, float scale, int32_t *out) { return scan_uart_scaled_i32(value, scale, out); }\n', encoding='utf-8')
    batch = out/'compile.cmd'
    batch.write_text(f'@echo off\ncall "{vs}/VC/Auxiliary/Build/vcvars64.bat" >nul\n'
        f'cl /nologo /std:c11 /O2 /W4 /WX /LD "{source}" "{wrapper}" /link '
        f'/EXPORT:p4_scan_parse_line /EXPORT:p4_scan_crc16 /EXPORT:checked_scale /OUT:"{out}/parser.dll"\n', encoding='utf-8')
    result = subprocess.run(['cmd', '/c', str(batch)], cwd=out, text=True, errors='replace', capture_output=True)
    if result.returncode:
        raise RuntimeError(result.stdout+result.stderr)
    library = ctypes.CDLL(str(out/'parser.dll'))
    library.p4_scan_parse_line.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(Packet)]
    library.p4_scan_parse_line.restype = ctypes.c_int
    library.p4_scan_crc16.argtypes = [ctypes.c_char_p, ctypes.c_size_t]
    library.p4_scan_crc16.restype = ctypes.c_uint16
    assert library.p4_scan_crc16(b'123456789', 9) == 0x29b1

    library.checked_scale.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_int32)]
    library.checked_scale.restype = ctypes.c_int
    numeric_cases = [(0, 1000, 0), (.0125, 1000, 13), (-.0125, 1000, -13),
        (2147483520., 1, 2147483520), (-2147483648., 1, -2147483648),
        (2147483648., 1, None), (-2147483904., 1, None),
        (2147483.75, 1000, None), (-2147484., 1000, None),
        (float('nan'), 1000, None), (float('inf'), 1000, None),
        (-float('inf'), 1000, None), (3e38, 1000, None)]
    for value, scale, expected in numeric_cases:
        output = ctypes.c_int32(1234)
        ok = library.checked_scale(value, scale, ctypes.byref(output))
        assert bool(ok) == (expected is not None), (value, scale, ok)
        assert output.value == (1234 if expected is None else expected)

    rng = random.Random(20261118)
    count = 0
    for i in range(1000):
        compact = bool(i%2)
        payload = ('012345' if i == 0 else '0' if i == 1 else
                   ''.join(str(rng.randrange(10)) for _ in range(255 if i%13 == 0 else rng.randint(1, 30))))
        sequence = rng.randrange(1000)
        valid = int(i%11 != 0)
        corners = [rng.randrange(12801 if j%2 == 0 else 7201) for j in range(8)]
        numbers = [rng.randint(-180000, 180000) for _ in range(3)] + [
            rng.randint(1, 3000) for _ in range(3)] + [rng.randint(-2000, 2000) for _ in range(2)]
        values = [numbers[0], numbers[5], numbers[7]] if compact else corners+numbers
        body = ','.join(map(str, ['@SCAN', 6, 'C' if compact else 'F', sequence, valid, payload, *values]))
        raw = frame(body)
        c = Packet()
        assert library.p4_scan_parse_line(raw, len(raw), ctypes.byref(c)) == 1
        p = parse_scan(raw)
        assert c.sequence == p['sequence'] == sequence
        assert bool(c.compact) == p['compact'] == compact
        assert bool(c.pose_valid) == p['pose_valid'] == bool(valid)
        assert c.payload_length == len(payload)
        assert bytes(c.payload[:c.payload_length]).decode('ascii') == p['payload'] == payload
        if valid:
            assert c.yaw_mdeg == round(p['yaw_y_deg']*1000) == numbers[0]
            if compact:
                assert p['optical_depth_m'] is None and c.depth_mm == 0
            else:
                assert c.depth_mm == round(p['optical_depth_m']*1000) == numbers[4]
            assert c.plane_mm == round(p['distance_to_plane_m']*1000) == numbers[5]
            assert c.along_mm == round(p['along_track_m']*1000) == numbers[7]
        damaged = bytearray(raw)
        damaged[4] ^= 1
        ctypes.memset(ctypes.byref(c), 0x5a, ctypes.sizeof(c))
        before = bytes(c)
        assert library.p4_scan_parse_line(bytes(damaged), len(damaged), ctypes.byref(c)) != 1
        assert bytes(c) == before
        try:
            parse_scan(bytes(damaged))
        except ValueError:
            pass
        else:
            raise AssertionError('corrupted packet accepted')
        count += 1
    # Correct field count and CRC ensure each malformed number is actually checked.
    compact_fields = ['@SCAN', '6', 'C', '999', '1', '012345', '-123', '490', '20']
    full_fields = ['@SCAN', '6', 'F', '1', '1', '012345'] + ['10']*8 + ['0']*8
    bad_bodies = ['@SCAN,4,C,1,1,303132333435,1,2,3',
                  '@SCAN,5,C,1,1,012345,1,2,3', '@SCAN,6,C,1,1,012345,1,2',
                  '@SCAN,6,C,1,1,012345,1,2,3,4', '@SCAN,6,X,1,1,012345,1,2,3']
    for base in (compact_fields, full_fields):
        packet = Packet()
        for seq in (998, 999, 0, 1):
            sample = base.copy()
            sample[3] = str(seq)
            raw = frame(','.join(sample))
            assert library.p4_scan_parse_line(raw, len(raw), ctypes.byref(packet)) == 1
            assert packet.sequence == parse_scan(raw)['sequence'] == seq
        for index, text in ((3,'1000'), (3,'4294967296'), (3,'-1'), (4,'2'),
                            *((5, payload) for payload in ('', '1,2', '1\r2', '1\n2', '1\x002',
                                                           '1A2', '12 3', '+12', '1'*256))):
            bad = list(base); bad[index] = text; bad_bodies.append(','.join(bad))
        for index in range(6 if base[2] == 'C' else 14, len(base)):
            for text in ('2147483648','-2147483649','999999999999999999999999999999', '1.2', 'nan', ''):
                bad = list(base); bad[index] = text; bad_bodies.append(','.join(bad))
        for index in range(6 if base[2] == 'C' else 14, len(base)):
            for text in ('2147483647','-2147483648'):
                edge = list(base); edge[index] = text
                raw = frame(','.join(edge))
                assert library.p4_scan_parse_line(raw,len(raw),ctypes.byref(Packet())) == 1
                parse_scan(raw)
    for bad in bad_bodies:
        raw = frame(bad)
        c = Packet()
        ctypes.memset(ctypes.byref(c),0x5a,ctypes.sizeof(c))
        before = bytes(c)
        assert library.p4_scan_parse_line(raw, len(raw), ctypes.byref(c)) == -1, bad
        assert bytes(c) == before
        try:
            parse_scan(raw)
        except ValueError:
            continue
        raise AssertionError(bad)
    command = calibration_command(FACTORY_CAL)
    assert parse_reply(command.replace('@CAL,SET', '@CAL,DATA').encode())['values'] == FACTORY_CAL
    for name, value in (('width', 640), ('fx', float('nan')), ('k1', 11), ('cy', 720)):
        try:
            calibration_command(dict(FACTORY_CAL, **{name:value}))
        except ValueError:
            continue
        raise AssertionError('invalid calibration accepted')
    report = dict(status='passed', cross_language_packets=count, crc_damage_cases=count,
                  checks=['C parser W4/WX build', 'CRC known vector', 'leading zero/1-to-255-digit payload',
                          'compact/full/invalid pose', 'frame wrap 999 to 0 in both modes', 'all int32 field boundaries', 'bad fields', 'sender NaN/Inf/overflow/rounding',
                          'output unchanged on rejection', 'calibration validation'])
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
