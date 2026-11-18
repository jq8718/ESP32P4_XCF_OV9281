"""Integer UART v6 telemetry, CRC-16/CCITT-FALSE and calibration commands."""
import binascii
import math
import re

POSE_NAMES = ('yaw_y_deg', 'pitch_x_deg', 'roll_z_deg', 'range_m',
              'optical_depth_m', 'distance_to_plane_m', 'tx_m', 'along_track_m')
COMPACT_POSE_NAMES = ('yaw_y_deg', 'distance_to_plane_m', 'along_track_m')
SEQUENCE_MAX = 999
CAL_NAMES = ('width', 'height', 'fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2', 'k3')
FACTORY_CAL = dict(zip(CAL_NAMES, (1280, 720, 733.242515, 732.511533,
    589.273600, 359.376026, .119448, -.223125, .000163, -.000596, .083362)))


def integer(text, low=-2147483648, high=2147483647):
    if not re.fullmatch(r'-?[0-9]+', text):
        raise ValueError('整数格式错误')
    value = int(text)
    if not low <= value <= high:
        raise ValueError('整数超出范围')
    return value


def frame(body):
    raw = body.encode('ascii')
    return raw + f'*{binascii.crc_hqx(raw, 0xffff):04X}\r\n'.encode('ascii')


def parse_scan(line):
    raw = line.rstrip(b'\r\n')
    try:
        body, checksum = raw.rsplit(b'*', 1)
    except ValueError:
        raise ValueError('缺少 CRC16 校验') from None
    if not re.fullmatch(b'[0-9A-Fa-f]{4}', checksum):
        raise ValueError('CRC16 格式错误')
    if binascii.crc_hqx(body, 0xffff) != int(checksum, 16):
        raise ValueError('CRC16 校验失败')
    f = body.decode('ascii').split(',')
    if len(f) < 6 or f[:2] != ['@SCAN', '6'] or f[2] not in ('C', 'F'):
        raise ValueError('不支持的 UART 协议版本或模式')
    compact = f[2] == 'C'
    if len(f) != (9 if compact else 22):
        raise ValueError('UART 字段数量错误')
    sequence = integer(f[3], 0, SEQUENCE_MAX)
    valid = bool(integer(f[4], 0, 1))
    if not re.fullmatch(r'[0-9]{1,255}', f[5]):
        raise ValueError('解码内容必须为 1～255 位数字')
    coordinates = [] if compact else [integer(v, 0, 12800 if i % 2 == 0 else 7200)/10
                                      for i, v in enumerate(f[6:14])]
    values = {key: None for key in POSE_NAMES}
    names = COMPACT_POSE_NAMES if compact else POSE_NAMES
    for key, text in zip(names, f[6:] if compact else f[14:]):
        number = integer(text)
        values[key] = number/1000 if valid else None
    return dict(payload=f[5], sequence=sequence,
                corners=[coordinates[i:i+2] for i in range(0, len(coordinates), 2)],
                pose_valid=valid, compact=compact, **values)


def validate_calibration(values):
    c = {name: float(values[name]) for name in CAL_NAMES}
    if any(not math.isfinite(v) for v in c.values()):
        raise ValueError('内参必须是有限数值')
    if c['width'] != 1280 or c['height'] != 720:
        raise ValueError('标定分辨率必须为 1280×720')
    if not all(100 <= c[name] <= 10000 for name in ('fx', 'fy')):
        raise ValueError('fx/fy 必须在 100～10000 之间')
    if not (0 <= c['cx'] < 1280 and 0 <= c['cy'] < 720):
        raise ValueError('主点必须在图像范围内')
    if any(abs(c[name]) > 10 for name in CAL_NAMES[6:]):
        raise ValueError('畸变参数绝对值不能大于 10')
    c['width'], c['height'] = 1280, 720
    return c


def calibration_command(values):
    c = validate_calibration(values)
    return '@CAL,SET,' + ','.join(format(c[name], '.9g') for name in CAL_NAMES)


def parse_reply(line):
    f = line.decode('ascii').split(',')
    if f[:2] == ['@CAL', 'DATA'] and len(f) == 13:
        return dict(kind='calibration', values=validate_calibration(dict(zip(CAL_NAMES, f[2:]))))
    if f[:2] == ['@OUT', 'DATA'] and len(f) == 3 and f[2] in ('FULL', 'COMPACT'):
        return dict(kind='output', mode=f[2])
    if len(f) >= 3 and f[0] in ('@CAL', '@OUT') and f[1] == 'ERR':
        return dict(kind='error', command=f[0], reason=','.join(f[2:]))
    raise ValueError('未知命令应答')
