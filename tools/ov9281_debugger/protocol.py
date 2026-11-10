import base64
import binascii
import csv
import io
import math
import time
import zlib
from collections import deque
from dataclasses import dataclass


@dataclass
class Snapshot:
    sequence: int
    timestamp_us: int
    width: int
    height: int
    success: bool
    decode_us: int | None
    corners: list
    payload: bytes
    jpeg: bytes = b''
    coordinates_only: bool = False
    geometry_quality: int | None = None
    geometry_edge_mask: int = 0
    geometry_support: int = 0
    geometry_residual: float = 0.0

    @property
    def text(self):
        return self.payload.decode('utf-8', errors='replace')

    def metadata(self):
        return {key: value for key, value in vars(self).items() if key not in ('jpeg', 'payload')} | {
            'payload': self.text, 'payload_hex': self.payload.hex(), 'image_encoding': None if self.coordinates_only else 'JPEG',
        }


def parse_coordinate(value):
    coordinate = float(value) if '.' in value else int(value)
    if not math.isfinite(coordinate) or not -2147483648 <= coordinate <= 2147483647:
        raise ValueError('Coordinate overflow')
    return coordinate


def parse_result(line):
    fields = next(csv.reader(io.StringIO(line.decode('latin1')), strict=True))
    if len(fields) not in (9, 12, 13, 17):
        raise ValueError('Not a QR result')
    coordinates = [parse_coordinate(value) for value in fields[1:9]]
    if any(value < -2147483648 or value > 2147483647 for value in coordinates):
        raise ValueError('Coordinate overflow')
    encoded = fields[0].encode('latin1')
    payload = bytearray()
    index = 0
    while index < len(encoded):
        if encoded[index] != 92:
            payload.append(encoded[index])
            index += 1
        elif encoded[index:index + 2] == b'\\\\':
            payload.append(92)
            index += 2
        elif encoded[index:index + 2] == b'\\x' and index + 4 <= len(encoded):
            payload.append(int(encoded[index + 2:index + 4], 16))
            index += 4
        else:
            raise ValueError('Invalid escape')
    result = {'payload': bytes(payload), 'corners': [coordinates[index:index + 2] for index in range(0, 8, 2)]}
    if len(fields) >= 12:
        sequence, capture_us, age_us = map(int, fields[9:12])
        if not 0 <= sequence <= 0xffffffff or not 0 < capture_us <= 0xffffffffffffffff or not 0 <= age_us <= 0xffffffffffffffff:
            raise ValueError('Invalid frame timing')
        result.update(sequence=sequence, capture_us=capture_us, age_us=age_us)
    if len(fields) >= 13:
        decode_us = int(fields[12])
        if not 0 <= decode_us <= 0xffffffff:
            raise ValueError('Invalid decode duration')
        result['decode_us'] = decode_us
    if len(fields) == 17:
        result.update(parse_geometry(fields[13:]))
    return result


def parse_geometry(fields):
    quality, mask, support, residual = map(int, fields)
    if quality not in (0, 1, 2) or not 0 <= mask <= 15 or not 0 <= support <= 128 or not 0 <= residual <= 1500:
        raise ValueError('Invalid geometry quality')
    if quality == 2 and (mask != 15 or support < 28):
        raise ValueError('Incomplete refined geometry')
    return dict(geometry_quality=quality, geometry_edge_mask=mask,
                geometry_support=support, geometry_residual=residual / 1000)


class StreamParser:
    def __init__(self):
        self.buffer = bytearray()
        self.pending = None
        self.image = bytearray()
        self.length = 0
        self.crc = 0
        self.discard_line = False

    def feed(self, data):
        self.buffer.extend(data)
        events = []
        while b'\r' in self.buffer:
            end = self.buffer.index(13)
            line = bytes(self.buffer[:end])
            del self.buffer[:end + 1]
            if self.discard_line:
                self.discard_line = False
                continue
            if not line or len(line) > 2048:
                continue
            try:
                events.append(('result', parse_result(line)))
                continue
            except (ValueError, csv.Error, StopIteration):
                pass
            try:
                if line.startswith(b'@TIMING,'):
                    fields = line.split(b',')
                    if len(fields) != 5:
                        raise ValueError('Invalid timing packet')
                    sequence, capture_us, decode_us, success = map(int, fields[1:])
                    if not (0 <= sequence <= 0xffffffff and 0 < capture_us <= 0xffffffffffffffff and 0 <= decode_us <= 0xffffffff and success in (0, 1)):
                        raise ValueError('Invalid timing values')
                    events.append(('timing', dict(sequence=sequence, capture_us=capture_us, decode_us=decode_us, success=bool(success))))
                elif line.startswith(b'@CPU,'):
                    fields = line.split(b',')
                    if len(fields) != 4:
                        raise ValueError('Invalid CPU packet')
                    interval_ms, core0_permille, core1_permille = map(int, fields[1:])
                    if not (1 <= interval_ms <= 60000 and 0 <= core0_permille <= 1000 and 0 <= core1_permille <= 1000):
                        raise ValueError('Invalid CPU utilization')
                    events.append(('cpu', dict(interval_ms=interval_ms, core0=core0_permille / 10,
                                               core1=core1_permille / 10)))
                elif line.startswith(b'@FRAME,'):
                    fields = line.decode('ascii').split(',')
                    if not ((len(fields) == 19 and fields[1] == '1') or (len(fields) == 23 and fields[1] == '2')):
                        raise ValueError('Unsupported snapshot header')
                    numbers = [int(value) for value in fields[2:8]]
                    numbers += [parse_coordinate(value) for value in fields[8:16]]
                    payload = bytes.fromhex(fields[16])
                    self.length = int(fields[17])
                    self.crc = int(fields[18], 16)
                    if not 1 <= self.length <= 1843200 or len(payload) > 255:
                        raise ValueError('Snapshot size out of range')
                    if numbers[2:4] not in ([1280, 720], [1024, 1024], [512, 512]) or numbers[4] not in (0, 1):
                        raise ValueError('Invalid image metadata')
                    self.pending = Snapshot(numbers[0], numbers[1], numbers[2], numbers[3], bool(numbers[4]),
                                            numbers[5], [numbers[index:index + 2] for index in range(6, 14, 2)], payload)
                    if fields[1] == '2':
                        for name, value in parse_geometry(fields[19:]).items():
                            setattr(self.pending, name, value)
                    self.image.clear()
                elif line.startswith(b'@DATA,') and self.pending:
                    self.image.extend(base64.b64decode(line[6:], validate=True))
                    if len(self.image) > self.length:
                        raise ValueError('Snapshot exceeded declared length')
                elif line.startswith(b'@END,') and self.pending:
                    if int(line.split(b',')[1]) != self.pending.sequence or len(self.image) != self.length:
                        raise ValueError('Incomplete snapshot')
                    if zlib.crc32(self.image) != self.crc:
                        raise ValueError('Snapshot CRC mismatch')
                    self.pending.jpeg = bytes(self.image)
                    events.append(('frame', self.pending))
                    self.pending = None
                    self.image.clear()
                elif line.startswith(b'@WAIT,'):
                    events.append(('waiting', line[6:].decode('ascii', errors='replace')))
            except (ValueError, binascii.Error, IndexError, UnicodeError) as error:
                self.pending = None
                self.image.clear()
                events.append(('error', str(error)))
        if len(self.buffer) > 4096:
            self.buffer.clear()
            self.discard_line = True
            self.pending = None
            self.image.clear()
            events.append(('error', 'Oversized serial line discarded'))
        return events


class ScanRate:
    def __init__(self, now=None):
        self.reset(now)

    def reset(self, now=None):
        self.started = time.monotonic() if now is None else now
        self.events = deque()
        self.recent = deque()
        self.total = 0

    def record(self, now=None):
        now = time.monotonic() if now is None else now
        if self.recent and now >= self.recent[-1] + 3:
            self.recent.clear()
        while self.recent and self.recent[0] < now - 5:
            self.recent.popleft()
        self.recent.append(now)
        self.events.append(now)
        self.total += 1

    def current(self, now=None):
        now = time.monotonic() if now is None else now
        if not self.recent or now >= self.recent[-1] + 3:
            return None
        samples = [event for event in self.recent if event >= now - 5]
        if len(samples) < 2 or samples[-1] <= samples[0]:
            return None
        return (len(samples) - 1) * 60 / (samples[-1] - samples[0])

    def values(self, now=None):
        now = time.monotonic() if now is None else now
        while self.events and self.events[0] <= now - 60:
            self.events.popleft()
        elapsed = max(0, now - self.started)
        return len(self.events), self.total * 60 / elapsed if elapsed > 0 else 0, self.total, elapsed
