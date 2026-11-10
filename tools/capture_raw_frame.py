"""Fetch one exact OV9281 Gray8 frame through the firmware's @RAW interface."""

import argparse
import base64
import json
import pathlib
import time
import zlib

import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', default='docs/decode_stages/source')
    args = parser.parse_args()
    destination = pathlib.Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with serial.Serial(args.port, 921600, timeout=0.1, write_timeout=0.5) as connection:
        connection.reset_input_buffer()
        connection.write(b'@RAW\r')
        buffer = bytearray()
        pixels = bytearray()
        metadata = None
        end = time.monotonic() + 12
        while time.monotonic() < end:
            buffer.extend(connection.read(min(max(connection.in_waiting, 1), 65536)))
            while b'\r' in buffer:
                position = buffer.index(13)
                line = bytes(buffer[:position])
                del buffer[:position + 1]
                if line.startswith(b'@RAWFRAME,'):
                    fields = line.decode('ascii').split(',')
                    if len(fields) != 9:
                        raise ValueError(f'Invalid RAW header: {line[:100]!r}')
                    _, sequence, timestamp, width, height, success, decode_us, length, crc = fields
                    metadata = dict(sequence=int(sequence), timestamp_us=int(timestamp),
                                    width=int(width), height=int(height), success=bool(int(success)),
                                    decode_us=int(decode_us), length=int(length), crc32=crc)
                    pixels.clear()
                elif line.startswith(b'@RAWDATA,') and metadata:
                    pixels.extend(base64.b64decode(line[9:], validate=True))
                    if len(pixels) > metadata['length']:
                        raise ValueError('RAW data exceeded advertised length')
                elif line.startswith(b'@RAWEND,') and metadata:
                    if int(line[8:]) != metadata['sequence'] or len(pixels) != metadata['length']:
                        raise ValueError('RAW frame incomplete')
                    if metadata['width'] * metadata['height'] != len(pixels):
                        raise ValueError('Unexpected RAW frame dimensions')
                    if zlib.crc32(pixels) != int(metadata['crc32'], 16):
                        raise ValueError('RAW CRC mismatch')
                    destination.with_suffix('.gray8').write_bytes(pixels)
                    destination.with_suffix('.pgm').write_bytes(
                        f"P5\n{metadata['width']} {metadata['height']}\n255\n".encode('ascii') + pixels)
                    destination.with_suffix('.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
                    print(json.dumps(metadata), flush=True)
                    return
        raise TimeoutError(f'No complete RAW frame ({len(pixels)} bytes received)')


if __name__ == '__main__':
    main()
