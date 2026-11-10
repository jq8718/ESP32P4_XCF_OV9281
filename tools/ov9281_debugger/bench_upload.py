"""Compare CDC result throughput with decoded-frame JPEG requests on and off."""

import argparse
import json
import statistics
import time

import serial

from protocol import StreamParser


def main():
    args_parser = argparse.ArgumentParser()
    args_parser.add_argument('--port', required=True)
    args_parser.add_argument('--seconds', type=float, default=12)
    args_parser.add_argument('--output', default='docs/upload_benchmark.json')
    args = args_parser.parse_args()
    report = []
    parser = StreamParser()
    with serial.Serial(args.port, 921600, timeout=0.03, write_timeout=0.3) as port:
        port.reset_input_buffer()
        port.write(b'@TIMINGON\r@GEOMETRYON\r@SUBPIXELON\r')
        try:
            for stage, upload in enumerate((True, False, True, False)):
                if not upload:
                    port.write(b'@DEBUGOFF\r@TIMINGON\r')
                requested = False
                last_request = 0.0
                started = time.monotonic()
                warmup = started + 2
                end = warmup + args.seconds
                results, timings, frames, cpu = [], [], [], []
                while time.monotonic() < end:
                    now = time.monotonic()
                    if upload and not requested and now - last_request >= 0.35:
                        port.write(b'@SNAP\r')
                        requested = True
                        last_request = now
                    if requested and now - last_request > 2.5:
                        requested = False
                        parser.pending = None
                        parser.image.clear()
                    data = port.read(min(max(port.in_waiting, 1), 65536))
                    for kind, value in parser.feed(data):
                        if kind in ('frame', 'waiting', 'error'):
                            requested = False
                        if now < warmup:
                            continue
                        if kind == 'result':
                            results.append(value)
                        elif kind == 'timing':
                            timings.append(value)
                        elif kind == 'frame':
                            frames.append(value)
                        elif kind == 'cpu':
                            cpu.append(value)
                decodes = [item['decode_us'] / 1000 for item in results if item.get('decode_us') is not None]
                ages = [item['age_us'] / 1000 for item in results if item.get('age_us') is not None]
                entry = dict(stage=stage + 1, upload=upload, seconds=args.seconds,
                             results=len(results), result_per_second=round(len(results) / args.seconds, 2),
                             scan_messages=len(timings), scan_success=sum(item['success'] for item in timings),
                             images=len(frames), image_success=sum(item.success for item in frames),
                             cpu_samples=len(cpu),
                             median_core0_percent=round(statistics.median(item['core0'] for item in cpu), 1) if cpu else None,
                             median_core1_percent=round(statistics.median(item['core1'] for item in cpu), 1) if cpu else None,
                             median_decode_ms=round(statistics.median(decodes), 2) if decodes else None,
                             median_age_ms=round(statistics.median(ages), 2) if ages else None)
                report.append(entry)
                print(json.dumps(entry, ensure_ascii=False), flush=True)
        finally:
            port.write(b'@DEBUGOFF\r@TIMINGOFF\r@GEOMETRYOFF\r@SUBPIXELOFF\r')
    with open(args.output, 'w', encoding='utf-8') as output:
        json.dump(report, output, ensure_ascii=False, indent=2)


if __name__ == '__main__':
    main()
