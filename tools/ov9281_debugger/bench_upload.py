"""Compare CDC result throughput with decoded-frame JPEG requests on and off."""

import argparse
import json
from pathlib import Path
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
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
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
                results, timings, frames, cpu, errors = [], [], [], [], []
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
                        elif kind == 'error':
                            errors.append(value)
                decodes = [item['decode_us'] / 1000 for item in results if item.get('decode_us') is not None]
                ages = [item['age_us'] / 1000 for item in results if item.get('age_us') is not None]
                sequences = [item['sequence'] for item in results if 'sequence' in item]
                capture_span = (results[-1]['capture_us'] - results[0]['capture_us']) if len(results) > 1 else 0
                sequence_span = sequences[-1] - sequences[0] if len(sequences) > 1 else 0
                entry = dict(stage=stage + 1, upload=upload, seconds=args.seconds,
                             results=len(results), result_per_second=round(len(results) / args.seconds, 2),
                             scan_messages=len(timings), scan_success=sum(item['success'] for item in timings),
                             images=len(frames), image_success=sum(item.success for item in frames),
                             cpu_samples=len(cpu),
                             median_core0_percent=round(statistics.median(item['core0'] for item in cpu), 1) if cpu else None,
                             median_core1_percent=round(statistics.median(item['core1'] for item in cpu), 1) if cpu else None,
                             duplicate_sequences=len(sequences) - len(set(sequences)),
                             missing_sequences=sequence_span + 1 - len(set(sequences)) if sequences else None,
                             dequeued_fps=round(sequence_span * 1000000 / capture_span, 3) if capture_span > 0 else None,
                             image_errors=len(errors),
                             max_age_ms=round(max(ages), 2) if ages else None,
                             median_decode_ms=round(statistics.median(decodes), 2) if decodes else None,
                             median_age_ms=round(statistics.median(ages), 2) if ages else None)
                report.append(entry)
                destination.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
                print(json.dumps(entry, ensure_ascii=False), flush=True)
        finally:
            try:
                port.write(b'@DEBUGOFF\r@TIMINGOFF\r@GEOMETRYOFF\r@SUBPIXELOFF\r')
            except serial.SerialException:
                pass


if __name__ == '__main__':
    main()
