"""Record P4 timing/profile/CPU data, with a 3-second warm-up and USB reconnect retry."""
import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import serial

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/ov9281_debugger'))
from protocol import StreamParser

p = argparse.ArgumentParser()
p.add_argument('--output', required=True)
p.add_argument('--port', default='COM65')
p.add_argument('--seconds', type=float, default=15)
p.add_argument('--upload', action='store_true')
a = p.parse_args()
events, profiles, fronts = [], [], []
parser = StreamParser()
buffer = bytearray()
def open_port():
    deadline = time.monotonic() + 10
    while True:
        try:
            return serial.Serial(a.port, 921600, timeout=.03, write_timeout=1)
        except serial.SerialException:
            if time.monotonic() >= deadline:
                raise
            time.sleep(.25)

with open_port() as port:
    port.write(b'@DEBUGOFF\r@TIMINGON\r@PROFILEON\r@GEOMETRYON\r')
    warm_end = time.monotonic() + 3
    while time.monotonic() < warm_end:
        port.read(min(max(port.in_waiting, 1), 65536))
    start = time.monotonic()
    next_snapshot = start
    while time.monotonic() - start < a.seconds:
        if a.upload and time.monotonic() >= next_snapshot:
            port.write(b'@SNAP\r')
            next_snapshot = time.monotonic() + .2
        data = port.read(min(max(port.in_waiting, 1), 65536))
        stamp = time.monotonic() - start
        for kind, value in parser.feed(data):
            if kind == 'result' and value['payload'].startswith(b'@'):
                continue  # @PROFILE has the same field count as a basic result.
            if kind == 'frame':
                events.append(dict(type=kind, received_s=stamp, **value.metadata()))
            else:
                if isinstance(value, dict):
                    value = {k: v.decode(errors='replace') if isinstance(v, bytes) else v for k, v in value.items()}
                events.append(dict(type=kind, received_s=stamp, data=value))
        buffer.extend(data)
        while b'\r' in buffer:
            line, _, buffer = buffer.partition(b'\r')
            if line.startswith((b'@PROFILE,', b'@FRONT,')):
                fields = [int(x) for x in line.split(b',')[1:]]
                (profiles if line.startswith(b'@PROFILE,') else fronts).append(fields)
    elapsed = time.monotonic() - start
    port.write(b'@TIMINGOFF\r@PROFILEOFF\r@GEOMETRYOFF\r@DEBUGOFF\r')

def summary(values):
    if not values:
        return None
    v = sorted(values)
    return dict(n=len(v), min=min(v), mean=round(statistics.mean(v), 2), median=statistics.median(v),
                p95=v[min(len(v)-1, int(.95*len(v)))], max=max(v))

timings = [e['data'] for e in events if e['type'] == 'timing']
cpus = [e['data'] for e in events if e['type'] == 'cpu']
successes = [t for t in timings if t['success']]
result = dict(upload=a.upload, elapsed_s=elapsed, attempts=len(timings), successes=len(successes),
              success_pct=100*len(successes)/len(timings) if timings else None,
              attempts_per_s=len(timings)/elapsed, successes_per_s=len(successes)/elapsed,
              decode_us=summary([t['decode_us'] for t in timings]),
              success_decode_us=summary([t['decode_us'] for t in successes]),
              core0_pct=summary([c['core0'] for c in cpus]), core1_pct=summary([c['core1'] for c in cpus]),
              profiles=profiles, fronts=fronts, frames_received=sum(e['type']=='frame' for e in events),
              events=events)
result['result_packets'] = sum(e['type']=='result' for e in events)
result['result_packets_per_s'] = result['result_packets']/elapsed
if len(timings) > 1:
    result['timing_sequence_gaps'] = sum(max(0, b['sequence']-c['sequence']-1)
        for c,b in zip(timings,timings[1:]))
Path(a.output).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k: v for k, v in result.items() if k not in ('events', 'profiles', 'fronts')}, ensure_ascii=False, indent=2))
