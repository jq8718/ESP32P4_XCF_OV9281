"""Summarize board-side frame and DataMatrix stage times from ESP logs."""

import json
import re
import statistics
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / "docs" / "decode_stages"


def fields(line):
    return {key: int(value) for key, value in re.findall(r"\b([a-z_][a-z_0-9]*)=(\d+)\b", line)}


def summarize(values):
    values = sorted(values)
    if not values:
        return None
    return {
        "n": len(values),
        "min": values[0],
        "median": round(statistics.median(values)),
        "mean": round(statistics.mean(values)),
        "p90": values[min(len(values) - 1, int(0.9 * len(values)))],
        "p95": values[min(len(values) - 1, int(0.95 * len(values)))],
        "max": values[-1],
    }


def main():
    lines = (DIRECTORY / "board_timing.log").read_text(encoding="utf-8", errors="replace").splitlines()
    marker = [fields(line) for line in lines if "p4scan_agv: dm_marker " in line]
    core = [fields(line) for line in lines if "p4scan_agv: datamatrix " in line]
    frame = [fields(line) for line in lines if "p4scan_agv: seq=" in line]
    geometry = [fields(line) for line in lines if "p4scan_agv: geometry seq=" in line]
    result = {"sample_counts": {"marker": len(marker), "core": len(core), "frame": len(frame), "geometry": len(geometry)}}
    for name, records, keys in (
        ("marker", marker, ("detect", "normalize", "core", "geometry", "total", "passes", "found")),
        ("core", core, ("finder", "candidate", "codeword", "ecc", "total", "finders")),
        ("frame", frame, ("prepare", "acquire", "queue_age", "decode", "age", "total")),
        ("geometry", geometry, ("core1", "wait", "core0", "dropped")),
    ):
        result[name] = {key: summarize([record[key] for record in records if key in record]) for key in keys}
    result["frame_status"] = {status: sum(f"status={status}" in line for line in lines) for status in ("OK", "TIMEOUT_NO_RESULT")}
    (DIRECTORY / "board_timing_summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
