"""Pack 1280x720 JPEG/Gray8 images for the test-only P4 replay application."""
import argparse
import glob
import hashlib
import io
import json
from pathlib import Path
import re
import struct

from PIL import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path, help="Files, or directories (nonrecursive).")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    paths = []
    for source in args.inputs:
        if glob.has_magic(str(source)):
            paths.extend(map(Path, sorted(glob.glob(str(source)))))
            continue
        paths.extend(sorted(p for p in source.iterdir() if p.suffix.lower() in
                            (".jpg", ".jpeg", ".gray8")) if source.is_dir() else [source])
    if not paths:
        parser.error("No input images found")
    entries, manifest = [], []
    for index, path in enumerate(paths):
        original = path.read_bytes()
        image = (Image.frombytes("L", (1280, 720), original) if path.suffix == ".gray8"
                 else Image.open(io.BytesIO(original)))
        if image.size != (1280, 720):
            raise ValueError(f"{path}: expected 1280x720")
        if path.suffix.lower() in (".jpg", ".jpeg") and image.mode == "L":
            encoded = original
        else:
            output = io.BytesIO()
            image.convert("L").save(output, "JPEG", quality=95, subsampling=0)
            encoded = output.getvalue()
        if len(encoded) > 256 * 1024:
            raise ValueError(f"{path}: exceeds the replay JPEG buffer")
        match = re.search(r"frame_(\d+)", path.stem)
        sequence = int(match.group(1)) if match else index
        entries.append(struct.pack("<II", sequence, len(encoded)) + encoded)
        manifest.append(dict(id=sequence, file=str(path), jpeg_bytes=len(encoded),
                             source_sha256=hashlib.sha256(original).hexdigest(),
                             replay_jpeg_sha256=hashlib.sha256(encoded).hexdigest()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(b"P4RP" + struct.pack("<I", len(entries)) + b"".join(entries))
    args.output.with_suffix(".manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"{len(entries)} images, {args.output.stat().st_size} bytes: {args.output}")


if __name__ == "__main__":
    main()
