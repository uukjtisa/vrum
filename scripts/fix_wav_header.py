"""
Repair WAV captures whose header sizes were never patched.

A capture that was still running when the app exited abruptly (window X button,
task kill) keeps its placeholder sizes of 0, so every audio tool reports it as
empty even though the samples are on disk. This rewrites the RIFF and data
chunk sizes from the actual file length.

    python scripts/fix_wav_header.py workspace/audio_capture/*.wav

Files whose headers are already correct are left alone.
"""

from __future__ import annotations

import glob
import os
import struct
import sys

HEADER_BYTES = 44
RIFF_SIZE_OFFSET = 4
DATA_SIZE_OFFSET = 40


def repair(path: str) -> str:
    size = os.path.getsize(path)
    if size < HEADER_BYTES:
        return "too small to be a WAV"

    with open(path, "r+b") as f:
        header = f.read(HEADER_BYTES)
        if header[0:4] != b"RIFF" or header[8:12] != b"WAVE":
            return "not a RIFF/WAVE file"
        if header[36:40] != b"data":
            return "unexpected chunk layout (not a plain 44-byte header)"

        riff_size = struct.unpack("<I", header[RIFF_SIZE_OFFSET:RIFF_SIZE_OFFSET + 4])[0]
        data_size = struct.unpack("<I", header[DATA_SIZE_OFFSET:DATA_SIZE_OFFSET + 4])[0]

        true_data = size - HEADER_BYTES
        true_riff = size - 8

        if riff_size == true_riff and data_size == true_data:
            return "already correct"

        f.seek(RIFF_SIZE_OFFSET)
        f.write(struct.pack("<I", true_riff))
        f.seek(DATA_SIZE_OFFSET)
        f.write(struct.pack("<I", true_data))

    channels = struct.unpack("<H", header[22:24])[0]
    rate = struct.unpack("<I", header[24:28])[0]
    bits = struct.unpack("<H", header[34:36])[0]
    frames = true_data // max(1, channels * bits // 8)

    return f"repaired -> {frames / rate:.1f}s ({frames} frames @ {rate} Hz)"


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    paths: list[str] = []
    for pattern in sys.argv[1:]:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else ([pattern] if os.path.exists(pattern) else []))

    if not paths:
        print("error: no input files found.", file=sys.stderr)
        return 1

    for path in paths:
        print(f"{os.path.basename(path):36s} {repair(path)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
