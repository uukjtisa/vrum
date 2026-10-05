"""Print the energy share of each firing harmonic in a clip.

The summary stats in analyze_sound.py (centroid, rolloff) describe where energy
sits in general; this answers the specific question that matters for making an
engine scream: how much of the sound is the fundamental, and how much is the
harmonic series stacked above it. A lumped 0-D exhaust produces a near-pure
fundamental, so H3+ being near zero is the signature of that limitation rather
than of a tuning mistake.

    python scripts/harmonics.py --f0 700 clip.wav [clip2.wav ...]
"""

import argparse
import os
import wave

import numpy as np


def harmonics(path, f0, count):
    with wave.open(path, "rb") as w:
        sr = w.getframerate()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)

    x = x.astype(float)
    if len(x) < sr:
        return None

    # Drop the first half second: the leveler is still settling there.
    x = x[sr // 2:]
    spectrum = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    freqs = np.fft.rfftfreq(len(x), 1.0 / sr)
    total = spectrum.sum()
    if total <= 0:
        return None

    # Band each harmonic proportionally rather than by a fixed bin count. Even
    # on the dyno the engine speed wobbles cycle to cycle, which smears every
    # peak across several Hz -- a narrow window then reports a harmonic as
    # missing when it is merely spread out.
    shares = []
    for k in range(1, count + 1):
        centre = f0 * k
        half = max(2.0, 0.02 * centre)
        band = (freqs >= centre - half) & (freqs <= centre + half)
        shares.append(100.0 * spectrum[band].sum() / total)

    crest = 20 * np.log10(np.abs(x).max() / (x.std() + 1e-9) + 1e-9)
    return shares, np.abs(x).max(), crest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clips", nargs="+")
    ap.add_argument("--f0", type=float, required=True,
                    help="firing frequency in Hz (rpm/60 * cylinders/2 for a 4-stroke)")
    ap.add_argument("--count", type=int, default=8)
    args = ap.parse_args()

    header = f"{'clip':<22}{'peak':>7}{'crest':>7}  " + "".join(
        f"{'H' + str(k):>7}" for k in range(1, args.count + 1))
    print(header)
    print("-" * len(header))

    for path in args.clips:
        result = harmonics(path, args.f0, args.count)
        name = os.path.basename(path)
        if result is None:
            print(f"{name:<22}  (too short / silent)")
            continue
        shares, peak, crest = result
        print(f"{name:<22}{peak:>7.0f}{crest:>7.1f}  "
              + "".join(f"{s:>6.2f}%" for s in shares))

    print(f"\nshares are % of total energy in a band around each multiple of "
          f"{args.f0:.0f} Hz.")


if __name__ == "__main__":
    main()
