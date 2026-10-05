"""
VRUM sound analyzer
===================

Objective before/after measurement for the engine note, so tuning decisions are
made on spectra instead of on memory of what the last clip sounded like. This is
the same loop AngeTheGreat used against his calibrated POWERFIST recordings --
render, measure, compare, repeat.

Usage
-----
    python scripts/analyze_sound.py CLIP.wav [MORE.wav ...] [options]

    # typical: everything we captured, against the real thing
    python scripts/analyze_sound.py workspace/audio_capture/*.wav \
                                    assets/reference/*.wav --plot

Options
-------
    --plot            write a spectrum-comparison PNG (and per-clip spectrograms)
    --out DIR         where plots go (default: workspace/audio_capture/analysis)
    --trim SECONDS    skip this much from the head of each clip (default 0.5,
                      which drops the capture-start transient)
    --max SECONDS     analyse at most this many seconds (default 30)

What the numbers mean
---------------------
    centroid      Spectral centroid in Hz -- the "center of mass" of the
                  spectrum. THE headline brightness number. A muffled clip sits
                  low; a screaming straight-pipe sits high.
    >2k >4k >6k >8k
                  Share of total energy above that frequency, in percent. The
                  1900 Hz input filter this project just removed showed up as a
                  near-zero >4k column.
    rolloff85/95  Frequency below which 85% / 95% of the energy lives.
    crest         Peak-to-RMS in dB. Low crest = squashed/compressed, which is
                  what the auto-leveler does when it is working hard.

Comparing a simulated clip to a real recording is a comparison of *shapes*, not
absolute levels: real recordings carry mic, distance and room coloration, and
the sim output has been through an auto-leveler. Chase the trend of the columns,
not equality.
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys

import numpy as np
import soundfile as sf


# Bands reported as "share of energy above X".
HF_EDGES_HZ = (2000, 4000, 6000, 8000)

# Welch-style averaged periodogram settings.
FFT_SIZE = 4096
HOP = FFT_SIZE // 2


class Clip:
    """One analysed audio file."""

    def __init__(self, path: str, trim: float, max_seconds: float):
        self.path = path
        self.label = os.path.basename(path)

        data, rate = sf.read(path, always_2d=True, dtype="float64")
        self.rate = rate

        # Mono-sum: we only ever care about the exhaust note's spectrum, and the
        # simulator output is mono anyway.
        mono = data.mean(axis=1)

        start = int(trim * rate)
        if start >= len(mono):
            start = 0
        mono = mono[start:start + int(max_seconds * rate)]

        if len(mono) < FFT_SIZE:
            raise ValueError(
                f"{self.label}: only {len(mono)} samples after trimming -- "
                f"need at least {FFT_SIZE}. Capture a longer clip or lower --trim."
            )

        self.samples = mono
        self.duration = len(mono) / rate

        self._analyse()

    @classmethod
    def from_array(cls, label: str, samples: np.ndarray, rate: int) -> "Clip":
        """Build a Clip from an in-memory slice (used by --scan)."""
        self = cls.__new__(cls)
        self.path = label
        self.label = label
        self.rate = rate
        self.samples = samples
        self.duration = len(samples) / rate

        if len(samples) < FFT_SIZE:
            raise ValueError(f"{label}: too short to analyse")

        self._analyse()
        return self

    def _analyse(self):
        self.freqs, self.power = self._spectrum()
        self._metrics()

    def _spectrum(self):
        """Averaged power spectrum (Welch). Returns (freqs, power)."""
        window = np.hanning(FFT_SIZE)
        # Normalise so window gain doesn't bias absolute power between clips.
        window_power = (window ** 2).sum()

        frames = 1 + (len(self.samples) - FFT_SIZE) // HOP
        accum = np.zeros(FFT_SIZE // 2 + 1)
        for i in range(frames):
            chunk = self.samples[i * HOP:i * HOP + FFT_SIZE] * window
            spectrum = np.fft.rfft(chunk)
            accum += (np.abs(spectrum) ** 2)

        accum /= (frames * window_power)
        freqs = np.fft.rfftfreq(FFT_SIZE, 1.0 / self.rate)

        return freqs, accum

    def _metrics(self):
        total = self.power.sum()
        if total <= 0:
            raise ValueError(f"{self.label}: silent clip, nothing to measure.")

        self.centroid = float((self.freqs * self.power).sum() / total)

        self.hf_share = {}
        for edge in HF_EDGES_HZ:
            # Above the clip's own Nyquist there is nothing to measure; report
            # None so a 44.1k reference isn't unfairly compared to a narrow clip.
            if edge >= self.rate / 2:
                self.hf_share[edge] = None
            else:
                self.hf_share[edge] = float(
                    100.0 * self.power[self.freqs >= edge].sum() / total)

        cumulative = np.cumsum(self.power) / total
        self.rolloff85 = float(self.freqs[np.searchsorted(cumulative, 0.85)])
        self.rolloff95 = float(self.freqs[np.searchsorted(cumulative, 0.95)])

        rms = float(np.sqrt(np.mean(self.samples ** 2)))
        peak = float(np.max(np.abs(self.samples)))
        self.rms_db = 20 * math.log10(rms) if rms > 0 else -np.inf
        self.peak_db = 20 * math.log10(peak) if peak > 0 else -np.inf
        self.crest_db = self.peak_db - self.rms_db


def scan_file(path: str, window: float) -> list[Clip]:
    """Chop a long recording into fixed windows and analyse each one.

    Source clips pulled off the internet are whole videos -- engine, talking,
    music, silence. Scanning shows which windows are actually engine so a clean
    segment can be picked, instead of averaging the narration in with the revs.
    """
    data, rate = sf.read(path, always_2d=True, dtype="float64")
    mono = data.mean(axis=1)

    step = int(window * rate)
    clips: list[Clip] = []
    name = os.path.basename(path)

    for start in range(0, len(mono) - FFT_SIZE, step):
        chunk = mono[start:start + step]
        if len(chunk) < FFT_SIZE:
            break

        label = f"{name[:18]}.. @{start / rate:7.1f}s"
        try:
            clips.append(Clip.from_array(label, chunk, rate))
        except ValueError:
            continue

    return clips


def _fmt(value, width, digits=0, missing="  -- "):
    if value is None:
        return missing.rjust(width)
    return f"{value:>{width}.{digits}f}"


def print_table(clips: list[Clip]) -> None:
    name_width = max(len(c.label) for c in clips)
    name_width = max(name_width, 4)

    header = (
        f"{'clip':<{name_width}}  {'rate':>6}  {'secs':>5}  {'centroid':>8}  "
        + "  ".join(f"{'>' + str(e // 1000) + 'k':>6}" for e in HF_EDGES_HZ)
        + f"  {'roll85':>7}  {'roll95':>7}  {'crest':>6}"
    )
    print()
    print(header)
    print("-" * len(header))

    for c in clips:
        row = (
            f"{c.label:<{name_width}}  {c.rate:>6}  {c.duration:>5.1f}  "
            f"{c.centroid:>8.0f}  "
            + "  ".join(_fmt(c.hf_share[e], 6, 2) for e in HF_EDGES_HZ)
            + f"  {c.rolloff85:>7.0f}  {c.rolloff95:>7.0f}  {c.crest_db:>6.1f}"
        )
        print(row)

    print()
    print("centroid/roll85/roll95 in Hz; >Nk = percent of energy above N kHz; "
          "crest in dB.")

    if len(clips) >= 2:
        first, last = clips[0], clips[-1]
        delta = last.centroid - first.centroid
        print()
        print(f"centroid shift, first -> last: {first.centroid:.0f} Hz -> "
              f"{last.centroid:.0f} Hz  ({delta:+.0f} Hz)")


def write_plots(clips: list[Clip], out_dir: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(out_dir, exist_ok=True)

    # --- Overlaid spectra: the one picture that shows a low-pass ceiling. ---
    fig, ax = plt.subplots(figsize=(11, 6))
    for c in clips:
        # Normalise each curve to its own peak so shapes compare regardless of
        # level; we are chasing spectral shape, not loudness.
        db = 10 * np.log10(np.maximum(c.power, 1e-20))
        db -= db.max()
        ax.semilogx(c.freqs[1:], db[1:], linewidth=1.1, label=c.label)

    ax.set_xlim(50, max(c.rate for c in clips) / 2)
    ax.set_ylim(-90, 3)
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("relative level (dB)")
    ax.set_title("VRUM spectrum comparison (peak-normalised)")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)

    spectrum_path = os.path.join(out_dir, "spectrum_comparison.png")
    fig.tight_layout()
    fig.savefig(spectrum_path, dpi=130)
    plt.close(fig)
    print(f"wrote {spectrum_path}")

    # --- Per-clip spectrograms. ---
    for c in clips:
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.specgram(c.samples, NFFT=FFT_SIZE, Fs=c.rate,
                    noverlap=FFT_SIZE - HOP, cmap="magma")
        ax.set_ylim(0, c.rate / 2)
        ax.set_xlabel("time (s)")
        ax.set_ylabel("frequency (Hz)")
        ax.set_title(f"spectrogram -- {c.label}")

        safe = os.path.splitext(c.label)[0].replace(os.sep, "_")
        path = os.path.join(out_dir, f"spectrogram_{safe}.png")
        fig.tight_layout()
        fig.savefig(path, dpi=130)
        plt.close(fig)
        print(f"wrote {path}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure the spectral brightness of VRUM captures against "
                    "real engine recordings.")
    parser.add_argument("files", nargs="+",
                        help="WAV files (globs allowed)")
    parser.add_argument("--plot", action="store_true",
                        help="write spectrum + spectrogram PNGs")
    parser.add_argument("--out",
                        default=os.path.join("workspace", "audio_capture",
                                             "analysis"),
                        help="output directory for plots")
    parser.add_argument("--trim", type=float, default=0.5,
                        help="seconds to skip at the head of each clip")
    parser.add_argument("--max", type=float, default=30.0,
                        help="max seconds to analyse per clip")
    parser.add_argument("--scan", type=float, metavar="SECONDS",
                        help="split each input into windows of this length and "
                             "report each window separately -- use it to locate "
                             "the engine-only parts of a long recording")
    args = parser.parse_args()

    # Windows shells don't expand globs, so do it here.
    paths: list[str] = []
    for pattern in args.files:
        matched = sorted(glob.glob(pattern))
        if matched:
            paths.extend(matched)
        elif os.path.exists(pattern):
            paths.append(pattern)
        else:
            print(f"warning: no match for {pattern!r}", file=sys.stderr)

    if not paths:
        print("error: no input files found.", file=sys.stderr)
        return 1

    clips: list[Clip] = []
    for path in paths:
        try:
            if args.scan:
                clips.extend(scan_file(path, args.scan))
            else:
                clips.append(Clip(path, args.trim, args.max))
        except Exception as exc:  # noqa: BLE001 -- report and keep going
            print(f"skipping {path}: {exc}", file=sys.stderr)

    if not clips:
        print("error: nothing could be analysed.", file=sys.stderr)
        return 1

    print_table(clips)

    if args.plot:
        write_plots(clips, args.out)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
