#!/usr/bin/env python3
"""
Wandelt eine RDS-Aufnahme im Audiobereich (WAV/MP3/FLAC ... per ffmpeg lesbar)
in eine Sample-Datei um, die host_test wie ADC-Daten des ESP32 verarbeitet.

Hintergrund: Ein MP3 kann keinen 57-kHz-Träger enthalten (Nyquist 22 kHz). Solche
Aufnahmen enthalten das RDS-Signal daher bereits nach unten gemischt (BPSK um
einen Träger von wenigen kHz). Das Skript
  1. schätzt die Trägerfrequenz (Quadrieren des analytischen Signals → Linie bei 2·fc),
  2. mischt per Einseitenband-Verschiebung auf 57 kHz hoch,
  3. resampelt auf 228 kHz und schreibt uint16-LE (12-Bit-Werte, Mitte 2048).

  audio_to_adc.py <eingabe.mp3> <ausgabe.bin> [fc_hz]      (fc optional, sonst automatisch)
"""
import subprocess, sys
import numpy as np
from scipy.signal import hilbert, resample_poly
from numpy.fft import fft, fftfreq

src, dst = sys.argv[1], sys.argv[2]
fs_in = 44100
raw = subprocess.run(["ffmpeg", "-v", "error", "-i", src, "-f", "f32le", "-ac", "1",
                      "-ar", str(fs_in), "-"], capture_output=True, check=True).stdout
x = np.frombuffer(raw, dtype="<f4").astype(np.float64)
x -= x.mean()

if len(sys.argv) > 3:
    fc = float(sys.argv[3])
else:
    z2 = hilbert(x) ** 2
    F = np.abs(fft(z2 * np.hanning(len(z2)))); f = fftfreq(len(z2), 1 / fs_in)
    m = (f > 2000) & (f < 9000)                      # 2·fc für fc ≈ 1–4,5 kHz
    fc = f[m][np.argmax(F[m])] / 2
print(f"Träger ≈ {fc:.1f} Hz")

y = resample_poly(x, 760, 147)                       # 44,1 kHz → 228 kHz
fs = 228000.0
t = np.arange(len(y)) / fs
up = np.real(hilbert(y) * np.exp(2j * np.pi * (57000.0 - fc) * t))
up *= 400 / np.max(np.abs(up))
np.clip(np.round(2048 + up), 0, 4095).astype("<u2").tofile(dst)
print(f"{len(y)/fs:.1f} s → {dst}")
