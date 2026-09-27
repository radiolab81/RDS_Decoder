#!/usr/bin/env python3
"""
Erzeugt ein synthetisches UKW-MPX-Signal mit RDS und schreibt es als rohe
uint16-LE-Samples (12-Bit-ADC-Werte) – so, wie der ESP32-ADC sie liefern würde.

  gen_mpx.py <out.bin> [ppm_fehler=0] [sekunden=30] [rds_lsb=40] [noise_lsb=3]

Enthalten: Pilot 19 kHz, L+R und L-R (DSB 38 kHz) als bandbegrenztes Rauschen,
RDS (Gruppen 0A, 2A, 3A, 11A=RT+, 4A) und ADC-Rauschen. Die Abtastrate wird um
"ppm_fehler" von 228 kHz verstimmt, um eine ungenaue ADC-Clock zu simulieren.
"""
import sys
import numpy as np

out   = sys.argv[1]
ppm   = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
secs  = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
rds_a = float(sys.argv[4]) if len(sys.argv) > 4 else 40.0     # RDS-Spitze in LSB
noise = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
fs    = 228000.0 * (1.0 + ppm * 1e-6)
rng   = np.random.default_rng(1)

# ---------------- RDS-Bitstrom ----------------
def checkword(info, off):
    reg = info << 10
    for i in range(25, 9, -1):
        if (reg >> i) & 1:
            reg ^= 0x5B9 << (i - 10)
    return (reg & 0x3FF) ^ off

OFF = dict(A=0x0FC, B=0x198, C=0x168, CP=0x350, D=0x1B4)
def group_bits(a, b, c, d, cp=False):
    bits = []
    for info, off in ((a, 'A'), (b, 'B'), (c, 'CP' if cp else 'C'), (d, 'D')):
        w = (info << 10) | checkword(info, OFF[off])
        bits += [(w >> (25 - k)) & 1 for k in range(26)]
    return bits

PI, PTY = 0xD31C, 10
PS = b"TESTFM  "
RT = b"Test Artist - Test Song Title\r".ljust(32, b' ')
def hdr(t, v): return (t << 12) | (v << 11) | (1 << 10) | (PTY << 5)

def g0a(seg):
    return group_bits(PI, hdr(0, 0) | (1 << 3) | seg, 0xE105, (PS[2*seg] << 8) | PS[2*seg+1])
def g2a(seg):
    c = RT[4*seg:4*seg+4]
    return group_bits(PI, hdr(2, 0) | seg, (c[0] << 8) | c[1], (c[2] << 8) | c[3])
def g3a():   # ODA-Zuordnung: RT+ (AID 4BD7) in Gruppe 11A
    return group_bits(PI, hdr(3, 0) | (11 << 1), 0x0000, 0x4BD7)
def g11a():  # RT+: Interpret (Typ 4) Start 0 Länge 11 / Titel (Typ 1) Start 14 Länge 15
    t1, s1, l1, t2, s2, l2 = 4, 0, 10, 1, 14, 14
    b = hdr(11, 0) | (1 << 4) | (1 << 3) | (t1 >> 3)
    c = ((t1 & 7) << 13) | (s1 << 7) | (l1 << 1) | (t2 >> 5)
    d = ((t2 & 31) << 11) | (s2 << 5) | l2
    return group_bits(PI, b, c, d)
def g4a(mjd, h, m, off_half):
    b = hdr(4, 0) | ((mjd >> 15) & 3)
    c = ((mjd & 0x7FFF) << 1) | (h >> 4)
    d = ((h & 15) << 12) | (m << 6) | (off_half & 0x1F)
    return group_bits(PI, b, c, d)

MJD = 61307      # 2026-09-24
seq = []
for cyc in range(4):
    for s in range(4):
        seq.append(g0a(s)); seq.append(g2a(2*s)); seq.append(g0a(s)); seq.append(g2a(2*s+1))
    seq.append(g3a()); seq.append(g11a()); seq.append(g4a(MJD, 12, 34, 4))
bits = []
for g in seq: bits += g
bits = np.array(bits, dtype=np.uint8)

# Differenzkodierung  d[n] = a[n] xor d[n-1]
d = np.zeros(len(bits), dtype=np.uint8); prev = 0
for i, a in enumerate(bits):
    prev ^= a; d[i] = prev

# ---------------- Basisband: Manchester + Bandbegrenzung ----------------
N = int(fs * secs)
t = np.arange(N) / fs
bp = t * 1187.5
k = np.floor(bp).astype(int) % len(d)
frac = bp - np.floor(bp)
sym = np.where(d[k] == 1, 1.0, -1.0) * np.where(frac < 0.5, 1.0, -1.0)
S = np.fft.rfft(sym); f = np.fft.rfftfreq(N, 1/fs)
H = np.clip((3600 - f) / 1600, 0, 1); H = 0.5 - 0.5*np.cos(np.pi*H)   # sanfter Rolloff 2..3,6 kHz
sym = np.fft.irfft(S * H, N)
sym /= np.max(np.abs(sym))

# ---------------- MPX ----------------
def band_noise(fmax):
    w = rng.standard_normal(N); W = np.fft.rfft(w)
    W[f > fmax] = 0; W[f < 30] = 0
    x = np.fft.irfft(W, N); return x / np.std(x)

lpr = 0.13 * band_noise(15000)            # L+R
lmr = 0.13 * band_noise(15000)            # L-R
pilot = 0.09 * np.cos(2*np.pi*19000*t + 0.7)
stereo = lmr * np.cos(2*np.pi*38000*t + 1.4)
carrier = np.cos(2*np.pi*57000*t + 2.1)
rds = (rds_a / 1500.0) * sym * carrier

mpx = lpr + pilot + stereo + rds
adc = 2048 + 1500.0 * mpx + noise * rng.standard_normal(N)
adc = np.clip(np.round(adc), 0, 4095).astype('<u2')
adc.tofile(out)
print(f"{N} Samples, fs={fs:.1f} Hz, RDS-Spitze={rds_a} LSB, MPX-Spitze={np.max(np.abs(adc.astype(int)-2048))} LSB")
