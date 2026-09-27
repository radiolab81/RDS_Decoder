# Software-Defined RDS Demodulator/Decoder

*A portable, bare-metal C implementation of the full RDS PHY, link and
application layer (57 kHz BPSK subcarrier → PS / RadioText / RT+ / clock
time) — no dedicated RDS IC required. Reference implementation for the
ESP32, easily portable to almost any MCU with an ADC and enough RAM.*

## Why this is different from the usual "RDS Decoder" projects

Most microcontroller RDS projects you'll find online are **not** RDS
receivers. They are a thin driver for a dedicated RDS/RBDS demodulator IC
(e.g. the classic SAA6588 or similar), where the actual carrier recovery,
bit synchronization and block decoding happen inside that chip's fixed-
function silicon. The microcontroller just polls the IC over I²C and
reformats what it hands back.

This project has **no such IC**. The ESP32 samples the raw analog MPX
composite signal directly on an ADC pin, and every single step of classic
RDS demodulation is implemented in portable, dependency-free C:

- down-conversion of the 57 kHz RDS subcarrier,
- decimation and channel filtering,
- a full **Costas loop** for carrier/phase recovery of the suppressed-carrier
  BPSK signal,
- a Manchester matched filter with a symbol-timing recovery loop,
- differential decoding,
- block synchronization and CRC/FEC per the RDS block structure,
- and the group-level protocol decoding (PS, RadioText, RT+, PTY, TP/TA,
  clock time).

In other words: this is **a software-defined RDS demodulator/decoder — the
entire subcarrier recovery and protocol decoding chain runs in software,
right down from the raw MPX baseband sample**, running on a $2
microcontroller with no external baseband IC — only a simple analog
front-end (AC coupling, bias, band-pass, anti-alias filter) is needed
between the FM MPX signal and the ADC pin. If you're interested in
DSP-on-MCU, Costas loops, or just how RDS actually works "under the hood",
this project exposes all of that instead of hiding it behind a chip.

## Signal chain

```
FM tuner MPX out
        │  (small preamp / band-pass around 54–60 kHz recommended, see below)
        ▼
   ESP32 ADC1 (DMA, continuous mode, ~228 kS/s = 4 × 57 kHz)
        │
        ▼
 Automatic DC-bias tracking (replaces any assumption of a fixed bias point)
        │
        ▼
 57 kHz mixer  (fs/4 → mixing reduces to sign flips: +1,0,-1,0 / 0,-1,0,+1)
        │
        ▼
 4th-order CIC decimator  (÷24  →  9.5 kS/s = 8 samples/bit)
        │
        ▼
 41-tap windowed-sinc low-pass filter (~2.4 kHz cutoff)
        │
        ▼
 Costas loop (2nd order, wide bandwidth while acquiring, narrow once locked)
        │
        ▼
 Manchester matched filter + early/late bit-timing recovery
        │
        ▼
 Differential decoding → raw RDS bitstream
        │
        ▼
 Block sync (offset words A/B/C/C'/D) + CRC + 1–2 bit burst error correction
        │
        ▼
 Group decoder: PI, PS, RadioText (2A/2B), RT+ (ODA 3A + type group),
                clock time (4A), PTY/TP/TA/MS
        │
        ▼
 Serial console: live raw groups + periodic station/text/time summary
```

### Why 4× the RDS subcarrier as the sample rate

Sampling at exactly `fs = 4 × 57 kHz = 228 kHz` turns the down-conversion
mixer into pure sign flips — no multiplications, no NCO, no trig tables in
the hot path. I/Q samples are pulled straight from alternating raw ADC
samples with alternating sign. This is the classic trick that makes this
kind of demodulator cheap enough to run on a Cortex-M0+/Xtensa-class MCU in
real time.

### Why a Costas loop

RDS uses **suppressed-carrier BPSK** on the 57 kHz subcarrier — there is no
discrete carrier tone to phase-lock a PLL onto directly. A Costas loop
recovers carrier phase (and, thanks to the loop's frequency term, small
carrier frequency offsets caused by ADC clock inaccuracy) using the
in-phase/quadrature product as its phase-error signal, and is inherently
insensitive to BPSK's 180° phase ambiguity, which is why differential
coding is used on top of it.

## Repository layout

```
main/
  rds_dsp.h / rds_dsp.c      – hardware-independent DSP core (mixer, CIC,
                                FIR, Costas loop, matched filter, bit timing,
                                automatic DC-bias tracking)
  rds_group.h / rds_group.c  – hardware-independent RDS protocol decoder
                                (block sync, CRC/FEC, group parsing, PS/RT/
                                RT+/CT, RDS text-set → UTF-8)
  main.c                     – ESP-IDF glue: ADC-DMA setup, sample-rate
                                self-calibration, FreeRTOS tasks, console
                                output

host_test/
  host_test.c                – runs rds_dsp.c + rds_group.c on a PC against
                                a raw sample file (no ESP-IDF dependency)
  gen_mpx.py                  – synthesizes a test MPX signal (pilot, stereo
                                noise, RDS groups, configurable clock error
                                and noise) for regression testing
  audio_to_adc.py             – converts a real-world recording (WAV/MP3/...)
                                into the sample format host_test expects,
                                auto-detecting the RDS subcarrier if it has
                                already been down-converted (e.g. audio-rate
                                captures where 57 kHz can't exist directly)
```

`rds_dsp.c` and `rds_group.c` have **zero ESP-IDF dependencies** — they are
portable C, which is what makes the host-side testing in `host_test/`
possible without any hardware.

## Building for the ESP32

```
idf.py set-target esp32
idf.py build flash monitor
```

Targets ESP-IDF 6.x. WiFi and Bluetooth are never initialized, so they stay
off — partly to avoid their well-known interference with the ADC1
channels, partly because this project has no need for them.

## Hardware / analog front-end

The ESP32's ADC is not linear or clean enough to sample MPX directly with
good results, and the RDS subcarrier is a small fraction of the total MPX
amplitude, so a minimal analog front-end in front of the ADC pin is
recommended:

- **AC coupling** into a bias network centered on the ADC's usable input
  window (~1.65 V for `ADC_ATTEN_DB_12`, ~0.475–0.5 V for `ADC_ATTEN_DB_0`).
  A coupling capacitor forming a high-pass corner around 3–10 Hz relative to
  the bias network's Thevenin resistance keeps DC and slow drift out
  without touching the 19 kHz pilot tone.
- **Band-pass gain stage around 54–60 kHz.** This does two things: it
  boosts the comparatively small RDS component, and — more importantly — it
  suppresses the 19 kHz pilot tone and the 38 kHz±15 kHz stereo (L−R)
  sidebands. This matters because the pilot's **third harmonic sits exactly
  at 57 kHz**: any ADC nonlinearity turns pilot energy into RDS-band carrier
  leakage, and the upper edge of the L−R band is only ~4 kHz away from the
  RDS band while carrying far more energy. `ADC_ATTEN_DB_0` (smaller input
  window, more effective resolution on a weak signal) is attractive
  precisely because many MPX outputs are already in the ~100 mV range and
  need only modest extra gain (roughly 2–3×) rather than the 10–20× a
  0–3.1 V full-scale input would need.
- **Anti-alias low-pass** ahead of the ADC, since Nyquist at ~228 kS/s is
  ~114 kHz; a simple 1–2 stage RC low-pass with a corner around 90–100 kHz
  is normally sufficient.

The DC bias point does **not** need to be trimmed to hit an exact target,
because the firmware measures and tracks it automatically at runtime — see
below.

### Automatic DC-bias tracking

Earlier revisions of this decoder subtracted a hardcoded `2048` (the
midpoint of the 12-bit ADC range) from every raw sample, silently assuming
a perfectly centered bias network matched to `ADC_ATTEN_DB_12`. That
assumption breaks the moment you change attenuation settings, or simply
have normal resistor-tolerance/temperature drift in the bias divider.

The DSP core now tracks the DC bias continuously with a first-order IIR
filter running at the full ADC sample rate:

```c
dc[n] = dc[n-1] + (x[n] - dc[n-1]) >> DC_SHIFT     // Q16 fixed-point
```

`DC_SHIFT = 14` gives a time constant of roughly 72 ms (~2.2 Hz corner),
comfortably below the 19 kHz pilot tone so no wanted signal energy is
tracked out, but fast enough to settle quickly on startup and follow slow
thermal drift. The estimate is seeded from the very first sample rather
than starting from an assumed constant, and the live value is printed on
the console (`DC-Bias : ### LSB`) so a badly designed bias network is
immediately visible instead of silently degrading the demodulator.

## Sample-rate self-calibration

The nominal target is `fs = 228000 Hz` (exactly 4 × 57 kHz), but the actual
rate the ESP32's continuous ADC driver achieves may differ slightly. Since
the mixer is a fixed fs/4 operation, any deviation from the nominal rate
shows up as a residual carrier offset the Costas loop has to pull in. At
startup, `main.c` measures the real sample rate over an ~8 second window
using the system timer, computes the resulting residual offset, and — if it
exceeds a threshold — re-requests a corrected ADC clock and re-measures.
The Costas loop is then initialized with the measured residual as its
starting frequency, rather than assuming the offset is zero.

## RDS protocol coverage

- **Block synchronization** via the four RDS offset words (A/B/C/C'/D) and
  their CRC (`x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1`), acquired without any
  external frame marker.
- **1–2 bit burst error correction** per block (conservative — deliberately
  not attempting to correct longer bursts, to avoid false corrections),
  with corrected PS/RadioText segments only committed once confirmed by a
  second, consistent reception.
- **Group 0A/0B** – Programme Service name (PS), TA/MS flags.
- **Group 2A/2B** – RadioText (64 chars via 2A, or 2B's shorter variant),
  with A/B flip-flop detection to clear stale text on a text change.
- **Group 3A / ODA** – announces which group type carries RT+
  (AID `0x4BD7`), so RT+ tag data (title/artist/etc., per RBDS spec) is
  decoded generically wherever the station places it.
- **Group 4A** – clock time (CT): Modified Julian Date, UTC time, and local
  offset; the firmware keeps counting the wall clock forward between the
  once-per-minute CT groups using the system timer.
- **RDS text-set → UTF-8** conversion for the accented characters used in
  the EBU Latin-based RDS character set.

Not yet implemented: **RDS2** enhancements (e.g. Enhanced RadioText / eRT
carried in group 12A with a different ODA AID). During testing against a
real RDS2 station capture, the classic PI/PS/RT/RT+ path decoded correctly
end-to-end, while the eRT payload (which duplicates the station/text
information with full Unicode support) was visible in the raw group dump
but not yet parsed by the group decoder — a natural next extension.

## Testing without hardware

Because the DSP core and group decoder have no ESP-IDF dependency, they can
be exercised entirely on a PC:

```
cd host_test
python3 gen_mpx.py mpx.bin [ppm_error] [seconds] [rds_peak_lsb] [noise_lsb]
gcc -O2 -I../main host_test.c ../main/rds_dsp.c ../main/rds_group.c -lm -o host_test
./host_test mpx.bin 228000 [v]      # 'v' dumps every raw group
```

```
...
GRP D31C B558 8014 09CE [ABCD]
GRP D31C 4541 DEF6 C884 [ABCD]
GRP D31C 0548 E105 5445 [ABCD]
GRP D31C 2540 5465 7374 [ABCD]
GRP D31C 0548 E105 5445 [ABCD]
GRP D31C 2541 2041 7274 [ABCD]
GRP D31C 0549 E105 5354 [ABCD]
GRP D31C 2542 6973 7420 [ABCD]
GRP D31C 0549 E105 5354 [ABCD]
GRP D31C 2543 2D20 5465 [ABCD]
GRP D31C 054A E105 464D [ABCD]
GRP D31C 2544 7374 2053 [ABCD]
GRP D31C 054A E105 464D [ABCD]
GRP D31C 2545 6F6E 6720 [ABCD]
GRP D31C 054B E105 2020 [ABCD]
GRP D31C 2546 5469 746C [ABCD]
GRP D31C 054B E105 2020 [ABCD]
GRP D31C 2547 650D 2020 [ABCD]
GRP D31C 3556 0000 4BD7 [ABCD]
GRP D31C B558 8014 09CE [ABCD]
GRP D31C 4541 DEF6 C884 [ABCD]
bits=35624  locked=1  df=-0.01 Hz  spb=8.0000  rms=214.3 lock=0.96
sync=1 PI=D31C PTY=10 PS="TESTFM  " RT="Test Artist - Test Song Title"
RT+ title="Test Song Title" artist="Test Artist"
time 2026-09-24 14:34:00 (off 120 min)
groups=341 blocks ok=1366 bad=0 corrected=0 sync_losses=0
```


`gen_mpx.py` synthesizes a full MPX signal (pilot, band-limited stereo
noise, Manchester-coded/differentially-encoded RDS groups) with
configurable ADC clock error and noise, which was used to validate carrier
acquisition, bit-timing recovery and FEC across a range of conditions
(clean signal, several hundred to several thousand ppm of simulated clock
error, and signal-to-noise ratios down to single-digit LSB RDS amplitude
against tens of LSB of noise).

`audio_to_adc.py` converts a real-world recording into the same raw sample
format. If the source is an audio-rate file (e.g. an MP3 that can't
represent a 57 kHz carrier directly because its Nyquist frequency is too
low), it estimates the already-down-converted subcarrier frequency by
squaring the analytic signal (which collapses BPSK to a single spectral
line at 2×the carrier) and re-mixes it up to 57 kHz before resampling to
228 kS/s. Real MPX-rate WAV captures (e.g. 192 kHz) are simply resampled.
This tooling was validated against both a plain audio-rate RDS recording
and a genuine wideband MPX capture from a real broadcast station, both of
which decoded PI, PS, PTY and RadioText correctly. A synthetic bias offset
of several hundred LSB was also injected to confirm the automatic DC
tracking recovers identical, error-free decoding compared to a centered
signal.

## Live console output

Every 5 seconds (configurable via `PRINT_INTERVAL_MS` in `main.c`):

```
──────────── RDS  (t = 70 s) ────────────
 Empfang    : SYNC | Costas locked (Δf +0.3 Hz, Lock 0.94) | Pegel 320 | 11.4 Gruppen/s
 DC-Bias    : 2043 LSB (automatisch erkannt)
 Station    : PI 6255  PS "JarviRad"  PTY 9  TP 0 TA 0  Musik
 RadioText  : "Engelbert Humper"
 Uhrzeit    : 2026-09-26 14:33:07  (UTC+2:00)
 Statistik  : Blöcke ok 621 / schlecht 0 / korrigiert 0, Sync-Verluste 0, fs 228050.6 Hz, Overruns 0
──────────────────────────────────────────
```

(Labels are in German in the current build — a straightforward find/replace
in the `printf` calls in `main.c` if you want an English console.)

Raw group dumps (`SHOW_RAW_GROUPS`, on by default) are printed as they
arrive, showing all four blocks in hex plus which were valid (`A/B/C/D`) or
FEC-corrected (`c`) — useful for debugging reception quality independent
of the higher-level protocol decoding.

## Configuration knobs (`main/main.c`)

| Define | Purpose |
|---|---|
| `ADC_CHANNEL_USED` | ADC1 input channel (GPIO34 = `ADC_CHANNEL_6` by default) |
| `ADC_ATTEN_USED` | Input attenuation / full-scale range; see the front-end discussion above for the DB_0 vs DB_12 trade-off |
| `ADC_SAMPLE_RATE_HZ` | Nominal sample rate (228000 = 4×57 kHz) |
| `PRINT_INTERVAL_MS` | How often the station/text/time summary is printed |
| `SHOW_RAW_GROUPS` | Toggle live raw-group dump on/off |
| `RETUNE_THRESHOLD_HZ` | Residual carrier offset (from clock error) above which the ADC clock is re-requested at startup |

## Status

Confirmed working live against real over-the-air FM broadcasts: Costas
loop locks, PI comes through cleanly, groups arrive quickly, and PS /
RadioText build up progressively as segments are received — matching the
behavior validated beforehand in the host-side tests above.

## Known limitations / honest caveats

- The Costas lock detector is a soft metric (0..1) with hysteresis; on a
  very weak or badly filtered analog input it can indicate "locked" while
  still producing mostly bad blocks — the block error statistics on the
  console are the more trustworthy indicator of genuine reception quality.
- RDS2 enhanced features (eRT, tagging beyond classic RT+) are visible in
  the raw group stream but not yet decoded into the summary output.

