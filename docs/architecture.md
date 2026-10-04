# Architecture

## Signal-processing pipeline

```
 raw IQ (any rate ≥ 25 Msps)
   │
   ├─ droneid_detect ──────────────────────────────────────────────────────────┐
   │   1. STFT energy envelope      64-pt FFT, 50 % overlap, peak bin per frame │
   │   2. Burst gate                > 1.15 × median floor, 550–750 µs           │
   │                                (legacy: 565–600 µs)                        │
   │   3. Band check + coarse CFO   2048-pt Welch PSD, 8–11 MHz occupied band   │
   │   4. Shift to baseband         per-burst frequency shift by −CFO           │
   │   5. Resample                  anti-aliased Lanczos-3 → 15.36 MHz          │
   └────────────────────────────────────────────────────────────────────────────┘
   │  candidate burst @ 15.36 MHz
   ├─ droneid_locate ──────────────────────────────────────────────────────────┐
   │   6. Integer CFO search        shifts of m × 15 kHz, m = 0, −1, +1, …      │
   │   7. Fine timing + FFO         cyclic-prefix autocorrelation               │
   │   8. OFDM demodulation         9 symbols (legacy 8), 1024-pt FFT,          │
   │                                601 subcarriers                             │
   │   9. ZC validation             root 147 on the second ZC symbol            │
   │  10. QPSK demapping            4 phase hypotheses, raw (non-equalised)     │
   │                                symbols                                     │
   │  11. Descramble                3GPP Gold sequence                          │
   │  12. De-rate-match + FEC       systematic hard decision first;             │
   │                                LTE turbo decode as fallback                │
   │  13. DUML parse + CRC-16       91-byte payload, CRC over bytes 0–88        │
   └────────────────────────────────────────────────────────────────────────────┘
   │
 telemetry_result_t / JSON  (reported only when the CRC is valid)
```

### DroneID physical layer

| Parameter | Value |
|---|---|
| Waveform | OFDM, 601 subcarriers (600 + DC), 15 kHz spacing, 1024-point FFT at 15.36 MHz |
| Occupied bandwidth | ≈ 10 MHz |
| Frame | 9 OFDM symbols (8 on legacy models); cyclic prefix 80 samples on the first and last symbols, 72 otherwise |
| Synchronisation | Zadoff-Chu sequences (N = 601), roots 600 and 147, on two of the symbols |
| Modulation / coding | QPSK; LTE turbo code (K = 1408) with LTE sub-block interleaving; 3GPP Gold-sequence scrambling |
| Payload | 91-byte DUML record, CRC-16 (poly 0x1021 reflected, init 0x3692) |
| Repetition | One beacon every 640 ms |

### Design decisions

- **The CRC is the validator.** A wrong frequency shift, timing or noise cannot produce a
  valid CRC-16, so the decoder tries several hypotheses cheaply and reports only frames whose
  CRC checks out. This lets it skip the expensive 600-root Zadoff-Chu search in standard mode.
  A single-root gate (root 147) rejects most non-DroneID bursts before demodulation.
- **Integer-CFO search.** The coarse CFO from the Welch PSD is quantised to PSD bins. The
  cyclic-prefix estimator only resolves offsets within ±½ subcarrier. The decoder therefore
  tries integer subcarrier shifts in the order 0, −1, +1, … ±10 (±150 kHz,
  `DRONEID_CFO_SPAN`). Clean bursts match at 0 and cost nothing extra.
- **Anti-aliased resampling.** Decimating a 100 Msps capture to 15.36 MHz with linear
  interpolation folds adjacent Wi-Fi and Bluetooth energy onto the DroneID channel. A
  Lanczos-3 kernel avoids that.
- **Raw-symbol QPSK.** Data symbols are demapped without channel equalisation, as in the
  reference receiver. All four phase rotations are tried and the CRC picks the right one.
- **Fast path first.** A systematic-bit hard decision decodes most clean frames. The full
  max-log-MAP turbo decoder runs only when the CRC fails, which recovers weaker frames.
- **Parallel detection.** The STFT envelope accounts for most of the CPU time at high sample
  rates. It runs in parallel with OpenMP, using one FFT plan per thread. Decoding a burst
  takes tens of milliseconds.
- **Stateless C ABI.** There are no handles or global state across calls. Results are
  allocated by the library and released with the matching `*_free_results` function.

## Source layout

```
include/             public headers: droneid_detect.h, droneid_locate.h, droneid_bridge.h
src/dsp/             shared DSP primitives (static library linked into each DLL)
  fft.c              FFT backend wrapper (bundled FFT or FFTW3), reusable plans
  stft.c             STFT energy envelope (OpenMP)
  welch_psd.c        Welch PSD, band search and coarse CFO
  resample.c         Lanczos-3 resampler
  fshift.c           frequency shift
  zc_sequence.c      Zadoff-Chu sequences
  gold_sequence.c    3GPP Gold sequence
  correlation.c      cross-correlation
src/detect/          droneid_detect: burst detector and public API
src/locate/          droneid_locate
  locate_api.c       per-burst decode loop (CFO search, validation)
  locate_file.c      file ingest: detection + decoding in one call
  ofdm_demod.c       timing, FFO, OFDM symbol extraction
  zc_sync.c          ZC root detection and channel estimate
  qpsk_decode.c      QPSK demapping, descrambling, hard-decision path
  turbo_deratematch.c, turbo_decode.c   LTE de-rate-matching and turbo decoder
  duml_parser.c, crc16.c, json_serializer.c
src/bridge/          droneid_bridge: droneid_process() for LabVIEW and other FFI callers
tests/               unit tests (synthetic signals)
tools/               command-line tools and the Python ctypes driver
examples/            minimal C example
third_party/kissfft/ bundled FFT
bin/windows-x64/     prebuilt Windows DLLs
```

## Tuning knobs

| Knob | Where | Default | Effect |
|---|---|---|---|
| Detection threshold | `PKT_STFT_THRESHOLD`, `src/detect/packet_detector.h` | 1.15 × median | Lower values detect weaker bursts but produce more false candidates |
| Burst duration gate | `PKT_MIN/MAX_DUR_*`, same file | 550–750 µs (legacy 565–600 µs) | Accepted burst lengths |
| Bandwidth gate | `PKT_BW_MIN/MAX_HZ`, same file | 8–11 MHz | Accepted occupied bandwidth |
| CFO search span | env `DRONEID_CFO_SPAN` | 10 (±150 kHz) | Tolerance to residual frequency offset |

`src/locate/locate_file.c` repeats the detection constants for the file path, so keep the two
in sync if you change them.
