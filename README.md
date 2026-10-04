# droneid-native

[![CI](https://github.com/Skeletoskull/droneid-native/actions/workflows/ci.yml/badge.svg)](https://github.com/Skeletoskull/droneid-native/actions/workflows/ci.yml)
[![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-blue.svg)](LICENSE)

A native C library that detects and decodes DJI **DroneID** (OcuSync 2.0) broadcasts
from raw IQ samples, in real time. It reports the drone's serial number and position
and the pilot's location.

- **Real time at 100 Msps.** A 0.65 s, 100 Msps capture is processed in about 0.5 s on a
  4-core laptop CPU.
- **SDR-independent.** The library only sees complex baseband samples and a sample rate.
  Any SDR with enough bandwidth works; see [Acquiring IQ data](docs/iq-capture-format.md).
- **No runtime dependencies.** Three C-ABI shared libraries, a bundled FFT and no Python.
  Prebuilt Windows x64 DLLs are included.
- **LabVIEW-ready.** One function, `droneid_process()`, takes a block of IQ and returns flat
  scalars that a Call Library Function Node can read directly.
- **CRC-validated output.** A frame is reported only when its CRC-16 checks out.

This library is a C reimplementation of the Python proof-of-concept receiver published with
*"Drone Security and the Mysterious Case of DJI's DroneID"* (NDSS 2023). That receiver was
built for correctness, not speed. This port keeps the same decoding math, adds an
anti-aliased resampler and a CFO search for wideband captures, and is fast enough for
continuous live use.

---

## Why droneid-native

Same captures, same machine (4-core laptop CPU), compared against the unmodified reference
receiver:

| | Reference receiver (Python) | droneid-native |
|---|---|---|
| Detection, 1.3 s capture at 50 Msps | 5.3 s, finds **0** of 3 bursts | 0.48 s, finds **3** of 3 (**11× faster**) |
| Decode one burst at 50 Msps | 1376 ms | 52 ms (**26× faster**) |
| Decode one burst at 100 Msps | 1763 ms, CRC fails | 68 ms, CRC valid |
| 100 Msps capture vs air time | 7.7× slower than real time | Faster than real time (0.53 s for 0.65 s) |
| Detection memory per 500 ms at 50 Msps | ≈400 MB STFT matrix | ≈3 MB (**≈128× less**) |
| Runtime | Python, NumPy, SciPy, Matplotlib | Self-contained DLLs; callable from LabVIEW, C, Python |

**Why it is faster:** compiled C in place of per-sample Python loops; a coarse-to-fine timing
search (about 100 evaluations instead of 1000); an OpenMP-parallel STFT detector that keeps
one value per frame instead of a full matrix; and cheap early rejection of false candidates
(ZC root check, then hard decision and CRC, with turbo decoding only as a fallback).

**Why it decodes more:** a median noise floor that works on long, sparse, real-world captures;
an anti-aliased Lanczos-3 resampler instead of linear interpolation; an integer-subcarrier CFO
search; and a real LTE turbo decoder where the reference reads only the systematic bits.

The full methodology, per-stage numbers, and the cases where the reference still does better
are in [How droneid-native compares](docs/comparison.md).

---

## Contents

- [Why droneid-native](#why-droneid-native)
- [Quick start](#quick-start)
- [Libraries](#libraries)
- [Input format](#input-format)
- [Building from source](#building-from-source)
- [API overview](#api-overview)
- [Tools](#tools)
- [Performance](#performance)
- [Documentation](#documentation)
- [Limitations](#limitations)
- [Acknowledgements and license](#acknowledgements-and-license)

---

## Quick start

### Windows, with the prebuilt DLLs (no build step)

[`bin/windows-x64/`](bin/windows-x64/) holds the full runtime set:

| File | Purpose |
|---|---|
| `droneid_bridge.dll` | One-call wrapper: `droneid_process()` |
| `droneid_detect.dll` | Burst detection |
| `droneid_locate.dll` | Frame decoding |
| `vcomp140.dll` | Microsoft OpenMP runtime (used by the detector) |

Keep the four files together, next to your executable or VI. To check them against a capture
without compiling anything, run this (requires Python 3 and NumPy):

```bash
python tools/droneid_process.py capture.bin 50e6
```

### LabVIEW

Add one **Call Library Function Node** for `droneid_process` in `droneid_bridge.dll` (C calling
convention). Wire your acquisition's complex-double array into `iq_interleaved` and its length into
`num_samples`, then read the decoded fields from the outputs. The full 19-parameter table is in
the [LabVIEW integration guide](docs/labview-integration.md).

### C

```c
#include "droneid_locate.h"
#include <stdio.h>

int main(void) {
    telemetry_result_t *frames = NULL;
    uint32_t n = 0;

    /* interleaved little-endian float32 I/Q, recorded at 50 Msps */
    if (locate_droneid_file("capture.bin", 50e6, 0, &frames, &n) == LOCATE_OK) {
        for (uint32_t i = 0; i < n; i++)
            printf("%s  drone=(%.6f, %.6f)  pilot=(%.6f, %.6f)\n",
                   frames[i].serial_number, frames[i].latitude, frames[i].longitude,
                   frames[i].app_lat, frames[i].app_lon);
    }
    locate_free_results(frames, n);
    return 0;
}
```

A complete example that runs the detect and decode stages separately is in
[`examples/c_example.c`](examples/c_example.c).

---

## Libraries

| Library | Role |
|---|---|
| `droneid_detect` | Finds DroneID bursts: STFT energy detection, burst-duration gate, 8–11 MHz bandwidth check, coarse CFO estimate, anti-aliased resampling to 15.36 MHz |
| `droneid_locate` | Decodes one burst: OFDM timing and fractional CFO, integer-subcarrier CFO search, Zadoff-Chu validation, QPSK demapping, descrambling, de-rate-matching, LTE turbo decoding (fallback), DUML parsing, CRC-16 |
| `droneid_bridge` | `droneid_process()`: runs detect and locate over one IQ buffer and returns the first decoded frame as flat outputs plus JSON |

All three are stateless. Concurrent calls on separate buffers are safe.

---

## Input format

The libraries take **complex baseband IQ, interleaved `I0 Q0 I1 Q1 …`**:

| Entry point | Sample type |
|---|---|
| `detect_droneid`, `locate_droneid_file`, `detect_droneid_file` | `float32` (files: raw little-endian `cf32`, no header) |
| `detect_droneid_d`, `droneid_process` | `float64` |

The capture has to meet these requirements:

- **Sample rate:** at least **25 Msps**. Validated at 25, 30.72, 50 and 100 Msps.
- **Tuning:** the whole ~10 MHz DroneID channel must lie inside the captured band. The
  detector finds the channel's offset from the centre frequency by itself.
- **Length:** a single capture needs at least **640 ms** to be sure of containing a beacon
  (DroneID repeats every 640 ms). For gap-free streaming, shorter buffers are fine.
- **Level:** fixed gain with no clipping. The absolute scale does not matter.

You can capture the IQ with any SDR and any software, as long as you write it out in this
format. [Acquiring IQ data](docs/iq-capture-format.md) covers each requirement in detail and
includes conversion snippets for common recording formats (int16, int8, SigMF, GNU Radio).

---

## Building from source

Requires CMake ≥ 3.16 and a C99 compiler. Builds have been tested with MSVC 2022 on Windows
and GCC 13 on Linux.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

On Windows, use the Visual Studio generator: `cmake -B build -G "Visual Studio 17 2022" -A x64`.
Libraries, tools and tests are written to `build/bin/Release/`.

| Option | Default | Meaning |
|---|---|---|
| `USE_FFTW3` | `OFF` | Use FFTW3 instead of the bundled FFT |
| `DRONEID_BUILD_TESTS` | `ON` | Build the unit tests |
| `DRONEID_BUILD_TOOLS` | `ON` | Build the command-line tools |
| `DRONEID_TEST_CAPTURE` | *(empty)* | Path to a real capture; adds an end-to-end `validate_capture` test |
| `DRONEID_TEST_CAPTURE_RATE` | `50000000` | Sample rate of that capture |

OpenMP is used automatically when the compiler supports it. More detail, including
installation and deployment, is in [Building and deployment](docs/building.md).

---

## API overview

```c
/* droneid_detect.h */
int32_t detect_droneid      (const float  *iq, uint32_t n, double fs, uint32_t flags,
                             detection_result_t **out, uint32_t *count);
int32_t detect_droneid_d    (const double *iq, uint32_t n, double fs, uint32_t flags,
                             detection_result_t **out, uint32_t *count);
int32_t detect_droneid_file (const char *path, double fs, uint32_t flags,
                             detection_result_t **out, uint32_t *count);
void    detect_free_results (detection_result_t *results, uint32_t count);

/* droneid_locate.h */
int32_t locate_droneid      (const locate_complex_t *candidate, uint32_t n, uint32_t flags,
                             telemetry_result_t *out);
int32_t locate_droneid_file (const char *path, double fs, uint32_t flags,
                             telemetry_result_t **out, uint32_t *count);
void    locate_free_results (telemetry_result_t *results, uint32_t count);
int32_t telemetry_to_json   (const telemetry_result_t *t, char *buf, uint32_t len);

/* droneid_bridge.h */
int32_t droneid_process     (const double *iq, uint32_t n, double fs, uint32_t flags,
                             /* 15 flat output parameters */ ...);
```

**Flags.** One bitfield is shared by every entry point.

| Value | Name | Effect |
|---|---|---|
| `0x1` | `*_FLAG_LEGACY` | Legacy frame format (Mavic Pro / Mavic 2): 565–600 µs bursts, 8 OFDM symbols |
| `0x2` | `LOCATE_FLAG_ASSUME_ZC` | Skip ZC root validation and report the first demodulated frame even if its CRC fails |
| `0x4` | `*_FLAG_CONJ` | Conjugate the input, to undo a spectrally inverted recording |

**Status codes.**

| Library | Codes |
|---|---|
| detect | `0` OK, `1` no frames, `-1` invalid argument, `-2` out of memory |
| locate | `0` OK, `-1` invalid argument, `-2` no decodable frame, `-4` JSON buffer too small, `-5` out of memory |
| bridge | `0` decoded, `1` no frame, `-1` invalid argument, `-2` detection error, `-3` out of memory |

The full documentation is in the headers in [`include/`](include/).

---

## Tools

The tools are built into `build/bin/`.

| Tool | Purpose |
|---|---|
| `droneid_scan <file> <fs> [flags]` | Detect and decode every frame in a capture |
| `droneid_candidates <file> <fs> [flags]` | List burst candidates (time, duration, CFO). Useful when checking a new capture setup |
| `droneid_bridge_cli <file> <fs> [flags] [chunk_ms]` | Run `droneid_process()` exactly as LabVIEW calls it, optionally in acquisition-sized buffers |
| `droneid_validate <file> <fs> [--legacy]` | End-to-end checks on a capture (used by the optional `validate_capture` test) |
| `turbo_selftest` | Self-test of the turbo FEC engine |
| `tools/droneid_process.py` | Python ctypes driver for the prebuilt DLLs |

Example:

```text
> droneid_scan capture.bin 100000000
capture.bin @ 100.00 Msps: rc=0, 1 frame(s)
  [0] serial=XXXXXXXXXXXXXX  seq=3274  lat=...  lon=...  alt=... m  pilot=(..., ...)  crc=OK
```

---

## Performance

Measured on an Intel Core i5-10310U (4 cores / 8 threads) with real over-the-air captures,
`flags = 0`, KissFFT and OpenMP enabled:

| Capture | Air time | Detection | Decode | Total compute |
|---|---|---|---|---|
| 100 Msps, 65 M samples, 1 beacon | 0.65 s | ~460 ms | ~65 ms | **~0.53 s** |
| 50 Msps, 65 M samples, 3 beacons | 1.30 s | ~480 ms | ~55 ms per frame | **~0.65 s** |

Compute time is shorter than air time in both cases, so the decoder keeps up with a live
stream. Detection (the STFT) accounts for most of the cost and scales with core count. Decoding
takes tens of milliseconds per frame. Set the environment variable `DRONEID_PROF=1` to print this
breakdown for `locate_droneid_file`.

---

## Documentation

| Document | Contents |
|---|---|
| [Acquiring IQ data](docs/iq-capture-format.md) | Required format, sample rate, tuning, gain, capture length, conversion snippets |
| [LabVIEW integration](docs/labview-integration.md) | `droneid_process` CLFN configuration, VI structure, troubleshooting |
| [Building and deployment](docs/building.md) | Build options, tests, installation, runtime dependencies |
| [Architecture](docs/architecture.md) | Signal-processing pipeline, source layout, tuning knobs |
| [Comparison](docs/comparison.md) | Measured speed, memory and decode-rate comparison with the reference receiver |
| [Changelog](CHANGELOG.md) | Release history |

---

## Limitations

- **Short, signal-dense clips.** The detector's noise floor is the median of the per-frame
  STFT power. That suits long, sparse, over-the-air captures. On very short clips where
  bursts fill a large share of the samples, the median rises into the signal and frames
  can be missed. Use captures of at least one beacon period (640 ms).
- **Files larger than 2 GiB** are not supported by the `*_file` functions. Split long
  recordings, or pass buffers to `detect_droneid` / `droneid_process`.
- **Range** depends on your RF front end (antenna, LNA, gain), not on this software. A burst
  needs enough in-channel SNR for synchronisation and the CRC to succeed.
- **Telemetry content** is whatever the drone broadcasts. Position fields read zero until
  the drone has a GPS fix.

---

## Acknowledgements and license

The decoding pipeline follows the reference receiver from the NDSS 2023 paper
*"Drone Security and the Mysterious Case of DJI's DroneID"* (N. Schiller et al., Ruhr University
Bochum and CISPA), [RUB-SysSec/DroneSecurity](https://github.com/RUB-SysSec/DroneSecurity).
The bundled FFT is derived from [KissFFT](https://github.com/mborgerding/kissfft) (BSD-3-Clause).
See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

Released under the [GNU Affero General Public License v3.0](LICENSE), the same license as the
reference implementation.

Receiving and decoding DroneID broadcasts may be regulated where you live. You are responsible
for using this software lawfully.
