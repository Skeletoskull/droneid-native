# How droneid-native compares to the reference receiver

droneid-native reimplements the Python proof-of-concept receiver published with the NDSS 2023
paper *"Drone Security and the Mysterious Case of DJI's DroneID"*
([RUB-SysSec/DroneSecurity](https://github.com/RUB-SysSec/DroneSecurity)). The reference receiver
was written to reproduce the paper's results; its README describes it as not optimised for bad RF
conditions, performance or range. This page explains what droneid-native changes, and shows the
measured effect on the same recordings.

## Measured comparison

Both receivers processed the same real over-the-air captures of a DJI drone, on the same machine
(Intel Core i5-10310U, 4 cores / 8 threads, Windows 11). The reference receiver is the unmodified
upstream code (commit `9ff8198`) on Python 3.13. Its only change was restoring three NumPy
aliases removed in NumPy 2 (`np.complex`, `np.float`, `np.int`), which does not alter any
computation. Python import time is excluded.

### 50 Msps capture (1.30 s of air time, 3 beacons)

| | Reference (Python) | droneid-native | |
|---|---|---|---|
| Detection over the whole capture | 5.28 s | 0.48 s | **11× faster** |
| Bursts found | **0** | **3** | |
| Decode one burst (given its position) | 1376 ms | 52 ms | **26× faster** |
| Frames with valid CRC | 0 | 3 | |
| Real-time capable | No (4× slower than air time) | Yes (0.65 s for 1.30 s) | |

### 100 Msps capture (0.65 s of air time, 1 beacon)

| | Reference (Python) | droneid-native | |
|---|---|---|---|
| Detection over the whole capture | 4.99 s | ~0.47 s | **≈11× faster** |
| Bursts found | **0** | **1** | |
| Decode one burst (given its position) | 1763 ms, **CRC fails** | 68 ms, CRC valid | **26× faster**, and it decodes |
| Real-time capable | No (7.7× slower than air time) | Yes (≈0.53 s for 0.65 s) | |

The "decode one burst" rows hand the reference receiver the burst's exact position (its
skip-detection path), so they measure decoding alone, with no help from detection. Even then the
reference cannot decode the 100 Msps burst, for the reasons described under
[Why it decodes more](#why-it-decodes-more).

## Why it is faster

1. **Compiled code instead of interpreted loops.** Several reference stages loop in Python. For
   example, fine-timing search computes a cyclic-prefix correlation separately at every sample
   offset of the burst, about 16 000 interpreted iterations per burst. droneid-native runs the
   same arithmetic as compiled C.
2. **Coarse-to-fine timing search.** The reference evaluates 1000 evenly spaced sub-sample
   offsets. Each evaluation re-interpolates the whole burst and recomputes nine FFTs. droneid-native
   searches coarse-to-fine and reaches the same resolution with about 100 evaluations.
3. **A parallel, streaming detector.** The reference builds a full 64-bin complex STFT matrix
   with SciPy and then averages it. droneid-native computes the STFT on every CPU core with
   OpenMP, uses pre-built FFT plans with one per thread, and keeps one number per STFT frame (the
   peak bin power), taking a single square root per frame.
4. **Early rejection by CRC.** Each candidate is checked cheaply first: Zadoff-Chu root 147,
   then a fast hard-decision decode and the CRC-16. The expensive turbo decoder runs only when
   those fail, and false candidates are discarded within milliseconds.

## Why it is more efficient

| Resource | Reference (Python) | droneid-native |
|---|---|---|
| Detection working memory, per 500 ms at 50 Msps | ≈400 MB STFT matrix, plus ≈200 MB for its magnitude | ≈3 MB (one float per STFT frame), **≈128× less** |
| Reading a capture file | Copies the entire file into RAM (`memmap(...).astype(...)`) | Streams it in 500 ms chunks: memory is bounded regardless of file length |
| Runtime | Python with NumPy, SciPy, Matplotlib and bitarray | Three self-contained DLLs plus the OpenMP runtime, no interpreter |
| Cost of a false candidate | Full demodulation, about 1.4 s | Rejected in milliseconds |
| CPU use | One core | All cores during detection |

At 100 Msps the reference receiver needs about 1.2 GB per 500 ms chunk just for the STFT and its
magnitude. droneid-native uses about 6 MB for the same step.

## Why it decodes more

| Problem in the reference | Effect | droneid-native |
|---|---|---|
| Noise floor = **mean** over all STFT bins | In long, sparse, real-world captures, ordinary noise frames exceed the threshold, so bursts merge into over-long runs that the duration gate rejects. The reference finds **0 of 3** bursts in the 50 Msps capture | Noise floor = **median** of per-frame peak power, which finds all 3 |
| **Linear interpolation** (`np.interp`) to resample to 15.36 MHz | Decimating 100 Msps by 6.5× folds adjacent Wi-Fi and Bluetooth energy onto the DroneID channel | **Anti-aliased Lanczos-3** resampler |
| **No integer-CFO correction** | The coarse offset estimate is quantised to PSD bins, and residuals beyond ±½ subcarrier (±7.5 kHz) break demodulation | Searches integer subcarrier shifts (0, ±1, … ±10, i.e. ±150 kHz); the CRC selects the correct one |
| **Systematic bits only**, no FEC decoding | Any bit error in the systematic stream fails the CRC | Hard decision first, then a full **LTE turbo decoder** (max-log-MAP) as fallback, which recovers weaker frames |
| By default, assumes ZC roots 600/147 **without checking** | Non-DroneID bursts go through full demodulation | Checks root 147 before demodulating |

## Integration

| | Reference (Python) | droneid-native |
|---|---|---|
| Form | Command-line Python scripts | C-ABI shared libraries; callable from C/C++, LabVIEW, Python (ctypes), C#, MATLAB… |
| LabVIEW | No | `droneid_process()`: one Call Library Function Node with flat outputs |
| Live use | Batch processing of capture windows | Faster than real time, so it fits a continuous acquisition loop |
| API | Script output | Stateless, thread-safe functions; JSON serialisation built in |

## Where the reference still has the edge

- **Short, signal-dense clips.** The reference's mean-based noise floor suits clips where bursts
  fill a large share of the samples, such as the two short sample files bundled with the
  reference repository, which it decodes and droneid-native does not. droneid-native's median-based
  floor is tuned for real, long over-the-air captures instead. See the
  [README limitations](../README.md#limitations).
- **Interactive inspection.** The reference ships matplotlib debugging plots and a GUI for
  examining individual frames. droneid-native is a library and has no plotting.

## Reproducing the measurements

droneid-native timings come from `DRONEID_PROF=1 droneid_scan <capture> <fs>` (best of several
runs). Reference timings come from calling its own `SpectrumCapture`, `Packet`, `Decoder` and
`DroneIDPacket` classes directly in a timing harness: detection over the full capture in 500 ms
chunks as its offline receiver does, and decoding of a 1.5 ms window around a known burst
through its skip-detection path, best of three. The captures are private recordings and are not
distributed; any capture that meets the [IQ requirements](iq-capture-format.md) can be used
instead.
