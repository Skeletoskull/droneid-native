# Acquiring IQ data

droneid-native does not control a radio. Acquire IQ samples with your own SDR and software,
put them in the format below, and pass them to the library as a file or a memory buffer. Any SDR
with enough bandwidth works, because the decoder only ever sees baseband samples and a sample
rate.

## Checklist

| Requirement | Value |
|---|---|
| Signal type | Complex baseband (I/Q) |
| Layout | Interleaved: `I0, Q0, I1, Q1, …` |
| Sample type, files and `detect_droneid` | 32-bit IEEE float, little-endian (`cf32_le`) |
| Sample type, `detect_droneid_d` and `droneid_process` | 64-bit IEEE float (`double`) |
| File header | None: raw samples only |
| Sample rate | **≥ 25 Msps** (validated: 25, 30.72, 50, 100 Msps) |
| Tuning | The ~10 MHz DroneID channel must fit inside the captured band |
| Capture length | ≥ 640 ms per capture (beacon period); shorter buffers are fine for gap-free streaming |
| Gain | Fixed gain, no ADC clipping |
| File size | < 2 GiB per file for the `*_file` functions |

## 1. Sample format

Each complex sample is two numbers, in-phase then quadrature, stored back to back:

```
offset  0      4      8      12     16 ...
        | I0   | Q0   | I1   | Q1   | I2 ...     float32, little-endian
```

- A file of `N` complex samples is exactly `8·N` bytes. There is no header, footer or metadata.
- In memory, `float32` samples are passed as `const float *` and `float64` samples as
  `const double *`. Pass `num_samples` as the number of **complex** samples, which is the
  array length divided by 2.
- In LabVIEW, an array of complex doubles (`CDB[]`) already has this memory layout and can be
  passed directly to `droneid_process`.
- The absolute amplitude does not matter. The detector threshold is relative to the measured
  noise floor, so full-scale `±1.0`, raw ADC counts converted to float and any other scaling all
  work.

## 2. Sample rate

A DroneID burst occupies about 10 MHz (601 OFDM subcarriers at 15 kHz spacing). The detector
looks for a band 8–11 MHz wide, and the decoder resamples each burst to 15.36 MHz internally.

| Sample rate | Result on real captures |
|---|---|
| 100 Msps | Decodes |
| 50 Msps | Decodes |
| 30.72 Msps | Decodes |
| 25 Msps | Decodes |
| 20 Msps | Marginal: some frames are missed |
| 15.36 Msps | Does not decode |

Use **at least 25 Msps**. Higher rates let one capture cover more channels at once (see
below), at the cost of more CPU time for detection. Pass the exact sample rate you captured at;
the library has no way to infer it.

## 3. Tuning and channel coverage

DroneID is transmitted on a set of fixed channels in the 2.4 GHz and 5.8 GHz bands. The reference
receiver from the NDSS 2023 paper scans these centre frequencies (MHz):

- **2.4 GHz:** 2414.5, 2429.5, 2434.5, 2444.5, 2459.5, 2474.5
- **5.8 GHz:** 5721.5, 5731.5, 5741.5, 5756.5, 5761.5, 5771.5, 5786.5, 5801.5, 5816.5, 5831.5

The burst does not need to be at the centre of your capture. The detector estimates each
burst's offset from the centre frequency (`cfo_hz` in `detection_result_t`) and shifts it to
baseband before decoding. A channel is decodable when its full ~10 MHz width lies inside the
usable part of your capture band:

```
|channel_centre - tuned_centre| + 5 MHz  <  usable_bandwidth / 2
```

Usable bandwidth is somewhat less than the sample rate, because the SDR's anti-alias filter
rolls off near the band edges.

You can trade bandwidth for retuning:

- **Wide capture (≈100 Msps).** One capture centred near 2444.5 MHz covers every 2.4 GHz channel.
  The 5.8 GHz channels need two or three centre frequencies.
- **Narrow capture (25–30 Msps).** Tune to one channel at a time and hop through the list.
  Narrow captures pick up less adjacent-band interference, which can help sensitivity.

A drone uses one channel at a time and its beacon repeats every 640 ms, so stay on each
frequency for at least 640 ms before retuning.

## 4. Capture length

The DroneID beacon repeats every **640 ms**, and each burst lasts about 0.64 ms.

- **One-shot or frequency-hopping captures.** Make each capture **at least 640 ms** long so it
  is guaranteed to contain a beacon. 1.0–1.3 s gives margin for a burst that straddles the edge.
- **Gap-free streaming on one frequency.** Buffers can be shorter (for example 40–300 ms),
  because the next beacon will land in a later buffer. Longer buffers mean fewer calls and fewer
  bursts cut by a buffer boundary.
- **Boundary losses.** A burst cut off at the start or end of a buffer cannot be decoded.
  Overlapping consecutive buffers by about 1 ms avoids losing it.
- **Clip length.** Very short clips (a few bursts packed into tens of milliseconds) are not a
  good input, because the detector's noise-floor estimate assumes most of the capture is noise.
  See the [README](../README.md#limitations).

Buffer size: `num_samples = sample_rate × duration`. For example, 100 Msps × 0.65 s = 65 M
samples, which is 520 MB as `float32` or 1.04 GB as `float64`. `num_samples` is a 32-bit
count, so keep each buffer below 4.29 G samples.

## 5. Gain and front end

- **Use a fixed gain.** Automatic gain control tends to back off when strong Wi-Fi or video
  links are nearby, which buries the weaker DroneID burst.
- **Avoid clipping.** Saturated samples break the OFDM demodulation. Leave a few dB of headroom
  below ADC full scale.
- **Range depends on the RF front end.** The decoder needs a clean, unclipped burst with enough
  in-channel SNR. A low-noise amplifier close to the antenna and a suitable 2.4/5.8 GHz antenna
  increase range far more than any software setting.

## 6. Spectral inversion

Some acquisition chains deliver the spectrum mirrored, which is the same as swapping I and Q.
If bursts are detected (`droneid_candidates` lists them) but never decode, try flag `0x4`
(`DETECT_FLAG_CONJ` / `LOCATE_FLAG_CONJ`), which conjugates the input before processing.

## 7. Converting existing recordings

All of these produce the raw `cf32_le` file the library expects (Python and NumPy):

```python
import numpy as np

# Interleaved signed 16-bit (sc16 / ci16_le) -> cf32
raw = np.fromfile("capture.sc16", dtype="<i2").astype(np.float32) / 32768.0
raw.tofile("capture.bin")

# Interleaved signed 8-bit (sc8 / ci8) -> cf32
raw = np.fromfile("capture.sc8", dtype=np.int8).astype(np.float32) / 128.0
raw.tofile("capture.bin")

# Interleaved float64 -> cf32
np.fromfile("capture.cf64", dtype="<f8").astype("<f4").tofile("capture.bin")

# Separate I and Q arrays -> interleaved cf32
iq = np.empty(2 * len(i_samples), dtype="<f4")
iq[0::2], iq[1::2] = i_samples, q_samples
iq.tofile("capture.bin")
```

- A **SigMF** recording with `core:datatype = "cf32_le"` can be used as is: pass the
  `.sigmf-data` file, and take the sample rate from `core:sample_rate` in the `.sigmf-meta` file.
- **GNU Radio** `File Sink` blocks with the *complex* item type write `cf32_le` directly.
- Big-endian or unsigned formats need conversion first (for example `dtype=">i2"`, or subtract
  the offset for unsigned samples).

## 8. Checking a new setup

1. Record about 1.3 s at a known sample rate while a DJI drone is powered on and linked.
2. Run `droneid_candidates capture.bin <fs>`. Each beacon should show up as a candidate lasting
   about 640 µs (about 580 µs for legacy models), with a plausible CFO.
3. Run `droneid_scan capture.bin <fs>`. Frames reported with `crc=OK` mean the whole chain works.

| Symptom | Likely cause |
|---|---|
| No candidates | Wrong sample rate, channel outside the capture band, capture shorter than 640 ms, burst too weak, or wrong sample type (int16 read as float) |
| Candidates but no decode | Spectral inversion (try flag `0x4`), clipping, low SNR, or a legacy model (try flag `0x1`) |
| Every buffer returns `-1` | `num_samples` is 0 or a NULL pointer was passed |
