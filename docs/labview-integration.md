# LabVIEW integration

LabVIEW calls the libraries through a **Call Library Function Node (CLFN)**. The recommended
entry point is `droneid_process()` in `droneid_bridge.dll`. It is a single call that runs detection
and decoding on one IQ buffer and returns flat scalar and string outputs. All memory is allocated
and freed inside the DLL.

```
acquisition (complex double array)  ──►  droneid_process()  ──►  serial / position / pilot / JSON
```

## 1. Deploy the DLLs

Copy the four files from [`bin/windows-x64/`](../bin/windows-x64/) to a permanent folder, either
next to the VI and built application, or in a folder on `PATH`:

```
droneid_bridge.dll   droneid_detect.dll   droneid_locate.dll   vcomp140.dll
```

The bridge loads the other three from its own folder, so **keep all four together**. They are
64-bit DLLs and need **64-bit LabVIEW**.

## 2. Configure the CLFN

| Field | Value |
|---|---|
| Library name or path | `droneid_bridge.dll` (relative to the VI, or an absolute path) |
| Function name | `droneid_process` |
| Calling convention | **C** |
| Thread | *Run in UI thread* while you set it up; *Run in any thread* once it works |
| Return type | Numeric, signed 32-bit integer |

Parameters, in order:

| # | Name | CLFN type | Pass | Direction |
|---|---|---|---|---|
| 1 | `iq_interleaved` | Array, 8-byte double, 1 dimension | Array Data Pointer | in |
| 2 | `num_samples` | Unsigned 32-bit integer | Value | in |
| 3 | `sample_rate` | 8-byte double | Value | in |
| 4 | `flags` | Unsigned 32-bit integer | Value | in |
| 5 | `num_frames_out` | Signed 32-bit integer | Pointer to Value | out |
| 6 | `crc_valid_out` | Signed 32-bit integer | Pointer to Value | out |
| 7 | `serial_out` | String, C String Pointer, **minimum size 17** | — | out |
| 8 | `latitude_out` | 8-byte double | Pointer to Value | out |
| 9 | `longitude_out` | 8-byte double | Pointer to Value | out |
| 10 | `altitude_m_out` | 8-byte double | Pointer to Value | out |
| 11 | `height_m_out` | 8-byte double | Pointer to Value | out |
| 12 | `app_lat_out` | 8-byte double | Pointer to Value | out |
| 13 | `app_lon_out` | 8-byte double | Pointer to Value | out |
| 14 | `v_north_out` | Signed 16-bit integer | Pointer to Value | out |
| 15 | `v_east_out` | Signed 16-bit integer | Pointer to Value | out |
| 16 | `sequence_number_out` | Unsigned 16-bit integer | Pointer to Value | out |
| 17 | `device_type_id_out` | Unsigned 8-bit integer | Pointer to Value | out |
| 18 | `json_out` | String, C String Pointer, **minimum size 4096** | — | out |
| 19 | `json_buf_len` | Unsigned 32-bit integer (wire `4096`) | Value | in |

The C prototype and the same table are in [`include/droneid_bridge.h`](../include/droneid_bridge.h).

### Wiring rules

- **IQ array.** An array of complex doubles (`CDB[]`) has the interleaved `I0, Q0, I1, Q1, …`
  layout the DLL expects. Wire it straight into parameter 1, and wire its **Array Size** into
  `num_samples`. If you build a plain `DBL[]` of interleaved values instead, wire
  **Array Size ÷ 2**. Passing twice the real count makes the DLL read past the end of the
  buffer.
- **Sample rate.** Wire the actual acquisition rate in Hz, for example `100e6`.
- **Flags.** `0` for standard operation, `1` for legacy drones (Mavic Pro / Mavic 2), `4` to
  conjugate a spectrally inverted input. The values can be added together.
- **Strings.** Pre-allocate `serial_out` (17 bytes) and `json_out` (4096 bytes) on the diagram,
  for example *Initialize Array* of U8 then *Byte Array To String*, so the DLL has a buffer to
  write into.
- **Integer sizes.** Configure parameters 14–17 with exactly the widths shown. A wider type in
  their place corrupts neighbouring data.

## 3. Read the results

| Return | Meaning | Outputs |
|---|---|---|
| `0` | A frame was decoded and its CRC is valid | Valid: first decoded frame |
| `1` | No DroneID decoded in this buffer | Zeroed |
| `-1` | Invalid argument (NULL array, `num_samples = 0`) | Unchanged |
| `-2` | Detection error | Zeroed |
| `-3` | Out of memory | Zeroed |

- `num_frames_out` counts **detected bursts**. A return of `1` with `num_frames_out > 0`
  means bursts were seen but none decoded: weak signal, clipping, or a non-DroneID burst that
  passed the detector.
- `crc_valid_out` is `1` for a decoded frame, `0` if bursts were seen but none decoded, and
  `-1` if nothing was detected.
- `json_out` holds the full telemetry record, including fields not exposed as scalars such
  as `v_up`, `gps_time`, home point and `uuid`. Wire it to a string indicator or a log file.
- The position fields read zero until the drone has a GPS fix.

## 4. VI structure

Keep acquisition and decoding in separate loops (producer/consumer), so that decoding never
stalls the radio:

```
┌ Acquisition loop ────────────────────────────────────────┐
│  fetch IQ (CDB[])  ──►  Enqueue Element                  │
└──────────────────────────────────────────────────────────┘
┌ Decode loop ─────────────────────────────────────────────┐
│  Dequeue Element ──► Array Size ──► num_samples          │
│        │                                                 │
│        └─────────► droneid_process CLFN (flags, Fs)      │
│                          │                               │
│                 Case: return = 0                         │
│                   True  ► update indicators, log JSON    │
│                   False ► continue                       │
└──────────────────────────────────────────────────────────┘
```

### Buffer length

Each burst lasts about 0.64 ms and the beacon repeats every 640 ms.

| Acquisition style | Buffer length |
|---|---|
| Gap-free streaming on one frequency | 40–300 ms per buffer |
| Hopping between frequencies | ≥ 640 ms per dwell, so each dwell is sure to contain a beacon |

A burst that straddles a buffer boundary is lost. With gap-free streaming the next beacon
arrives 640 ms later. Measured compute time is about 0.5 s for 0.65 s of 100 Msps data on a
4-core laptop CPU, so a dedicated decode loop keeps up in real time. See
[Acquiring IQ data](iq-capture-format.md) for sample rate, tuning and gain.

## 5. Check the DLL outside LabVIEW

If the VI shows nothing, write one acquired buffer to disk as float32 and run it through the
same call path:

```bat
python tools\droneid_process.py buffer.bin 100e6
droneid_bridge_cli buffer.bin 100000000
```

If the DLL decodes the file but the VI does not, the CLFN wiring is wrong (parameter types,
`num_samples`). If the DLL does not decode the file either, the problem is the capture itself
(centre frequency, gain, sample rate). Work through the checklist in
[Acquiring IQ data](iq-capture-format.md#8-checking-a-new-setup).

## 6. Lower-level API

For access to every burst candidate rather than only the first decoded frame, call the two
underlying libraries directly:

1. `detect_droneid_d(iq, n, fs, flags, &results, &count)` in `droneid_detect.dll` returns
   an array of `detection_result_t`. Each element holds a pointer to the burst, resampled to
   15.36 MHz.
2. `locate_droneid(candidate_samples, num_candidate_samples, flags, &telemetry)` in
   `droneid_locate.dll` decodes one burst into a caller-allocated `telemetry_result_t`.
3. `telemetry_to_json(&telemetry, buf, len)` serialises the result.
4. `detect_free_results(results, count)` releases the detection array.

This path needs pointer arithmetic over the C structs (`detection_result_t` is 48 bytes on
x64). It is much easier to drive from a small C wrapper DLL than from LabVIEW directly, which is
exactly what `droneid_bridge` is. For most applications, use `droneid_process`.

## 7. Troubleshooting

| Symptom | Fix |
|---|---|
| "The specified module could not be found" | All four DLLs must be in one folder; use 64-bit LabVIEW; install the [VC++ 2015–2022 x64 redistributable](https://aka.ms/vs/17/release/vc_redist.x64.exe) if `vcruntime140.dll` is missing |
| Return `-1` on every call | `num_samples` is 0 or the array is not wired |
| LabVIEW crashes or hangs in the CLFN | `num_samples` larger than the array (wire Array Size, or ÷ 2 for a `DBL[]`), wrong integer widths on parameters 14–17, or strings not pre-allocated |
| Return `1` with `num_frames_out > 0` | Bursts detected but not decoded: check gain and clipping, try flag `4`, or flag `1` for legacy models |
| Always `num_frames_out = 0` | Wrong `sample_rate`, channel outside the tuned band, or a buffer too short to contain a beacon |
