# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [1.0.0] — 2026-10-04

First public release.

### Added
- `droneid_detect`: STFT burst detection, duration and bandwidth gates, coarse CFO estimation,
  anti-aliased Lanczos-3 resampling to 15.36 MHz; float32, float64 and file entry points.
- `droneid_locate`: OFDM demodulation, integer-subcarrier CFO search, Zadoff-Chu validation,
  QPSK demapping, Gold descrambling, de-rate-matching, LTE turbo decoding, DUML parsing,
  CRC-16 and JSON serialisation.
- `droneid_bridge`: single-call `droneid_process()` with flat outputs for LabVIEW and other
  FFI callers.
- OpenMP-parallel detection; real-time processing of 100 Msps captures on a 4-core CPU.
- Command-line tools: `droneid_scan`, `droneid_candidates`, `droneid_bridge_cli`,
  `droneid_validate`, `turbo_selftest`, and the `droneid_process.py` ctypes driver.
- Unit tests for every library and an optional end-to-end test on a user-supplied capture.
- Prebuilt Windows x64 DLLs (MSVC 2022, Release, bundled FFT, OpenMP).
- Documentation: IQ acquisition requirements, LabVIEW integration, build and deployment,
  architecture.

[1.0.0]: https://github.com/Skeletoskull/droneid-native/releases/tag/v1.0.0
