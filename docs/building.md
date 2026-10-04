# Building and deployment

## Requirements

| | Windows | Linux |
|---|---|---|
| Compiler | MSVC 2019 or 2022 (x64) | GCC ≥ 9 or Clang ≥ 10 |
| Build system | CMake ≥ 3.16 | CMake ≥ 3.16 |
| Optional | — | FFTW3 single precision (`libfftw3-dev`) |

The code is C99 and has no required external dependencies. FFTs use the bundled KissFFT-derived
implementation unless FFTW3 is enabled. OpenMP is used automatically when the compiler provides
it, to parallelise the detection STFT.

## Build

### Windows (Visual Studio 2022)

```bat
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

DLLs, tools and test executables are written to `build\bin\Release\`.

### Linux

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

### Options

| Option | Default | Description |
|---|---|---|
| `USE_FFTW3` | `OFF` | Use FFTW3 if found (falls back to the bundled FFT otherwise) |
| `DRONEID_BUILD_TESTS` | `ON` | Build the unit tests and register them with CTest |
| `DRONEID_BUILD_TOOLS` | `ON` | Build the command-line tools in `tools/` |
| `DRONEID_TEST_CAPTURE` | *(empty)* | Path to a real capture; adds the `validate_capture` end-to-end test |
| `DRONEID_TEST_CAPTURE_RATE` | `50000000` | Sample rate of `DRONEID_TEST_CAPTURE`, in Hz |
| `CMAKE_MSVC_RUNTIME_LIBRARY` | *(CMake default, `/MD`)* | Set to `MultiThreaded` to link the C runtime statically |

## Tests

The unit tests use synthetic signals and need no data:

| Test | Covers |
|---|---|
| `test_dsp` | FFT, subcarrier mapping, Zadoff-Chu and Gold sequences, resampling, frequency shift |
| `test_detect` | Burst detection, duration and bandwidth gates, legacy mode, CFO estimation, file API, memory handling |
| `test_locate` | OFDM demodulation, QPSK, de-rate-matching, DUML parsing, CRC-16, JSON round-trip |
| `test_bridge` | `droneid_process` argument checks, output zeroing, no-signal path |
| `turbo_selftest` | Turbo encoder → noisy channel → decoder |

To also run an end-to-end check on one of your own recordings (see
[Acquiring IQ data](iq-capture-format.md)):

```bash
cmake -B build -DDRONEID_TEST_CAPTURE=/path/to/capture.bin -DDRONEID_TEST_CAPTURE_RATE=50000000
cmake --build build --config Release
ctest --test-dir build -C Release -L integration --output-on-failure
```

`validate_capture` passes when at least one frame decodes with a valid CRC and every decoded
frame passes the plausibility and JSON round-trip checks.

## Install

```bash
cmake --install build --config Release --prefix <dir>
```

This installs:

```
<dir>/include/   droneid_detect.h  droneid_locate.h  droneid_bridge.h
<dir>/lib/       import libraries (.lib) on Windows, shared libraries (.so) on Linux
<dir>/bin/       DLLs on Windows, and the command-line tools
<dir>/share/droneid_native/examples/c_example.c
```

Link a C program against the installed SDK:

```bash
# Linux
gcc app.c -I<dir>/include -L<dir>/lib -ldroneid_detect -ldroneid_locate -lm -Wl,-rpath,<dir>/lib

# Windows (Developer Command Prompt)
cl app.c /I <dir>\include <dir>\lib\droneid_detect.lib <dir>\lib\droneid_locate.lib
```

## Deploying on Windows

The runtime set is the three DLLs plus the OpenMP runtime:

```
droneid_bridge.dll  droneid_detect.dll  droneid_locate.dll  vcomp140.dll
```

[`bin/windows-x64/`](../bin/windows-x64/) contains a prebuilt copy (MSVC 2022, x64 Release,
bundled FFT, OpenMP). To deploy your own build, copy the three DLLs from `build\bin\Release\`
together with `vcomp140.dll` from the Visual Studio redistributable folder
(`VC\Redist\MSVC\<version>\x64\Microsoft.VC143.OpenMP\`).

- Keep the files together, either next to the executable or VI, or in a folder on `PATH`.
- The DLLs also import `VCRUNTIME140.dll` and the Universal CRT. Both are present on most
  machines; otherwise install the
  [VC++ 2015–2022 x64 redistributable](https://aka.ms/vs/17/release/vc_redist.x64.exe).
- The DLLs are 64-bit, so the host process (for example LabVIEW) must be 64-bit too.
- `dumpbin /dependents droneid_detect.dll` lists the imports if a load fails.

## Deploying on Linux

```bash
sudo cmake --install build          # installs under /usr/local
sudo ldconfig
```

Alternatively, install to a private prefix and set `LD_LIBRARY_PATH`, or link with an rpath
as shown above.

## FFTW3 licensing

FFTW3 is GPL-licensed. This project is already AGPL-3.0, so linking FFTW3 adds no new
obligations for open-source use. If you need different licensing terms, keep the default
`USE_FFTW3=OFF`; the bundled FFT is BSD-3-Clause.

## Runtime environment variables

| Variable | Effect |
|---|---|
| `DRONEID_PROF=1` | `locate_droneid_file` prints a detection-vs-decode timing breakdown to stderr |
| `DRONEID_CFO_SPAN=<n>` | Half-width of the integer-subcarrier CFO search, in 15 kHz steps (default 10 = ±150 kHz, max 200) |
| `DRONEID_DUMP=<dir>` | For one decoded burst, write the demodulated symbol grid, the candidate samples and metadata into `<dir>` (analysis aid) |

The variables are read once per process. Leave them unset in production.
