#!/usr/bin/env python3
"""Run a capture through droneid_process() in droneid_bridge.dll via ctypes.

This takes the same call path LabVIEW uses (float64 interleaved I/Q in, flat
scalars out). It is a quick way to test the prebuilt DLLs without a C compiler.

Usage:
    python droneid_process.py <iq_file> <sample_rate_hz> [--flags N] [--dll-dir DIR]

    iq_file   raw interleaved little-endian float32 I/Q
    --flags   shared DETECT_/LOCATE_ flag bitfield (default 0):
              0x1 legacy frame format, 0x2 assume standard ZC roots,
              0x4 conjugate input (undo a spectrally inverted recording)
    --dll-dir folder holding droneid_bridge.dll, droneid_detect.dll,
              droneid_locate.dll and vcomp140.dll (default: ../bin/windows-x64)

Requires numpy.
"""
import argparse
import ctypes as C
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DLL_DIR = os.path.join(HERE, os.pardir, "bin", "windows-x64")
STATUS = {0: "decoded", 1: "no frame", -1: "invalid argument",
          -2: "detection error", -3: "allocation failure"}


def load_bridge(dll_dir):
    dll_dir = os.path.abspath(dll_dir)
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(dll_dir)  # resolve detect/locate/vcomp140 deps
    lib = C.CDLL(os.path.join(dll_dir, "droneid_bridge.dll"))
    fn = lib.droneid_process
    fn.restype = C.c_int32
    fn.argtypes = [
        C.POINTER(C.c_double), C.c_uint32, C.c_double, C.c_uint32,
        C.POINTER(C.c_int32), C.POINTER(C.c_int32), C.c_char_p,
        C.POINTER(C.c_double), C.POINTER(C.c_double), C.POINTER(C.c_double),
        C.POINTER(C.c_double), C.POINTER(C.c_double), C.POINTER(C.c_double),
        C.POINTER(C.c_int16), C.POINTER(C.c_int16), C.POINTER(C.c_uint16),
        C.POINTER(C.c_uint8), C.c_char_p, C.c_uint32,
    ]
    return fn


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("iq_file")
    ap.add_argument("sample_rate", type=float)
    ap.add_argument("--flags", type=lambda s: int(s, 0), default=0)
    ap.add_argument("--dll-dir", default=DEFAULT_DLL_DIR)
    args = ap.parse_args()

    process = load_bridge(args.dll_dir)
    iq = np.ascontiguousarray(np.fromfile(args.iq_file, dtype="<f4"), dtype=np.float64)
    n_samples = len(iq) // 2

    num_frames, crc = C.c_int32(), C.c_int32()
    serial, js = C.create_string_buffer(17), C.create_string_buffer(4096)
    lat, lon, alt, height = C.c_double(), C.c_double(), C.c_double(), C.c_double()
    app_lat, app_lon = C.c_double(), C.c_double()
    v_n, v_e, seq, dev = C.c_int16(), C.c_int16(), C.c_uint16(), C.c_uint8()

    rc = process(iq.ctypes.data_as(C.POINTER(C.c_double)), n_samples,
                 args.sample_rate, args.flags,
                 C.byref(num_frames), C.byref(crc), serial,
                 C.byref(lat), C.byref(lon), C.byref(alt), C.byref(height),
                 C.byref(app_lat), C.byref(app_lon),
                 C.byref(v_n), C.byref(v_e), C.byref(seq), C.byref(dev),
                 js, len(js))

    print(f"{os.path.basename(args.iq_file)}: rc={rc} ({STATUS.get(rc, 'unknown')}), "
          f"frames={num_frames.value}, crc={crc.value}")
    if rc == 0:
        print(f"  serial={serial.value.decode(errors='replace')}  "
              f"lat={lat.value:.6f}  lon={lon.value:.6f}  alt={alt.value:.1f} m")
    return 0 if rc == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
