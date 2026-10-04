# Third-party notices

## DroneSecurity reference receiver

The decoding pipeline is a C reimplementation of the Python proof-of-concept receiver that
accompanies the paper *"Drone Security and the Mysterious Case of DJI's DroneID"* (NDSS 2023),
published at <https://github.com/RUB-SysSec/DroneSecurity> under the GNU Affero General Public
License v3.0. This project is distributed under the same license; see [LICENSE](LICENSE).

## KissFFT

`third_party/kissfft/` is a simplified FFT derived from KissFFT
(<https://github.com/mborgerding/kissfft>), distributed under the BSD 3-Clause License:

```
Copyright (c) 2003-2010, Mark Borgerding. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list of
    conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice, this list of
    conditions and the following disclaimer in the documentation and/or other materials
    provided with the distribution.
  * Neither the author nor the names of any contributors may be used to endorse or promote
    products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Microsoft OpenMP runtime

`bin/windows-x64/vcomp140.dll` is the Microsoft Visual C++ OpenMP runtime, redistributed as
permitted by the Microsoft Visual Studio redistributable-code terms. It is also available in
the [Visual C++ 2015–2022 Redistributable](https://aka.ms/vs/17/release/vc_redist.x64.exe).

## FFTW3 (optional)

When built with `-DUSE_FFTW3=ON`, the libraries link against FFTW3
(<https://www.fftw.org/>), which is licensed under the GNU General Public License v2 or later.
FFTW3 is not included in this repository or in the prebuilt binaries.
