#ifndef DSP_INTERNAL_H
#define DSP_INTERNAL_H

/**
 * dsp_internal.h - Internal DSP primitive interface
 *
 * Not exported. Used by both detect and locate libraries via static linking.
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Complex type ─────────────────────────────────────────────────────────── */
/* MSVC does not support C99 float _Complex operators (+, -, *, /).
   We define dsp_complex_t and helper functions that work on both MSVC and GCC. */

#ifdef _MSC_VER
#  include <complex.h>   /* for _Fcomplex, _FCbuild */
   typedef _Fcomplex dsp_complex_t;

   static __inline dsp_complex_t _dsp_cmplx(float re, float im) {
       return _FCbuild(re, im);
   }
   static __inline float _dsp_crealf(_Fcomplex z) { return z._Val[0]; }
   static __inline float _dsp_cimagf(_Fcomplex z) { return z._Val[1]; }
   static __inline float _dsp_cabsf(_Fcomplex z) {
       return sqrtf(z._Val[0]*z._Val[0] + z._Val[1]*z._Val[1]);
   }
   static __inline float _dsp_cargf(_Fcomplex z) {
       return atan2f(z._Val[1], z._Val[0]);
   }
   static __inline _Fcomplex _dsp_conjf(_Fcomplex z) {
       return _FCbuild(z._Val[0], -z._Val[1]);
   }
   static __inline dsp_complex_t _dsp_cadd(dsp_complex_t a, dsp_complex_t b) {
       return _FCbuild(_dsp_crealf(a)+_dsp_crealf(b), _dsp_cimagf(a)+_dsp_cimagf(b));
   }
   static __inline dsp_complex_t _dsp_csub(dsp_complex_t a, dsp_complex_t b) {
       return _FCbuild(_dsp_crealf(a)-_dsp_crealf(b), _dsp_cimagf(a)-_dsp_cimagf(b));
   }
   static __inline dsp_complex_t _dsp_cmul(dsp_complex_t a, dsp_complex_t b) {
       return _FCbuild(
           _dsp_crealf(a)*_dsp_crealf(b) - _dsp_cimagf(a)*_dsp_cimagf(b),
           _dsp_crealf(a)*_dsp_cimagf(b) + _dsp_cimagf(a)*_dsp_crealf(b));
   }
   static __inline dsp_complex_t _dsp_cdiv(dsp_complex_t a, dsp_complex_t b) {
       float d = _dsp_crealf(b)*_dsp_crealf(b) + _dsp_cimagf(b)*_dsp_cimagf(b);
       return _FCbuild(
           (_dsp_crealf(a)*_dsp_crealf(b) + _dsp_cimagf(a)*_dsp_cimagf(b)) / d,
           (_dsp_cimagf(a)*_dsp_crealf(b) - _dsp_crealf(a)*_dsp_cimagf(b)) / d);
   }
   static __inline dsp_complex_t _dsp_cneg(dsp_complex_t a) {
       return _FCbuild(-_dsp_crealf(a), -_dsp_cimagf(a));
   }

   /* Map C99 names to our helpers */
#  define crealf   _dsp_crealf
#  define cimagf   _dsp_cimagf
#  define cabsf    _dsp_cabsf
#  define cargf    _dsp_cargf
#  define conjf    _dsp_conjf
#  define CMPLX(re, im) _dsp_cmplx((float)(re), (float)(im))

#else
   /* GCC / Clang: native C99 complex */
#  include <complex.h>
   typedef float _Complex dsp_complex_t;

#  define _dsp_cmplx(re, im)  ((float _Complex)((re) + (im)*I))
#  define _dsp_crealf(z)      crealf(z)
#  define _dsp_cimagf(z)      cimagf(z)
#  define _dsp_cabsf(z)       cabsf(z)
#  define _dsp_cargf(z)       cargf(z)
#  define _dsp_conjf(z)       conjf(z)
#  define _dsp_cadd(a, b)     ((a) + (b))
#  define _dsp_csub(a, b)     ((a) - (b))
#  define _dsp_cmul(a, b)     ((a) * (b))
#  define _dsp_cdiv(a, b)     ((a) / (b))
#  define _dsp_cneg(a)        (-(a))
#  ifndef CMPLX
#    define CMPLX(re, im)     ((float _Complex)((re) + (im)*I))
#  endif
#endif

/* ── Constants ────────────────────────────────────────────────────────────── */
#define DSP_NFFT           1024
#define DSP_NCARRIERS      601
#define DSP_NCARRIERS_HALF 300
#define DSP_STFT_NFFT      64
#define DSP_WELCH_NFFT     2048
#define DSP_FS_TARGET      15.36e6

/* CP lengths at 15.36 MHz */
#define DSP_CP_STANDARD_0  80
#define DSP_CP_STANDARD_N  72
#define DSP_NSYMBOLS_STD   9
#define DSP_NSYMBOLS_LEG   8

/* ZC symbol indices */
#define DSP_ZC_IDX_STD_0   3
#define DSP_ZC_IDX_STD_1   5
#define DSP_ZC_IDX_LEG_0   2
#define DSP_ZC_IDX_LEG_1   4

/* Gold sequence parameters */
#define DSP_GOLD_SEED      0x12345678u
#define DSP_GOLD_NC        1600

/* ── FFT ──────────────────────────────────────────────────────────────────── */
void dsp_fft_forward(const dsp_complex_t *in, dsp_complex_t *out, uint32_t n);
void dsp_fft_inverse(const dsp_complex_t *in, dsp_complex_t *out, uint32_t n);
void dsp_fft_subcarriers(const dsp_complex_t *in, dsp_complex_t *out);
void dsp_ifft_subcarriers(const dsp_complex_t *in, dsp_complex_t *out);

/* Reusable single-size FFT plan for tight loops (e.g. the STFT envelope, which
   runs millions of fixed-size FFTs). Create once, exec per frame, destroy. Each
   plan owns its own state so exec() is lock-free and reentrant across plans —
   unlike dsp_fft_forward(), which locks + searches the shared plan cache on every
   call. Same numerical result as dsp_fft_forward/inverse for the same size. */
typedef struct dsp_fft_plan dsp_fft_plan;
dsp_fft_plan *dsp_fft_plan_create(uint32_t n, int inverse);
void          dsp_fft_plan_exec(const dsp_fft_plan *p,
                                const dsp_complex_t *in, dsp_complex_t *out);
void          dsp_fft_plan_destroy(dsp_fft_plan *p);

/* ── ZC sequence ──────────────────────────────────────────────────────────── */
void dsp_zc_sequence_time(uint32_t u, uint32_t seq_length, dsp_complex_t *out);
void dsp_zc_sequence_freq(uint32_t root, dsp_complex_t *out);

/* ── Gold sequence ────────────────────────────────────────────────────────── */
void dsp_gold_sequence(uint32_t seed, uint32_t Nc, uint32_t length, uint8_t *out);

/* ── Frequency shift ──────────────────────────────────────────────────────── */
void dsp_fshift(dsp_complex_t *samples, uint32_t n,
                float offset_hz, double sample_rate);

/* ── Resampling ───────────────────────────────────────────────────────────── */
uint32_t dsp_resample(const dsp_complex_t *in, uint32_t n_in,
                      dsp_complex_t *out, uint32_t n_out,
                      double fs_in, double fs_out);
uint32_t dsp_resample_output_length(uint32_t n_in, double fs_in, double fs_out);
void dsp_with_sample_offset(const dsp_complex_t *in, uint32_t n,
                             float offset, dsp_complex_t *out);

/* ── Correlation ──────────────────────────────────────────────────────────── */
void dsp_correlate(const dsp_complex_t *x, const dsp_complex_t *y,
                   dsp_complex_t *out, uint32_t n);
uint32_t dsp_argmax_abs(const dsp_complex_t *x, uint32_t n);

/* ── Welch PSD ────────────────────────────────────────────────────────────── */
bool dsp_welch_psd(const dsp_complex_t *samples, uint32_t n,
                   double sample_rate,
                   float *psd_out, float *freqs_out);
bool dsp_estimate_cfo(const dsp_complex_t *samples, uint32_t n,
                      double sample_rate,
                      float bw_min_hz, float bw_max_hz,
                      float *cfo_hz_out);

/* ── STFT ─────────────────────────────────────────────────────────────────── */
void dsp_stft_power(const dsp_complex_t *samples, uint32_t n,
                    float *power_out, uint32_t *n_frames);
float dsp_stft_mean_magnitude(const dsp_complex_t *samples, uint32_t n);

#endif /* DSP_INTERNAL_H */
