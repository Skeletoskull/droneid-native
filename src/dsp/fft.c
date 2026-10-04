/**
 * fft.c — FFT backend wrapper
 *
 * Wraps either FFTW3 (single-precision) or KissFFT behind a unified interface.
 * Backend is selected at compile time via USE_FFTW3 preprocessor define.
 *
 * Thread safety: FFTW plan cache is protected by a mutex for the entire
 * duration of plan lookup + execution to avoid use-after-eviction races.
 *
 * NumPy compatibility:
 *   dsp_fft_forward()  matches np.fft.fft()   (no normalization)
 *   dsp_fft_inverse()  matches np.fft.ifft()  (divides by N)
 */

#include "dsp_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef USE_FFTW3
#  include <fftw3.h>
#else
#  include "../../third_party/kissfft/kiss_fft.h"
#endif

/* ── Platform mutex ───────────────────────────────────────────────────────── */
#ifdef _WIN32
#  include <windows.h>

   /* Use a statically initialized CRITICAL_SECTION via InitOnceExecuteOnce
      to avoid the double-checked locking race on Windows. */
   static CRITICAL_SECTION g_fft_lock;
   static INIT_ONCE        g_fft_once = INIT_ONCE_STATIC_INIT;

   static BOOL CALLBACK fft_lock_init_once(PINIT_ONCE InitOnce,
                                            PVOID Parameter,
                                            PVOID *Context) {
       (void)InitOnce; (void)Parameter; (void)Context;
       InitializeCriticalSection(&g_fft_lock);
       return TRUE;
   }

   static void fft_lock(void) {
       InitOnceExecuteOnce(&g_fft_once, fft_lock_init_once, NULL, NULL);
       EnterCriticalSection(&g_fft_lock);
   }
   static void fft_unlock(void) { LeaveCriticalSection(&g_fft_lock); }

#else
#  include <pthread.h>
   static pthread_mutex_t g_fft_lock = PTHREAD_MUTEX_INITIALIZER;
   static void fft_lock(void)   { pthread_mutex_lock(&g_fft_lock); }
   static void fft_unlock(void) { pthread_mutex_unlock(&g_fft_lock); }
#endif

/* ── Plan cache (up to 8 distinct sizes) ─────────────────────────────────── */
#define FFT_PLAN_CACHE_SIZE 8

typedef struct {
    uint32_t n;
    int      inverse;
#ifdef USE_FFTW3
    fftwf_plan    plan;
    fftwf_complex *in_buf;
    fftwf_complex *out_buf;
#else
    kiss_fft_cfg cfg;
#endif
} fft_plan_entry_t;

static fft_plan_entry_t g_plan_cache[FFT_PLAN_CACHE_SIZE];
static int g_plan_count = 0;

/* ── FFTW3 backend ────────────────────────────────────────────────────────── */
#ifdef USE_FFTW3

/**
 * Look up or create an FFTW plan.
 * Must be called with g_fft_lock already held.
 * Returns a pointer into the cache (valid while lock is held).
 */
static fft_plan_entry_t *get_fftw_plan_locked(uint32_t n, int inverse) {
    /* Search cache */
    for (int i = 0; i < g_plan_count; i++) {
        if (g_plan_cache[i].n == n && g_plan_cache[i].inverse == inverse) {
            return &g_plan_cache[i];
        }
    }
    /* Evict oldest entry if cache is full */
    if (g_plan_count >= FFT_PLAN_CACHE_SIZE) {
        fftwf_destroy_plan(g_plan_cache[0].plan);
        fftwf_free(g_plan_cache[0].in_buf);
        fftwf_free(g_plan_cache[0].out_buf);
        memmove(&g_plan_cache[0], &g_plan_cache[1],
                sizeof(fft_plan_entry_t) * (FFT_PLAN_CACHE_SIZE - 1));
        g_plan_count--;
    }
    fft_plan_entry_t *e = &g_plan_cache[g_plan_count++];
    e->n       = n;
    e->inverse = inverse;
    e->in_buf  = fftwf_alloc_complex(n);
    e->out_buf = fftwf_alloc_complex(n);
    e->plan    = fftwf_plan_dft_1d((int)n, e->in_buf, e->out_buf,
                                   inverse ? FFTW_BACKWARD : FFTW_FORWARD,
                                   FFTW_ESTIMATE);
    return e;
}

void dsp_fft_forward(const dsp_complex_t *in, dsp_complex_t *out, uint32_t n) {
    fft_lock();
    fft_plan_entry_t *e = get_fftw_plan_locked(n, 0);
    /* Copy input, execute, copy output — all under the lock to protect
       the shared in_buf/out_buf buffers from concurrent access. */
    memcpy(e->in_buf, in, sizeof(fftwf_complex) * n);
    fftwf_execute(e->plan);
    memcpy(out, e->out_buf, sizeof(fftwf_complex) * n);
    fft_unlock();
}

void dsp_fft_inverse(const dsp_complex_t *in, dsp_complex_t *out, uint32_t n) {
    fft_lock();
    fft_plan_entry_t *e = get_fftw_plan_locked(n, 1);
    memcpy(e->in_buf, in, sizeof(fftwf_complex) * n);
    fftwf_execute(e->plan);
    /* FFTW unnormalized IFFT: divide by N to match np.fft.ifft semantics */
    float scale = 1.0f / (float)n;
    fftwf_complex *ob = e->out_buf;
    for (uint32_t k = 0; k < n; k++) {
        ((float *)&out[k])[0] = ob[k][0] * scale;
        ((float *)&out[k])[1] = ob[k][1] * scale;
    }
    fft_unlock();
}

#else /* KissFFT backend */

/**
 * Look up or create a KissFFT config.
 * Must be called with g_fft_lock already held.
 * Returns a pointer into the cache (valid while lock is held).
 */
static fft_plan_entry_t *get_kiss_plan_locked(uint32_t n, int inverse) {
    for (int i = 0; i < g_plan_count; i++) {
        if (g_plan_cache[i].n == n && g_plan_cache[i].inverse == inverse) {
            return &g_plan_cache[i];
        }
    }
    if (g_plan_count >= FFT_PLAN_CACHE_SIZE) {
        kiss_fft_free(g_plan_cache[0].cfg);
        memmove(&g_plan_cache[0], &g_plan_cache[1],
                sizeof(fft_plan_entry_t) * (FFT_PLAN_CACHE_SIZE - 1));
        g_plan_count--;
    }
    fft_plan_entry_t *e = &g_plan_cache[g_plan_count++];
    e->n       = n;
    e->inverse = inverse;
    e->cfg     = kiss_fft_alloc((int)n, inverse, NULL, NULL);
    return e;
}

void dsp_fft_forward(const dsp_complex_t *in, dsp_complex_t *out, uint32_t n) {
    fft_lock();
    fft_plan_entry_t *e = get_kiss_plan_locked(n, 0);
    kiss_fft(e->cfg, (const kiss_fft_cpx *)in, (kiss_fft_cpx *)out);
    fft_unlock();
}

void dsp_fft_inverse(const dsp_complex_t *in, dsp_complex_t *out, uint32_t n) {
    fft_lock();
    fft_plan_entry_t *e = get_kiss_plan_locked(n, 1);
    kiss_fft(e->cfg, (const kiss_fft_cpx *)in, (kiss_fft_cpx *)out);
    /* KissFFT does not normalize the inverse FFT.
       Divide by N to match np.fft.ifft semantics. */
    float scale = 1.0f / (float)n;
    kiss_fft_cpx *o = (kiss_fft_cpx *)out;
    for (uint32_t k = 0; k < n; k++) {
        o[k].r *= scale;
        o[k].i *= scale;
    }
    fft_unlock();
}

#endif /* USE_FFTW3 */

/* ── Reusable single-size plan ────────────────────────────────────────────── */
/* Owns its own FFT state so exec() needs no lock — used by the STFT envelope to
   avoid taking the global lock + cache search on every one of millions of FFTs. */
struct dsp_fft_plan {
    uint32_t n;
    int      inverse;
#ifdef USE_FFTW3
    fftwf_plan plan;
#else
    kiss_fft_cfg cfg;
#endif
};

dsp_fft_plan *dsp_fft_plan_create(uint32_t n, int inverse) {
    dsp_fft_plan *p = (dsp_fft_plan *)malloc(sizeof(*p));
    if (!p) return NULL;
    p->n = n;
    p->inverse = inverse;
    fft_lock();                 /* plan creation is not thread-safe (FFTW) */
#ifdef USE_FFTW3
    {
        fftwf_complex *tin  = fftwf_alloc_complex(n);
        fftwf_complex *tout = fftwf_alloc_complex(n);
        p->plan = (tin && tout)
            ? fftwf_plan_dft_1d((int)n, tin, tout,
                                inverse ? FFTW_BACKWARD : FFTW_FORWARD, FFTW_ESTIMATE)
            : NULL;
        if (tin)  fftwf_free(tin);
        if (tout) fftwf_free(tout);
        if (!p->plan) { fft_unlock(); free(p); return NULL; }
    }
#else
    p->cfg = kiss_fft_alloc((int)n, inverse, NULL, NULL);
    if (!p->cfg) { fft_unlock(); free(p); return NULL; }
#endif
    fft_unlock();
    return p;
}

void dsp_fft_plan_exec(const dsp_fft_plan *p,
                       const dsp_complex_t *in, dsp_complex_t *out) {
#ifdef USE_FFTW3
    /* array-execute form: thread-safe to run a plan on caller-supplied buffers */
    fftwf_execute_dft(p->plan, (fftwf_complex *)(void *)in, (fftwf_complex *)out);
    if (p->inverse) {
        float scale = 1.0f / (float)p->n;
        uint32_t k;
        for (k = 0; k < p->n; k++) {
            ((float *)&out[k])[0] *= scale;
            ((float *)&out[k])[1] *= scale;
        }
    }
#else
    kiss_fft(p->cfg, (const kiss_fft_cpx *)in, (kiss_fft_cpx *)out);
    if (p->inverse) {
        float scale = 1.0f / (float)p->n;
        kiss_fft_cpx *o = (kiss_fft_cpx *)out;
        uint32_t k;
        for (k = 0; k < p->n; k++) { o[k].r *= scale; o[k].i *= scale; }
    }
#endif
}

void dsp_fft_plan_destroy(dsp_fft_plan *p) {
    if (!p) return;
    fft_lock();
#ifdef USE_FFTW3
    if (p->plan) fftwf_destroy_plan(p->plan);
#else
    if (p->cfg) kiss_fft_free(p->cfg);
#endif
    fft_unlock();
    free(p);
}

/* ── Subcarrier reorder (mirrors helpers.py tfft / itfft) ────────────────── */

void dsp_fft_subcarriers(const dsp_complex_t *in, dsp_complex_t *out) {
    /* Heap-allocate to avoid 8 KB stack frame in Release/MSVC */
    dsp_complex_t *full = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NFFT);
    if (!full) { memset(out, 0, sizeof(dsp_complex_t) * DSP_NCARRIERS); return; }
    dsp_fft_forward(in, full, DSP_NFFT);

    /* Reorder: negative freqs → DC → positive freqs
     *   Python: new_fft = np.concatenate((fft[-300:], fft[:301]))
     *   out[0..299]   = full[724..1023]   (negative frequencies: last 300)
     *   out[300..600] = full[0..300]       (DC + positive frequencies: first 301)
     */
    memcpy(out, full + DSP_NFFT - DSP_NCARRIERS_HALF,
           sizeof(dsp_complex_t) * DSP_NCARRIERS_HALF);
    memcpy(out + DSP_NCARRIERS_HALF, full,
           sizeof(dsp_complex_t) * (DSP_NCARRIERS_HALF + 1));
    free(full);
}

void dsp_ifft_subcarriers(const dsp_complex_t *in, dsp_complex_t *out) {
    /* Heap-allocate to avoid 8 KB stack frame in Release/MSVC */
    dsp_complex_t *full = (dsp_complex_t *)calloc(DSP_NFFT, sizeof(dsp_complex_t));
    if (!full) { memset(out, 0, sizeof(dsp_complex_t) * DSP_NFFT); return; }

    /* Reverse reorder: [-300..0..+300] → full FFT bins
     *   Python: c_full[-300:] = c[:300]     → full[724:1024] = in[0:300]
     *           c_full[:301]  = c[300:]     → full[0:301]    = in[300:601]
     */
    memcpy(full + DSP_NFFT - DSP_NCARRIERS_HALF, in,
           sizeof(dsp_complex_t) * DSP_NCARRIERS_HALF);
    memcpy(full, in + DSP_NCARRIERS_HALF,
           sizeof(dsp_complex_t) * (DSP_NCARRIERS_HALF + 1));

    /* dsp_fft_inverse already applies the 1/N normalization */
    dsp_fft_inverse(full, out, DSP_NFFT);
    free(full);
}
