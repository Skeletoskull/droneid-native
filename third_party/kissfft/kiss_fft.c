/*
 * KissFFT — minimal single-precision complex FFT implementation
 * BSD-3-Clause License
 * https://github.com/mborgerding/kissfft
 *
 * Simplified implementation for use as FFTW3 fallback.
 * Supports arbitrary sizes via Cooley-Tukey mixed-radix decomposition.
 */

#include "kiss_fft.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Internal state ──────────────────────────────────────────────────────── */

typedef struct kiss_fft_state {
    int nfft;
    int inverse;
    int *factors;
    kiss_fft_cpx *twiddles;
} kiss_fft_state;

/* ── Twiddle factor computation ──────────────────────────────────────────── */

static kiss_fft_cpx kf_cexp(double phase) {
    kiss_fft_cpx c;
    c.r = (float)cos(phase);
    c.i = (float)sin(phase);
    return c;
}

/* ── Radix-2 butterfly ───────────────────────────────────────────────────── */

static void kf_bfly2(kiss_fft_cpx *Fout, const size_t fstride,
                     const kiss_fft_cfg st, int m) {
    kiss_fft_cpx *Fout2 = Fout + m;
    const kiss_fft_cpx *tw1 = st->twiddles;
    kiss_fft_cpx t;
    do {
        t.r = Fout2->r * tw1->r - Fout2->i * tw1->i;
        t.i = Fout2->r * tw1->i + Fout2->i * tw1->r;
        tw1 += fstride;
        Fout2->r = Fout->r - t.r;
        Fout2->i = Fout->i - t.i;
        Fout->r  += t.r;
        Fout->i  += t.i;
        ++Fout2;
        ++Fout;
    } while (--m);
}

/* ── Radix-4 butterfly ───────────────────────────────────────────────────── */

static void kf_bfly4(kiss_fft_cpx *Fout, const size_t fstride,
                     const kiss_fft_cfg st, const size_t m) {
    kiss_fft_cpx *Fout0, *Fout1, *Fout2, *Fout3;
    size_t k;
    const kiss_fft_cpx *tw1, *tw2, *tw3;
    kiss_fft_cpx scratch[6];
    size_t N = m;

    Fout0 = Fout;
    Fout1 = Fout0 + N;
    Fout2 = Fout0 + 2 * N;
    Fout3 = Fout0 + 3 * N;
    tw3 = tw2 = tw1 = st->twiddles;

    for (k = N; k--; ) {
        scratch[0].r = Fout1->r * tw1->r - Fout1->i * tw1->i;
        scratch[0].i = Fout1->r * tw1->i + Fout1->i * tw1->r;
        tw1 += fstride;

        scratch[1].r = Fout2->r * tw2->r - Fout2->i * tw2->i;
        scratch[1].i = Fout2->r * tw2->i + Fout2->i * tw2->r;
        tw2 += fstride * 2;

        scratch[2].r = Fout3->r * tw3->r - Fout3->i * tw3->i;
        scratch[2].i = Fout3->r * tw3->i + Fout3->i * tw3->r;
        tw3 += fstride * 3;

        scratch[5].r = Fout0->r - scratch[1].r;
        scratch[5].i = Fout0->i - scratch[1].i;
        Fout0->r += scratch[1].r;
        Fout0->i += scratch[1].i;

        scratch[3].r = scratch[0].r + scratch[2].r;
        scratch[3].i = scratch[0].i + scratch[2].i;
        scratch[4].r = scratch[0].r - scratch[2].r;
        scratch[4].i = scratch[0].i - scratch[2].i;

        Fout2->r = Fout0->r - scratch[3].r;
        Fout2->i = Fout0->i - scratch[3].i;
        Fout0->r += scratch[3].r;
        Fout0->i += scratch[3].i;

        if (st->inverse) {
            Fout1->r = scratch[5].r - scratch[4].i;
            Fout1->i = scratch[5].i + scratch[4].r;
            Fout3->r = scratch[5].r + scratch[4].i;
            Fout3->i = scratch[5].i - scratch[4].r;
        } else {
            Fout1->r = scratch[5].r + scratch[4].i;
            Fout1->i = scratch[5].i - scratch[4].r;
            Fout3->r = scratch[5].r - scratch[4].i;
            Fout3->i = scratch[5].i + scratch[4].r;
        }
        ++Fout0; ++Fout1; ++Fout2; ++Fout3;
    }
}

/* ── Generic butterfly ───────────────────────────────────────────────────── */

static void kf_bfly_generic(kiss_fft_cpx *Fout, const size_t fstride,
                             const kiss_fft_cfg st, int m, int p) {
    int u, k, q1, q;
    const kiss_fft_cpx *twiddles = st->twiddles;
    kiss_fft_cpx t;
    int Norig = st->nfft;
    kiss_fft_cpx *scratch = (kiss_fft_cpx *)malloc(sizeof(kiss_fft_cpx) * (size_t)p);
    if (!scratch) return;

    for (u = 0; u < m; ++u) {
        k = u;
        for (q1 = 0; q1 < p; ++q1) {
            scratch[q1] = Fout[k];
            k += m;
        }
        k = u;
        for (q1 = 0; q1 < p; ++q1) {
            int twidx = 0;
            Fout[k] = scratch[0];
            for (q = 1; q < p; ++q) {
                twidx += (int)(fstride * k);
                if (twidx >= Norig) twidx -= Norig;
                t.r = scratch[q].r * twiddles[twidx].r - scratch[q].i * twiddles[twidx].i;
                t.i = scratch[q].r * twiddles[twidx].i + scratch[q].i * twiddles[twidx].r;
                Fout[k].r += t.r;
                Fout[k].i += t.i;
            }
            k += m;
        }
    }
    free(scratch);
}

/* ── Recursive FFT work ──────────────────────────────────────────────────── */

static void kf_work(kiss_fft_cpx *Fout, const kiss_fft_cpx *f,
                    const size_t fstride, int in_stride,
                    int *factors, const kiss_fft_cfg st) {
    kiss_fft_cpx *Fout_beg = Fout;
    const int p = *factors++;
    const int m = *factors++;
    const kiss_fft_cpx *Fout_end = Fout + p * m;

    if (m == 1) {
        do {
            *Fout = *f;
            f += fstride * in_stride;
        } while (++Fout != Fout_end);
    } else {
        do {
            kf_work(Fout, f, fstride * p, in_stride, factors, st);
            f += fstride * in_stride;
        } while ((Fout += m) != Fout_end);
    }

    Fout = Fout_beg;
    switch (p) {
        case 2:  kf_bfly2(Fout, fstride, st, m); break;
        case 4:  kf_bfly4(Fout, fstride, st, (size_t)m); break;
        default: kf_bfly_generic(Fout, fstride, st, m, p); break;
    }
}

/* ── Factor decomposition ────────────────────────────────────────────────── */

static void kf_factor(int n, int *facbuf) {
    int p = 4;
    double floor_sqrt = floor(sqrt((double)n));

    do {
        while (n % p) {
            switch (p) {
                case 4:  p = 2; break;
                case 2:  p = 3; break;
                default: p += 2; break;
            }
            if (p > floor_sqrt) p = n;
        }
        n /= p;
        *facbuf++ = p;
        *facbuf++ = n;
    } while (n > 1);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

kiss_fft_cfg kiss_fft_alloc(int nfft, int inverse_fft, void *mem, size_t *lenmem) {
    kiss_fft_cfg st = NULL;
    size_t memneeded = sizeof(struct kiss_fft_state)
                     + sizeof(kiss_fft_cpx) * (size_t)nfft   /* twiddles */
                     + sizeof(int) * (size_t)(2 * 32);        /* factors */

    if (lenmem == NULL) {
        st = (kiss_fft_cfg)malloc(memneeded);
    } else {
        if (mem != NULL && *lenmem >= memneeded)
            st = (kiss_fft_cfg)mem;
        *lenmem = memneeded;
    }
    if (!st) return NULL;

    st->nfft    = nfft;
    st->inverse = inverse_fft;
    st->twiddles = (kiss_fft_cpx *)((char *)st + sizeof(struct kiss_fft_state));
    st->factors  = (int *)((char *)st->twiddles + sizeof(kiss_fft_cpx) * (size_t)nfft);

    for (int i = 0; i < nfft; ++i) {
        double phase = -2.0 * M_PI * i / nfft;
        if (inverse_fft) phase = -phase;
        st->twiddles[i] = kf_cexp(phase);
    }
    kf_factor(nfft, st->factors);
    return st;
}

void kiss_fft(kiss_fft_cfg cfg, const kiss_fft_cpx *fin, kiss_fft_cpx *fout) {
    kf_work(fout, fin, 1, 1, cfg->factors, cfg);
}

void kiss_fft_free(kiss_fft_cfg cfg) {
    free(cfg);
}

void kiss_fft_cleanup(void) {
    /* nothing to do in this implementation */
}
