/**
 * turbo_decode.c — Soft-decision + LTE turbo FEC decode for DJI DroneID.
 *
 * Port of a validated Python reference decoder. Used as a fallback when the
 * systematic-only hard decision fails the CRC; it recovers real over-the-air
 * frames that the hard decoder cannot.
 *
 * Parameters: K=1408 info bits, rate-1/3 LTE turbo, QPP f1=43 f2=88,
 * constituent RSC g0=1+D^2+D^3 (feedback), g1=1+D+D^3 (parity), max-log-MAP.
 * Standard (9-symbol) frames only; legacy falls back to the hard path.
 *
 * MSVC-friendly: works on real/imag floats (no C99 complex operators).
 */

#include "locate_internal.h"
#include "turbo_decode.h"
#include "turbo_deratematch.h"   /* RM_PERM_TURBO[32] */
#include <math.h>
#include <stdlib.h>

#define TK    1408
#define TF1   43
#define TF2   88
#define TD    (TK + 4)      /* 1412 = per-stream length (info + 4 tail) */
#define TR    45
#define TC    32
#define TKPI  (TR * TC)     /* 1440 */
#define TND   (TKPI - TD)   /* 28 nulls per stream */
#define TKW   (3 * TKPI)    /* 4320 circular-buffer length */
#define TE    7200          /* transmitted coded bits (6 data symbols x 1200) */
#define NEGF  (-1e9f)

/* ── QPP interleaver and its inverse ─────────────────────────────────────── */
static void qpp_build(int *pi, int *ipi) {
    int i;
    for (i = 0; i < TK; i++)
        pi[i] = (int)(((long long)TF1 * i + (long long)TF2 * i * i) % TK);
    for (i = 0; i < TK; i++) ipi[pi[i]] = i;
}

/* ── LTE constituent RSC trellis ─────────────────────────────────────────── */
static void trellis_build(int ns[8][2], int op[8][2]) {
    int s, u;
    for (s = 0; s < 8; s++) {
        int r1 = (s >> 2) & 1, r2 = (s >> 1) & 1, r3 = s & 1;
        for (u = 0; u < 2; u++) {
            int a = u ^ r2 ^ r3;       /* feedback  (g0 = 1 + D^2 + D^3) */
            int z = a ^ r1 ^ r3;       /* parity    (g1 = 1 + D + D^3)   */
            ns[s][u] = (a << 2) | (r1 << 1) | r2;
            op[s][u] = z;
        }
    }
}

/* ── max-log-MAP component decoder ───────────────────────────────────────── */
static void bcjr(const float *Lsys, const float *Lpar, const float *La,
                 float *Le, int N, int ns[8][2], int op[8][2]) {
    float *al = (float *)malloc(sizeof(float) * (size_t)(N + 1) * 8);
    float *be = (float *)malloc(sizeof(float) * (size_t)(N + 1) * 8);
    int k, s, u;
    if (!al || !be) { free(al); free(be); return; }
#define AL(kk,ss) al[(size_t)(kk)*8 + (ss)]
#define BE(kk,ss) be[(size_t)(kk)*8 + (ss)]
    /* forward */
    for (s = 0; s < 8; s++) AL(0, s) = (s == 0) ? 0.0f : NEGF;
    for (k = 1; k <= N; k++) for (s = 0; s < 8; s++) AL(k, s) = NEGF;
    for (k = 1; k <= N; k++) {
        float ls = La[k-1] + Lsys[k-1], lp = Lpar[k-1];
        for (s = 0; s < 8; s++) {
            float a0 = AL(k-1, s);
            if (a0 <= NEGF) continue;
            for (u = 0; u < 2; u++) {
                float g = 0.5f * (ls * (float)(1 - 2*u) + lp * (float)(1 - 2*op[s][u]));
                float v = a0 + g;
                int n = ns[s][u];
                if (v > AL(k, n)) AL(k, n) = v;
            }
        }
    }
    /* backward (terminated to state 0) */
    for (s = 0; s < 8; s++) BE(N, s) = (s == 0) ? 0.0f : NEGF;
    for (k = N-1; k >= 0; k--) for (s = 0; s < 8; s++) BE(k, s) = NEGF;
    for (k = N-1; k >= 0; k--) {
        float ls = La[k] + Lsys[k], lp = Lpar[k];
        for (s = 0; s < 8; s++) {
            for (u = 0; u < 2; u++) {
                float g = 0.5f * (ls * (float)(1 - 2*u) + lp * (float)(1 - 2*op[s][u]));
                float v = BE(k+1, ns[s][u]) + g;
                if (v > BE(k, s)) BE(k, s) = v;
            }
        }
    }
    /* per-bit extrinsic LLR */
    for (k = 0; k < N; k++) {
        float ls = La[k] + Lsys[k], lp = Lpar[k];
        float m0 = NEGF, m1 = NEGF;
        for (s = 0; s < 8; s++) {
            float a0 = AL(k, s);
            if (a0 <= NEGF) continue;
            for (u = 0; u < 2; u++) {
                float g = 0.5f * (ls * (float)(1 - 2*u) + lp * (float)(1 - 2*op[s][u]));
                float m = a0 + g + BE(k+1, ns[s][u]);
                if (u == 0) { if (m > m0) m0 = m; }
                else        { if (m > m1) m1 = m; }
            }
        }
        Le[k] = (m0 - m1) - ls;
    }
#undef AL
#undef BE
    free(al); free(be);
}

/* ── turbo decode loop (LTE tail demux + 8 iterations) ───────────────────── */
/* d0,d1,d2: per-stream LLRs (length TD). Writes TK hard bits to bits_out.   */
static int turbo_decode_lte(const float *d0, const float *d1, const float *d2,
                            uint8_t *bits_out) {
    int ns[8][2], op[8][2];
    int N = TK + 3, i, it;
    int *pi  = (int *)malloc(sizeof(int) * TK);
    int *ipi = (int *)malloc(sizeof(int) * TK);
    float *Lsys1 = (float *)malloc(sizeof(float) * N);
    float *Lpar1 = (float *)malloc(sizeof(float) * N);
    float *Lpar2 = (float *)malloc(sizeof(float) * N);
    float *Lsys2 = (float *)malloc(sizeof(float) * N);
    float *La    = (float *)malloc(sizeof(float) * N);
    float *La2   = (float *)malloc(sizeof(float) * N);
    float *Le1   = (float *)malloc(sizeof(float) * N);
    float *Le2   = (float *)malloc(sizeof(float) * N);
    float s2t[3];
    if (!pi||!ipi||!Lsys1||!Lpar1||!Lpar2||!Lsys2||!La||!La2||!Le1||!Le2) {
        free(pi);free(ipi);free(Lsys1);free(Lpar1);free(Lpar2);free(Lsys2);
        free(La);free(La2);free(Le1);free(Le2); return -1;
    }
    trellis_build(ns, op);
    qpp_build(pi, ipi);
    for (i = 0; i < TK; i++) { Lsys1[i] = d0[i]; Lpar1[i] = d1[i]; Lpar2[i] = d2[i]; }
    /* LTE tail demux (3GPP 36.212 5.1.3.2.2) */
    Lsys1[TK]=d0[TK];   Lsys1[TK+1]=d2[TK];   Lsys1[TK+2]=d1[TK+1];   /* enc1 sys  x  */
    Lpar1[TK]=d1[TK];   Lpar1[TK+1]=d0[TK+1]; Lpar1[TK+2]=d2[TK+1];   /* enc1 par  z  */
    Lpar2[TK]=d1[TK+2]; Lpar2[TK+1]=d0[TK+3]; Lpar2[TK+2]=d2[TK+3];   /* enc2 par  z' */
    s2t[0]=d0[TK+2]; s2t[1]=d2[TK+2]; s2t[2]=d1[TK+3];                /* enc2 sys  x' */
    for (i = 0; i < N; i++) La[i] = 0.0f;
    for (it = 0; it < 8; it++) {
        bcjr(Lsys1, Lpar1, La, Le1, N, ns, op);
        for (i = 0; i < TK; i++) { Lsys2[i] = Lsys1[pi[i]]; La2[i] = Le1[pi[i]]; }
        Lsys2[TK]=s2t[0]; Lsys2[TK+1]=s2t[1]; Lsys2[TK+2]=s2t[2];
        La2[TK]=0.0f; La2[TK+1]=0.0f; La2[TK+2]=0.0f;
        bcjr(Lsys2, Lpar2, La2, Le2, N, ns, op);
        for (i = 0; i < TK; i++) La[i] = Le2[ipi[i]];
        La[TK]=0.0f; La[TK+1]=0.0f; La[TK+2]=0.0f;
    }
    bcjr(Lsys1, Lpar1, La, Le1, N, ns, op);
    for (i = 0; i < TK; i++) {
        float post = Lsys1[i] + La[i] + Le1[i];
        bits_out[i] = (post < 0.0f) ? 1 : 0;
    }
    free(pi);free(ipi);free(Lsys1);free(Lpar1);free(Lpar2);free(Lsys2);
    free(La);free(La2);free(Le1);free(Le2);
    return 0;
}

/* ── LTE rate-dematch map: transmitted bit -> (stream, d-index) ──────────── */
static int rm0(int k) { return (k % TR) * TC + RM_PERM_TURBO[k / TR]; }
static int rm2(int k) { return (RM_PERM_TURBO[k / TR] + TC * (k % TR) + 1) % TKPI; }

static int build_map(int *src_stream, int *src_didx) {
    int *W_S = (int *)malloc(sizeof(int) * TKW);
    int *W_D = (int *)malloc(sizeof(int) * TKW);
    int *NN  = (int *)malloc(sizeof(int) * TKW);
    int w, nnl = 0, pos_v0 = -1, START, j;
    if (!W_S || !W_D || !NN) { free(W_S); free(W_D); free(NN); return -1; }
    for (w = 0; w < TKW; w++) {
        int s, rm;
        if (w < TKPI) { s = 0; rm = rm0(w); }
        else {
            int m = w - TKPI;
            if (m % 2 == 0) { s = 1; rm = rm0(m / 2); }
            else            { s = 2; rm = rm2((m - 1) / 2); }
        }
        W_S[w] = s;
        W_D[w] = (rm >= TND) ? (rm - TND) : -1;
    }
    for (w = 0; w < TKW; w++)
        if (W_D[w] >= 0) { if (W_S[w] == 0 && pos_v0 < 0) pos_v0 = nnl; NN[nnl++] = w; }
    START = (((pos_v0 - 4148) % nnl) + nnl) % nnl;   /* place systematic at offset 4148 */
    for (j = 0; j < TE; j++) {
        int ww = NN[(START + j) % nnl];
        src_stream[j] = W_S[ww];
        src_didx[j]   = W_D[ww];
    }
    free(W_S); free(W_D); free(NN);
    return 0;
}

/* ── soft QPSK demod: complex symbols -> 7200 bit LLRs (skips symbol 0) ──── */
static const float CQ_RE[4] = { 0.70710678f, 0.70710678f, -0.70710678f, -0.70710678f };
static const float CQ_IM[4] = { 0.70710678f, -0.70710678f, -0.70710678f, 0.70710678f };

static void soft_llr(dsp_complex_t syms[][DSP_NCARRIERS], int phase, float *L) {
    int si, k, q, idx = 0, b0[4], b1[4];
    for (q = 0; q < 4; q++) { int v = QPSK_TO_BITS[phase][q]; b0[q]=v&1; b1[q]=(v>>1)&1; }
    for (si = 1; si <= 6; si++) {                 /* drop symbol 0 (Gold pilot) */
        double sumP = 0.0; int cnt = 0, ci;
        float *dist, norm, sigma2; double minsum = 0.0;
        for (k = 0; k < DSP_NCARRIERS; k++) {
            if (k == DSP_NCARRIERS_HALF) continue;
            { float re = crealf(syms[si][k]), im = cimagf(syms[si][k]);
              sumP += (double)re*re + (double)im*im; cnt++; }
        }
        norm = (float)sqrt(sumP / (cnt > 0 ? cnt : 1)) + 1e-12f;
        dist = (float *)malloc(sizeof(float) * (size_t)cnt * 4);
        if (!dist) { int z; for (z = 0; z < cnt*2; z++) L[idx++] = 0.0f; continue; }
        ci = 0;
        for (k = 0; k < DSP_NCARRIERS; k++) {
            float re, im, mn = 1e30f;
            if (k == DSP_NCARRIERS_HALF) continue;
            re = crealf(syms[si][k]) / norm; im = cimagf(syms[si][k]) / norm;
            for (q = 0; q < 4; q++) {
                float dr = re - CQ_RE[q], di = im - CQ_IM[q];
                float d = dr*dr + di*di;
                dist[ci*4+q] = d;
                if (d < mn) mn = d;
            }
            minsum += mn; ci++;
        }
        sigma2 = (float)(minsum / (cnt > 0 ? cnt : 1)) + 1e-9f;
        for (ci = 0; ci < cnt; ci++) {
            float m00=1e30f, m10=1e30f, m01=1e30f, m11=1e30f;
            for (q = 0; q < 4; q++) {
                float d = dist[ci*4+q];
                if (b0[q]==0) { if (d<m00) m00=d; } else { if (d<m10) m10=d; }
                if (b1[q]==0) { if (d<m01) m01=d; } else { if (d<m11) m11=d; }
            }
            L[idx++] = (m10 - m00) / sigma2;      /* LLR bit0 (L=log P0/P1) */
            L[idx++] = (m11 - m01) / sigma2;      /* LLR bit1 */
        }
        free(dist);
    }
}

/* Resolve the QPSK phase via the Gold pilot (symbol 0 carries a known Gold
   sequence). Returns the rotation whose hard-decoded symbol-0 bits best match
   it, so the decoder typically runs ONE turbo pass instead of four. */
static int resolve_phase(dsp_complex_t syms[][DSP_NCARRIERS]) {
    uint8_t g[1200];
    int best = 0, bestm = -1, phase, k;
    dsp_gold_sequence(DSP_GOLD_SEED, DSP_GOLD_NC, 1200, g);
    for (phase = 0; phase < 4; phase++) {
        int m = 0, idx = 0;
        for (k = 0; k < DSP_NCARRIERS; k++) {
            float re, im; int q, v;
            if (k == DSP_NCARRIERS_HALF) continue;
            re = crealf(syms[0][k]); im = cimagf(syms[0][k]);
            q = (re >= 0.0f) ? ((im >= 0.0f) ? 0 : 1) : ((im < 0.0f) ? 2 : 3);
            v = QPSK_TO_BITS[phase][q];
            if ((v & 1)        == (int)g[idx]) m++;
            idx++;
            if (((v >> 1) & 1) == (int)g[idx]) m++;
            idx++;
        }
        if (m > bestm) { bestm = m; best = phase; }
    }
    return best;
}

/* ── public: soft + turbo decode (phase resolved via the Gold pilot) ─────── */
int32_t turbo_decode_dji(dsp_complex_t syms[][DSP_NCARRIERS],
                         int n_data_symbols, bool legacy,
                         uint8_t *payload_out) {
    float *L = NULL, *d0 = NULL, *d1 = NULL, *d2 = NULL;
    uint8_t *gs = NULL, *bits = NULL;
    int *ss = NULL, *sd = NULL;
    int phase, i, j;
    int order[4], norder, oi, p;
    int32_t result = LOCATE_ERR_CRC_FAIL;

    if (legacy || n_data_symbols < 7) return LOCATE_ERR_CRC_FAIL;  /* standard only */

    L  = (float *)malloc(sizeof(float) * TE);
    gs = (uint8_t *)malloc(TE);
    ss = (int *)malloc(sizeof(int) * TE);
    sd = (int *)malloc(sizeof(int) * TE);
    d0 = (float *)malloc(sizeof(float) * TD);
    d1 = (float *)malloc(sizeof(float) * TD);
    d2 = (float *)malloc(sizeof(float) * TD);
    bits = (uint8_t *)malloc(TK);
    if (!L||!gs||!ss||!sd||!d0||!d1||!d2||!bits) { result = LOCATE_ERR_ALLOC; goto done; }

    dsp_gold_sequence(DSP_GOLD_SEED, DSP_GOLD_NC, TE, gs);
    if (build_map(ss, sd) != 0) { result = LOCATE_ERR_ALLOC; goto done; }

    /* Resolve phase once; try it first, fall back to the others only if it fails. */
    norder = 1;
    order[0] = resolve_phase(syms);
    for (p = 0; p < 4; p++) if (p != order[0]) order[norder++] = p;

    for (oi = 0; oi < norder; oi++) {
        phase = order[oi];
        soft_llr(syms, phase, L);
        for (j = 0; j < TE; j++) if (gs[j]) L[j] = -L[j];          /* descramble */
        for (i = 0; i < TD; i++) { d0[i]=0.0f; d1[i]=0.0f; d2[i]=0.0f; }
        for (j = 0; j < TE; j++) {                                 /* scatter + soft combine */
            int di = sd[j];
            if      (ss[j]==0) d0[di] += L[j];
            else if (ss[j]==1) d1[di] += L[j];
            else               d2[di] += L[j];
        }
        if (turbo_decode_lte(d0, d1, d2, bits) != 0) { result = LOCATE_ERR_ALLOC; goto done; }
        {
            uint8_t bytes[176]; int bi, jj;
            uint16_t crc_calc, crc_pkt;
            for (bi = 0; bi < 176; bi++) {
                uint8_t b = 0;
                for (jj = 0; jj < 8; jj++) if (bits[bi*8+jj]) b |= (uint8_t)(1u << (7-jj));
                bytes[bi] = b;
            }
            crc_calc = compute_crc16(bytes, 89);
            crc_pkt  = (uint16_t)bytes[89] | ((uint16_t)bytes[90] << 8);
            if (crc_calc == crc_pkt) {
                memcpy(payload_out, bytes, 176);
                result = LOCATE_OK;
                goto done;
            }
        }
    }

done:
    free(L); free(gs); free(ss); free(sd);
    free(d0); free(d1); free(d2); free(bits);
    return result;
}

/* ── self-test: encode random bits, add AWGN, turbo-decode, verify ───────── */
static double rnd_gauss(void) {
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
    double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

static void rsc_enc(const uint8_t *u, uint8_t *sysb, uint8_t *par) {
    int r1=0, r2=0, r3=0, i, a, z, uu, t1, t2, t3;
    for (i = 0; i < TK; i++) {
        a = u[i] ^ r2 ^ r3; z = a ^ r1 ^ r3;
        sysb[i] = (uint8_t)u[i]; par[i] = (uint8_t)z;
        t1 = a; t2 = r1; t3 = r2; r1 = t1; r2 = t2; r3 = t3;
    }
    for (i = 0; i < 3; i++) {
        uu = r2 ^ r3; a = uu ^ r2 ^ r3; z = a ^ r1 ^ r3;
        sysb[TK+i] = (uint8_t)uu; par[TK+i] = (uint8_t)z;
        t1 = a; t2 = r1; t3 = r2; r1 = t1; r2 = t2; r3 = t3;
    }
}

int32_t locate_turbo_self_test(void) {
    int *pi = (int *)malloc(sizeof(int) * TK);
    int *ipi = (int *)malloc(sizeof(int) * TK);
    uint8_t *msg = (uint8_t *)malloc(TK), *msgi = (uint8_t *)malloc(TK);
    uint8_t *s1 = (uint8_t *)malloc(TK+3), *p1 = (uint8_t *)malloc(TK+3);
    uint8_t *s2 = (uint8_t *)malloc(TK+3), *p2 = (uint8_t *)malloc(TK+3);
    uint8_t *D0 = (uint8_t *)malloc(TD), *D1 = (uint8_t *)malloc(TD), *D2 = (uint8_t *)malloc(TD);
    float *d0 = (float *)malloc(sizeof(float)*TD);
    float *d1 = (float *)malloc(sizeof(float)*TD);
    float *d2 = (float *)malloc(sizeof(float)*TD);
    uint8_t *dec = (uint8_t *)malloc(TK);
    int i, errs = 0;
    double sigma = sqrt(1.0 / (2.0 * pow(10.0, 1.0/10.0)));   /* Es/N0 = 1 dB */
    if (!pi||!ipi||!msg||!msgi||!s1||!p1||!s2||!p2||!D0||!D1||!D2||!d0||!d1||!d2||!dec) {
        errs = -1; goto cleanup;
    }
    qpp_build(pi, ipi);
    srand(12345u);
    for (i = 0; i < TK; i++) msg[i] = (uint8_t)(rand() & 1);
    for (i = 0; i < TK; i++) msgi[i] = msg[pi[i]];
    rsc_enc(msg, s1, p1);
    rsc_enc(msgi, s2, p2);
    for (i = 0; i < TK; i++) { D0[i]=s1[i]; D1[i]=p1[i]; D2[i]=p2[i]; }
    /* LTE tail mux (inverse of the demux in turbo_decode_lte) */
    D0[TK]=s1[TK];   D2[TK]=s1[TK+1];   D1[TK+1]=s1[TK+2];   /* x  */
    D1[TK]=p1[TK];   D0[TK+1]=p1[TK+1]; D2[TK+1]=p1[TK+2];   /* z  */
    D0[TK+2]=s2[TK]; D2[TK+2]=s2[TK+1]; D1[TK+3]=s2[TK+2];   /* x' */
    D1[TK+2]=p2[TK]; D0[TK+3]=p2[TK+1]; D2[TK+3]=p2[TK+2];   /* z' */
    for (i = 0; i < TD; i++) {
        d0[i] = (float)(2.0 * ((1.0 - 2.0*D0[i]) + sigma*rnd_gauss()) / (sigma*sigma));
        d1[i] = (float)(2.0 * ((1.0 - 2.0*D1[i]) + sigma*rnd_gauss()) / (sigma*sigma));
        d2[i] = (float)(2.0 * ((1.0 - 2.0*D2[i]) + sigma*rnd_gauss()) / (sigma*sigma));
    }
    if (turbo_decode_lte(d0, d1, d2, dec) != 0) { errs = -1; goto cleanup; }
    for (i = 0; i < TK; i++) if (dec[i] != msg[i]) errs++;
cleanup:
    free(pi);free(ipi);free(msg);free(msgi);free(s1);free(p1);free(s2);free(p2);
    free(D0);free(D1);free(D2);free(d0);free(d1);free(d2);free(dec);
    return errs;   /* 0 = engine recovered the message (FEC working) */
}
