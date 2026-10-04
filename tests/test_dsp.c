/**
 * test_dsp.c - DSP primitives unit tests
 *
 * Tests FFT correctness, ZC sequences, Gold sequences,
 * resampling, and frequency shift.
 * Uses dsp_complex_t and helpers for MSVC compatibility.
 *
 * MSVC C89-compatible: all variable declarations are at the top of each block.
 */

#include "../src/dsp/dsp_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

static int g_pass = 0, g_fail = 0;

#define ASSERT(cond, msg) do { \
    if (cond) { printf("  PASS: %s\n", msg); g_pass++; } \
    else       { printf("  FAIL: %s\n", msg); g_fail++; } \
} while(0)

/* ── FFT tests ────────────────────────────────────────────────────────────── */

static void test_fft_dc(void) {
    dsp_complex_t in[DSP_NFFT];
    dsp_complex_t out[DSP_NFFT];
    float err, max_other;
    int i;

    for (i = 0; i < DSP_NFFT; i++) in[i] = CMPLX(1.0f, 0.0f);
    dsp_fft_forward(in, out, DSP_NFFT);

    err = _dsp_cabsf(_dsp_csub(out[0], CMPLX((float)DSP_NFFT, 0.0f)));
    ASSERT(err < 1e-3f, "FFT DC: bin 0 = N");

    max_other = 0.0f;
    for (i = 1; i < DSP_NFFT; i++) {
        float a = _dsp_cabsf(out[i]);
        if (a > max_other) max_other = a;
    }
    ASSERT(max_other < 1e-3f, "FFT DC: other bins ~= 0");
}

static void test_fft_tone(void) {
    dsp_complex_t in[DSP_NFFT];
    dsp_complex_t out[DSP_NFFT];
    float peak;
    int k = 7, n;

    for (n = 0; n < DSP_NFFT; n++) {
        double phase = 2.0 * M_PI * k * n / DSP_NFFT;
        in[n] = CMPLX((float)cos(phase), (float)sin(phase));
    }
    dsp_fft_forward(in, out, DSP_NFFT);
    peak = _dsp_cabsf(out[k]);
    ASSERT(fabsf(peak - (float)DSP_NFFT) < 1.0f, "FFT tone: peak at correct bin");
}

static void test_fft_roundtrip(void) {
    dsp_complex_t in[DSP_NFFT];
    dsp_complex_t fwd[DSP_NFFT];
    dsp_complex_t inv_buf[DSP_NFFT];
    float max_err;
    int i;

    for (i = 0; i < DSP_NFFT; i++)
        in[i] = CMPLX((float)(i % 13) * 0.1f, (float)(i % 7) * 0.05f);
    dsp_fft_forward(in, fwd, DSP_NFFT);
    /* dsp_fft_inverse already normalizes by 1/N (matching np.fft.ifft),
       so inv_buf should equal in[] directly without further scaling. */
    dsp_fft_inverse(fwd, inv_buf, DSP_NFFT);

    max_err = 0.0f;
    for (i = 0; i < DSP_NFFT; i++) {
        float err = _dsp_cabsf(_dsp_csub(inv_buf[i], in[i]));
        if (err > max_err) max_err = err;
    }
    ASSERT(max_err < 1e-4f, "FFT round-trip: ifft(fft(x)) == x (1/N normalized)");
}

/* ── ZC sequence tests ────────────────────────────────────────────────────── */

static void test_zc_unit_magnitude(void) {
    dsp_complex_t zc[DSP_NCARRIERS];
    float max_err;
    int i;

    dsp_zc_sequence_time(147, DSP_NCARRIERS, zc);
    max_err = 0.0f;
    for (i = 0; i < DSP_NCARRIERS; i++) {
        float err = fabsf(_dsp_cabsf(zc[i]) - 1.0f);
        if (err > max_err) max_err = err;
    }
    ASSERT(max_err < 1e-5f, "ZC sequence: unit magnitude");
}

static void test_zc_formula(void) {
    uint32_t u = 42, N = DSP_NCARRIERS, n;
    dsp_complex_t zc[DSP_NCARRIERS];
    float max_err;

    dsp_zc_sequence_time(u, N, zc);
    max_err = 0.0f;
    for (n = 0; n < N; n++) {
        double phase = -M_PI * u * n * (n + 1.0) / N;
        dsp_complex_t expected = CMPLX((float)cos(phase), (float)sin(phase));
        float err = _dsp_cabsf(_dsp_csub(zc[n], expected));
        if (err > max_err) max_err = err;
    }
    ASSERT(max_err < 1e-5f, "ZC formula matches reference");
}

/* ── Gold sequence tests ──────────────────────────────────────────────────── */

static void test_gold_binary(void) {
    uint8_t gold[100];
    int all_binary, i;

    dsp_gold_sequence(DSP_GOLD_SEED, DSP_GOLD_NC, 100, gold);
    all_binary = 1;
    for (i = 0; i < 100; i++)
        if (gold[i] != 0 && gold[i] != 1) { all_binary = 0; break; }
    ASSERT(all_binary, "Gold sequence: all values 0 or 1");
}

static void test_gold_reproducible(void) {
    uint8_t g1[200], g2[200];
    dsp_gold_sequence(DSP_GOLD_SEED, DSP_GOLD_NC, 200, g1);
    dsp_gold_sequence(DSP_GOLD_SEED, DSP_GOLD_NC, 200, g2);
    ASSERT(memcmp(g1, g2, 200) == 0, "Gold sequence: reproducible");
}

/* ── Resampling tests ─────────────────────────────────────────────────────── */

static void test_resample_length(void) {
    uint32_t n_in = 1000;
    double fs_in = 50e6, fs_out = DSP_FS_TARGET;
    uint32_t expected = dsp_resample_output_length(n_in, fs_in, fs_out);
    ASSERT(expected > 0 && expected < n_in, "Resample: output shorter than input");
}

static void test_resample_dc(void) {
    uint32_t n_in = 500, i;
    dsp_complex_t in[500];
    dsp_complex_t out[500];
    uint32_t n_out;
    float max_err;

    for (i = 0; i < n_in; i++) in[i] = CMPLX(1.0f, 0.0f);
    n_out = dsp_resample_output_length(n_in, 50e6, DSP_FS_TARGET);
    if (n_out > 500) n_out = 500;
    dsp_resample(in, n_in, out, n_out, 50e6, DSP_FS_TARGET);

    max_err = 0.0f;
    for (i = 0; i < n_out; i++) {
        float err = _dsp_cabsf(_dsp_csub(out[i], CMPLX(1.0f, 0.0f)));
        if (err > max_err) max_err = err;
    }
    ASSERT(max_err < 1e-5f, "Resample: DC signal preserved");
}

/* ── Subcarrier reorder tests (tfft / itfft equivalence) ─────────────────── */

/**
 * Verify dsp_fft_subcarriers matches the Python tfft() function:
 *   new_fft = np.concatenate((fft[-half_carriers:], fft[:half_carriers+1]))
 * where half_carriers=300, NFFT=1024.
 *
 * Strategy: compute the FFT independently, manually reorder as Python does,
 * and compare against dsp_fft_subcarriers output.
 */
static void test_subcarrier_reorder(void) {
    dsp_complex_t in[DSP_NFFT];
    dsp_complex_t out[DSP_NCARRIERS];
    dsp_complex_t full[DSP_NFFT];
    dsp_complex_t expected[DSP_NCARRIERS];
    float max_err;
    int i;

    /* Fill with a known test pattern */
    for (i = 0; i < DSP_NFFT; i++) {
        double phase = 2.0 * M_PI * 7 * i / DSP_NFFT;
        in[i] = CMPLX((float)cos(phase), (float)sin(phase));
    }

    /* Compute expected via manual reorder (mirrors Python tfft) */
    dsp_fft_forward(in, full, DSP_NFFT);
    /* expected[0:300] = full[724:1024]  (negative freqs: last 300) */
    for (i = 0; i < DSP_NCARRIERS_HALF; i++)
        expected[i] = full[DSP_NFFT - DSP_NCARRIERS_HALF + i];
    /* expected[300:601] = full[0:301]   (DC + positive freqs: first 301) */
    for (i = 0; i <= DSP_NCARRIERS_HALF; i++)
        expected[DSP_NCARRIERS_HALF + i] = full[i];

    /* Compute via dsp_fft_subcarriers */
    dsp_fft_subcarriers(in, out);

    max_err = 0.0f;
    for (i = 0; i < DSP_NCARRIERS; i++) {
        float err = _dsp_cabsf(_dsp_csub(out[i], expected[i]));
        if (err > max_err) max_err = err;
    }
    ASSERT(max_err < 1e-4f, "Subcarrier reorder: matches Python tfft");
}

/**
 * Verify dsp_ifft_subcarriers matches the Python itfft() function:
 *   c_full[-half_carriers:] = c[:half_carriers]
 *   c_full[:half_carriers+1] = c[half_carriers:]
 *   return np.fft.ifft(c_full)
 */
static void test_subcarrier_inverse_reorder(void) {
    dsp_complex_t carriers[DSP_NCARRIERS];
    dsp_complex_t out[DSP_NFFT];
    dsp_complex_t c_full[DSP_NFFT];
    dsp_complex_t expected[DSP_NFFT];
    float max_err;
    int i;

    /* Fill carriers with a test pattern */
    for (i = 0; i < DSP_NCARRIERS; i++) {
        double phase = 2.0 * M_PI * 3 * i / DSP_NCARRIERS;
        carriers[i] = CMPLX((float)cos(phase), (float)sin(phase));
    }

    /* Compute expected via manual reverse reorder (mirrors Python itfft) */
    memset(c_full, 0, sizeof(c_full));
    /* c_full[-300:] = carriers[:300]  → c_full[724:1024] = carriers[0:300] */
    for (i = 0; i < DSP_NCARRIERS_HALF; i++)
        c_full[DSP_NFFT - DSP_NCARRIERS_HALF + i] = carriers[i];
    /* c_full[:301] = carriers[300:]  → c_full[0:301] = carriers[300:601] */
    for (i = 0; i <= DSP_NCARRIERS_HALF; i++)
        c_full[i] = carriers[DSP_NCARRIERS_HALF + i];
    dsp_fft_inverse(c_full, expected, DSP_NFFT);

    /* Compute via dsp_ifft_subcarriers */
    dsp_ifft_subcarriers(carriers, out);

    max_err = 0.0f;
    for (i = 0; i < DSP_NFFT; i++) {
        float err = _dsp_cabsf(_dsp_csub(out[i], expected[i]));
        if (err > max_err) max_err = err;
    }
    ASSERT(max_err < 1e-4f, "Subcarrier inverse reorder: matches Python itfft");
}

/**
 * Verify that dsp_fft_subcarriers and dsp_ifft_subcarriers are inverses:
 * itfft(tfft(x))[FFT window] ≈ x  (the 601 carriers reconstruct after round-trip)
 */
static void test_subcarrier_roundtrip(void) {
    dsp_complex_t in[DSP_NFFT];
    dsp_complex_t carriers[DSP_NCARRIERS];
    dsp_complex_t out[DSP_NFFT];
    float max_err;
    int i;

    /* Input: a tone at a positive frequency that falls within active carriers */
    for (i = 0; i < DSP_NFFT; i++) {
        double phase = 2.0 * M_PI * 50 * i / DSP_NFFT;
        in[i] = CMPLX((float)cos(phase), (float)sin(phase));
    }

    /* Forward subcarrier extraction then inverse */
    dsp_fft_subcarriers(in, carriers);
    dsp_ifft_subcarriers(carriers, out);

    /* The round-trip goes through a subcarrier window (only 601 of 1024 bins
       are preserved), so we check that the active subcarrier content is
       reproduced (error < 1e-3 accounts for the discarded out-of-band energy) */
    max_err = 0.0f;
    for (i = 0; i < DSP_NFFT; i++) {
        float err = _dsp_cabsf(_dsp_csub(out[i], in[i]));
        if (err > max_err) max_err = err;
    }
    /* For a single-tone signal within the subcarrier bandwidth, round-trip
       should be nearly exact */
    ASSERT(max_err < 1e-3f, "Subcarrier round-trip: ifft(fft(x)) ≈ x");
}

/* ── Welch / CFO bandwidth test ───────────────────────────────────────────── */

static void test_welch_wideband_cfo(void) {
    /* Linear chirp spanning 9 MHz, centred at +1 MHz CFO */
    const uint32_t n = 65536;
    const double fs = 50e6;
    const float cfo_true = 1.0e6f;
    const float chirp_bw = 9.0e6f;
    dsp_complex_t *s;
    float t_end, chirp_k, f0;
    float psd[DSP_WELCH_NFFT], freqs[DSP_WELCH_NFFT];
    float cfo_est, err;
    bool ok;
    uint32_t i;

    s = (dsp_complex_t *)malloc(n * sizeof(dsp_complex_t));
    if (!s) return;

    t_end   = (float)n / (float)fs;
    chirp_k = chirp_bw / t_end;      /* Hz/s chirp rate */
    f0      = cfo_true - 0.5f * chirp_bw;

    for (i = 0; i < n; i++) {
        float t     = (float)i / (float)fs;
        float phase = 2.0f * (float)M_PI * (f0 * t + 0.5f * chirp_k * t * t);
        s[i] = CMPLX(cosf(phase), sinf(phase));
    }

    /* Debug: list band widths found */
    if (dsp_welch_psd(s, n, fs, psd, freqs)) {
        float mean = 0.0f;
        float max_bw = 0.0f;
        float bin_hz;
        bool in_band = false;
        uint32_t bs = 0, k;

        for (k = 0; k < DSP_WELCH_NFFT; k++) mean += psd[k];
        mean /= (float)DSP_WELCH_NFFT;

        {
            uint32_t dc = DSP_WELCH_NFFT / 2;
            for (k = dc - 10; k < dc + 10; k++) psd[k] = 1.1f * mean;
        }

        bin_hz = (float)(fs / DSP_WELCH_NFFT);

        for (k = 0; k <= DSP_WELCH_NFFT; k++) {
            bool above = (k < DSP_WELCH_NFFT) && (psd[k] > mean);
            if (above && !in_band) { bs = k; in_band = true; }
            else if (!above && in_band) {
                float bw_found;
                in_band = false;
                bw_found = ((float)(k - 1) - (float)bs) * bin_hz;
                if (bw_found > max_bw) max_bw = bw_found;
            }
        }
        printf("  DBG chirp max_band_bw=%.2f MHz mean=%.4e\n",
               max_bw / 1e6f, mean);
    }

    cfo_est = 0.0f;
    ok = dsp_estimate_cfo(s, n, fs, 8e6f, 11e6f, &cfo_est);
    free(s);

    ASSERT(ok, "Welch CFO: detects 8-11 MHz wideband burst");
    if (ok) {
        err = fabsf(cfo_est - cfo_true);
        ASSERT(err < 500e3f, "Welch CFO: estimate within 500 kHz");
    }
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void) {
    printf("=== DSP Primitives Tests ===\n");

    printf("\n[FFT]\n");
    test_fft_dc();
    test_fft_tone();
    test_fft_roundtrip();

    printf("\n[ZC Sequences]\n");
    test_zc_unit_magnitude();
    test_zc_formula();

    printf("\n[Gold Sequences]\n");
    test_gold_binary();
    test_gold_reproducible();

    printf("\n[Resampling]\n");
    test_resample_length();
    test_resample_dc();

    printf("\n[Welch CFO]\n");
    test_welch_wideband_cfo();

    printf("\n[Subcarrier Reorder (tfft/itfft)]\n");
    test_subcarrier_reorder();
    test_subcarrier_inverse_reorder();
    test_subcarrier_roundtrip();

    printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail > 0) ? 1 : 0;
}
