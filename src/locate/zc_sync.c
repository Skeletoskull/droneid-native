/**
 * zc_sync.c — ZC synchronisation, channel estimation, and zero-forcing equalisation
 *
 * Mirrors find_zc_seq(), estimate_channel(), and symbol_equalized() from Packet.py.
 *
 * Note: zc_sync_and_equalize() is called for ZC root validation only.
 *       QPSK decode uses raw (non-equalized) symbols — see locate_api.c.
 *
 * Responsibilities:
 *   - ZC root detection by correlating against roots r = 1..600
 *   - Validate: second ZC root must be 147 (standard mode)
 *   - Channel estimation using both ZC symbols, averaged per subcarrier
 *   - Zero-forcing equalisation applied to all data-bearing symbols
 */

#include "zc_sync.h"
#include <math.h>

/* ── Find best ZC root by correlation ───────────────────────────────────────
 * Mirrors Packet.find_zc_seq().
 */
static int find_zc_root(const dsp_complex_t *symbol_f) {
    float best_score = -1.0f;
    int best_root = 1;
    int r;

    dsp_complex_t *zc_t     = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    dsp_complex_t *corr_out = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    if (!zc_t || !corr_out) { free(zc_t); free(corr_out); return 1; }

    for (r = 1; r < DSP_NCARRIERS; r++) {
        dsp_zc_sequence_time((uint32_t)r, DSP_NCARRIERS, zc_t);
        dsp_correlate(symbol_f, zc_t, corr_out, DSP_NCARRIERS);
        uint32_t peak_idx = dsp_argmax_abs(corr_out, DSP_NCARRIERS);
        float score = cabsf(corr_out[peak_idx]);
        if (score > best_score) { best_score = score; best_root = r; }
    }

    free(zc_t);
    free(corr_out);
    return best_root;
}

/* Cheap ZC gate for the integer-CFO search: does this symbol correlate strongly
   with root 147? One correlation vs find_zc_root's 599. Coarse (un-fine-timed)
   symbols are fine — a timing ramp moves the correlation peak position, not its
   magnitude. Lenient threshold (avoid false negatives; the full zc_sync +CRC
   confirm afterwards). */
bool zc_symbol_matches_147(const dsp_complex_t *symbol_f) {
    dsp_complex_t *zc_t = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    dsp_complex_t *corr = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    float pk, mean = 0.0f;
    uint32_t k, peak;
    bool ok;
    if (!zc_t || !corr) { free(zc_t); free(corr); return false; }
    dsp_zc_sequence_time(147u, DSP_NCARRIERS, zc_t);
    dsp_correlate(symbol_f, zc_t, corr, DSP_NCARRIERS);
    peak = dsp_argmax_abs(corr, DSP_NCARRIERS);
    pk = cabsf(corr[peak]);
    for (k = 0; k < (uint32_t)DSP_NCARRIERS; k++) mean += cabsf(corr[k]);
    mean /= (float)DSP_NCARRIERS;
    ok = (mean > 1e-12f) && (pk > 4.0f * mean);
    free(zc_t); free(corr);
    return ok;
}

/* ── Channel estimation (zero-forcing) ──────────────────────────────────────
 * Mirrors Packet.estimate_channel().
 */
static void estimate_channel(const dsp_complex_t *received, int root,
                               dsp_complex_t *channel_out) {
    dsp_complex_t *zc_freq = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    if (!zc_freq) {
        int k;
        for (k = 0; k < DSP_NCARRIERS; k++) channel_out[k] = CMPLX(1.0f, 0.0f);
        return;
    }

    dsp_zc_sequence_freq((uint32_t)root, zc_freq);
    zc_freq[DSP_NCARRIERS_HALF] = CMPLX(1.0f, 0.0f);  /* restore DC */

    for (int k = 0; k < DSP_NCARRIERS; k++) {
        if (cabsf(zc_freq[k]) > 1e-10f)
            channel_out[k] = _dsp_cdiv(received[k], zc_freq[k]);
        else
            channel_out[k] = CMPLX(1.0f, 0.0f);
    }

    free(zc_freq);
}

/* ── Public entry point ────────────────────────────────────────────────────── */

int32_t zc_sync_and_equalize(ofdm_frame_t *frame, uint32_t flags,
                               dsp_complex_t equalized[][DSP_NCARRIERS],
                               int *n_data_symbols_out) {
    bool legacy = (flags & LOCATE_FLAG_LEGACY) != 0;

    int zc_idx0    = legacy ? DSP_ZC_IDX_LEG_0  : DSP_ZC_IDX_STD_0;
    int zc_idx1    = legacy ? DSP_ZC_IDX_LEG_1  : DSP_ZC_IDX_STD_1;
    const int *data_syms = legacy ? DATA_SYMBOLS_LEG : DATA_SYMBOLS_STD;
    int n_data     = legacy ? N_DATA_SYMBOLS_LEG : N_DATA_SYMBOLS_STD;

    /* Step 1: Determine ZC roots.
       LOCATE_FLAG_ASSUME_ZC skips the 599-root correlation search (the dominant
       per-candidate cost) and assumes the standard DroneID roots 600/147; the
       CRC validates the frame downstream. Otherwise search + validate root1==147. */
    int root0, root1;
    if ((flags & LOCATE_FLAG_ASSUME_ZC) != 0 && !legacy) {
        root0 = 600;
        root1 = 147;
    } else {
        root0 = find_zc_root(frame->symbols[zc_idx0]);
        root1 = find_zc_root(frame->symbols[zc_idx1]);
        /* Step 2: Validate second ZC root — must be 147 in standard mode */
        if (!legacy && root1 != 147)
            return LOCATE_ERR_ZC_NOT_FOUND;
    }

    /* Step 3: Channel estimation */
    dsp_complex_t *ch0     = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    dsp_complex_t *ch1     = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    dsp_complex_t *channel = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    if (!ch0 || !ch1 || !channel) { free(ch0); free(ch1); free(channel); return LOCATE_ERR_ALLOC; }

    estimate_channel(frame->symbols[zc_idx0], root0, ch0);
    estimate_channel(frame->symbols[zc_idx1], root1, ch1);

    for (int k = 0; k < DSP_NCARRIERS; k++)
        channel[k] = _dsp_cmul(CMPLX(0.5f, 0.0f), _dsp_cadd(ch0[k], ch1[k]));

    free(ch0);
    free(ch1);

    /* Step 4: Zero-forcing equalisation */
    for (int di = 0; di < n_data; di++) {
        int si = data_syms[di];
        for (int k = 0; k < DSP_NCARRIERS; k++) {
            if (cabsf(channel[k]) > 1e-10f)
                equalized[di][k] = _dsp_cdiv(frame->symbols[si][k], channel[k]);
            else
                equalized[di][k] = frame->symbols[si][k];
        }
    }

    *n_data_symbols_out = n_data;
    free(channel);
    return LOCATE_OK;
}
