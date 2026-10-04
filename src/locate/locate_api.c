/**
 * locate_api.c — Public API entry points for droneid_locate.
 *
 * All variable declarations are at the top of each block for MSVC C89 compat.
 */

#include "locate_internal.h"
#include <stdio.h>
#include <stdlib.h>

#define VERSION_MAJOR 1u
#define VERSION_MINOR 0u
#define VERSION_PATCH 0u

/* Half-span (in 15 kHz subcarriers) of the integer-CFO search. Default ±10 =
   ±150 kHz. Raise via env DRONEID_CFO_SPAN to tolerate larger live CFO drift /
   band-detection skew in crowded bands — each extra shift is just one cheap
   coarse demod, and clean bursts still match at m=0 and stop, so there is no
   added cost for good frames. */
static int locate_cfo_span(void) {
    static int s = -1;
    if (s < 0) {
        const char *e = getenv("DRONEID_CFO_SPAN");
        s = (e && e[0]) ? atoi(e) : 10;
        if (s < 0)   s = 0;
        if (s > 200) s = 200;   /* sanity clamp */
    }
    return s;
}

/* ── locate_droneid ──────────────────────────────────────────────────────── */

int32_t locate_droneid(const locate_complex_t *candidate_samples,
                        uint32_t num_samples, uint32_t flags,
                        telemetry_result_t *result_out) {
    ofdm_frame_t  *frame     = NULL;
    dsp_complex_t (*raw_data)[DSP_NCARRIERS] = NULL;  /* raw (non-equalized) data symbols */
    dsp_complex_t (*equalized)[DSP_NCARRIERS] = NULL;
    dsp_complex_t *shifted = NULL;
    int            n_data = 0, di;
    bool           legacy, decoded = false;
    uint8_t        payload[256];
    const int     *data_syms;
    const double   subcarrier = DSP_FS_TARGET / (double)DSP_NFFT;   /* 15 kHz */
    /* Integer-subcarrier CFO trials, generated in the order 0, -1, +1, -2, +2 …
       up to ±span so the most likely shift (m=0) is tried first and clean bursts
       cost nothing extra. span is runtime-configurable (see locate_cfo_span). */
    int span = locate_cfo_span();
    int n_try, mi;

    if (!candidate_samples || num_samples == 0 || !result_out)
        return LOCATE_ERR_INVALID_ARG;

    memset(result_out, 0, sizeof(*result_out));
    legacy    = (flags & LOCATE_FLAG_LEGACY) != 0;
    data_syms = legacy ? DATA_SYMBOLS_LEG : DATA_SYMBOLS_STD;

    frame     = (ofdm_frame_t *)calloc(1, sizeof(ofdm_frame_t));
    equalized = (dsp_complex_t (*)[DSP_NCARRIERS])calloc(9 * DSP_NCARRIERS, sizeof(dsp_complex_t));
    raw_data  = (dsp_complex_t (*)[DSP_NCARRIERS])calloc(9 * DSP_NCARRIERS, sizeof(dsp_complex_t));
    shifted   = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * num_samples);
    if (!frame || !equalized || !raw_data || !shifted) {
        free(frame); free(equalized); free(raw_data); free(shifted);
        return LOCATE_ERR_ALLOC;
    }

    /* Integer-CFO search: the coarse band-offset estimate is bin-quantized
       (Fs/2048 — ~49 kHz at 100 Msps) plus band-edge error, leaving a residual
       of several subcarriers that the CP fractional-CFO step (±7.5 kHz) cannot
       pull in. Try integer-subcarrier shifts; accept the first whose ZC root is
       147 AND whose decoded DUML passes CRC (so a wrong shift can't be falsely
       accepted). ASSUME_ZC (no ZC validation) collapses the search to m=0. */
    n_try = (flags & LOCATE_FLAG_ASSUME_ZC) ? 1 : (2 * span + 1);
    for (mi = 0; mi < n_try; mi++) {
        /* mi → m: 0, -1, +1, -2, +2, … (m=0 first). */
        int m = (mi == 0) ? 0 : ((mi & 1) ? -((mi + 1) / 2) : (mi / 2));
        /* Standard mode: skip the 599-root ZC search inside zc_sync_and_equalize.
           The cheap root-147 gate below already confirms the ZC symbol, and the
           DUML CRC is the definitive validator — a wrong shift or non-DroneID
           burst cannot forge a valid CRC. This removes the dominant per-candidate
           cost (2 × 599 O(N²) correlations). Legacy keeps the full search (no 147
           invariant). */
        uint32_t zc_flags = legacy ? flags : (flags | LOCATE_FLAG_ASSUME_ZC);
        memcpy(shifted, candidate_samples, sizeof(dsp_complex_t) * num_samples);
        if (m != 0)
            dsp_fshift(shifted, num_samples, (float)(-(double)m * subcarrier), DSP_FS_TARGET);

        /* Cheap coarse demod (skips the 1000-iter fine timing), then a one-
           correlation root-147 gate (vs the 599-root search) so wrong shifts and
           non-DroneID bursts bail fast. */
        if (ofdm_demodulate(shifted, num_samples, flags, 0, frame) != LOCATE_OK) continue;
        if (!(flags & LOCATE_FLAG_ASSUME_ZC) &&
            !zc_symbol_matches_147(frame->symbols[legacy ? DSP_ZC_IDX_LEG_1 : DSP_ZC_IDX_STD_1]))
            continue;

        /* Matching shift → full demod (fine timing + phase) for an accurate decode. */
        if (ofdm_demodulate(shifted, num_samples, flags, 1, frame) != LOCATE_OK) continue;
        if (zc_sync_and_equalize(frame, zc_flags, equalized, &n_data) != LOCATE_OK) continue;

        for (di = 0; di < n_data; di++)
            memcpy(raw_data[di], frame->symbols[data_syms[di]],
                   sizeof(dsp_complex_t) * DSP_NCARRIERS);
        if (qpsk_decode(raw_data, n_data, legacy, payload) != LOCATE_OK) continue;

        memset(result_out, 0, sizeof(*result_out));
        duml_parse(payload, result_out);
        if ((flags & LOCATE_FLAG_ASSUME_ZC) || result_out->crc_valid) {
            decoded = true;   /* in ASSUME mode keep m=0 result regardless of CRC */
            /* Optional research dump (env DRONEID_DUMP=<dir>): write the real
               demodulated symbol grid and the winning candidate for one decoded
               burst, so figures can be made from measured data. Inert unless set. */
            {
                const char *dd = getenv("DRONEID_DUMP");
                if (dd && dd[0]) {
                    char pth[1024]; FILE *fp;
                    snprintf(pth, sizeof(pth), "%s/symbols.bin", dd);
                    fp = fopen(pth, "wb");
                    if (fp) { fwrite(frame->symbols, sizeof(dsp_complex_t), 9u * DSP_NCARRIERS, fp); fclose(fp); }
                    snprintf(pth, sizeof(pth), "%s/cand.bin", dd);
                    fp = fopen(pth, "wb");
                    if (fp) { fwrite(shifted, sizeof(dsp_complex_t), num_samples, fp); fclose(fp); }
                    snprintf(pth, sizeof(pth), "%s/meta.txt", dd);
                    fp = fopen(pth, "w");
                    if (fp) { fprintf(fp, "n=%u\nm=%d\nstart=%d\nffo_hz=%f\nserial=%s\n",
                                      num_samples, m, frame->start_sample, frame->ffo_hz,
                                      result_out->serial_number); fclose(fp); }
                }
            }
            break;
        }
    }

    free(frame); free(equalized); free(raw_data); free(shifted);
    return decoded ? LOCATE_OK : LOCATE_ERR_ZC_NOT_FOUND;
}

/* ── locate_droneid_file / locate_free_results ───────────────────────────── */
/* Implemented in locate_file.c (detection + decoding pipeline). */

/* ── telemetry_to_json ───────────────────────────────────────────────────── */

int32_t telemetry_to_json(const telemetry_result_t *result,
                           char *buf, uint32_t buf_len) {
    return json_serialize(result, buf, buf_len);
}

/* ── telemetry_from_json ─────────────────────────────────────────────────── */

int32_t telemetry_from_json(const char *json_str, telemetry_result_t *result) {
    return json_deserialize(json_str, result);
}

/* ── compute_crc16 ───────────────────────────────────────────────────────── */
/* Implemented in crc16.c — declared in droneid_locate.h */

/* ── locate_version ──────────────────────────────────────────────────────── */

void locate_version(uint32_t *major, uint32_t *minor, uint32_t *patch) {
    if (major) *major = VERSION_MAJOR;
    if (minor) *minor = VERSION_MINOR;
    if (patch) *patch = VERSION_PATCH;
}
