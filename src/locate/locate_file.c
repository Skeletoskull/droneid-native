/**
 * locate_file.c — File-based processing for droneid_locate.
 *
 * Implements locate_droneid_file() and locate_free_results().
 *
 * Architecture note: droneid_locate is independent of droneid_detect at
 * runtime.  To support offline file processing, this module re-implements the same minimal detection pipeline used by
 * packet_detector.c in droneid_detect, using only the DSP primitives that
 * are statically linked into both libraries (dsp_stft_power,
 * dsp_estimate_cfo, dsp_fshift, dsp_resample, dsp_resample_output_length).
 */

#include "locate_internal.h"
#include "../dsp/dsp_internal.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

/* Optional timing profiler — enabled by setting env var DRONEID_PROF=1.
   Prints the detect-vs-decode split to stderr. Zero cost when disabled. */
static int lf_prof_enabled(void) {
    static int cached = -1;
    if (cached < 0) { const char *e = getenv("DRONEID_PROF"); cached = (e && e[0] && e[0] != '0') ? 1 : 0; }
    return cached;
}
#define LF_PROF_MS(t0) (1000.0 * (double)((clock()) - (t0)) / (double)CLOCKS_PER_SEC)

/* ── Detection parameters (mirrors packet_detector.h) ─────────────────── */
#define LF_STFT_THRESHOLD   1.15f
#define LF_BW_MIN_HZ        8e6f
#define LF_BW_MAX_HZ        11e6f
#define LF_GUARD_US         45.0f   /* 3 × 15 µs guard on each side */
#define LF_MIN_DUR_STD_US   550.0f
#define LF_MAX_DUR_STD_US   750.0f
#define LF_MIN_DUR_LEG_US   565.0f
#define LF_MAX_DUR_LEG_US   600.0f
#define LF_MAX_CANDS        64

typedef struct {
    uint32_t start_sample;
    uint32_t end_sample;
} lf_candidate_t;

/* Median of per-frame STFT peak power — robust noise floor (see packet_detector.c).
   mean-over-all-bins reads below the per-frame peak, so noise frames falsely pass. */
static int lf_cmp_float(const void *a, const void *b) {
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa < fb) ? -1 : (fa > fb) ? 1 : 0;
}
static float lf_median(const float *x, uint32_t n) {
    /* Subsample to ~25k points (noise-floor estimate only) — keeps it cheap. */
    uint32_t stride = (n > 25000u) ? (n / 25000u) : 1u;
    uint32_t i, j = 0, cnt = (n + stride - 1) / stride;
    float *c; float m;
    if (n == 0) return 0.0f;
    c = (float *)malloc(sizeof(float) * cnt);
    if (!c) return 0.0f;
    for (i = 0; i < n; i += stride) c[j++] = x[i];
    qsort(c, j, sizeof(float), lf_cmp_float);
    /* Median of per-frame peak power (see packet_detector.c). */
    m = c[j / 2];
    free(c);
    return m;
}

/* ── find_candidates ─────────────────────────────────────────────────────── */
/**
 * Scan the STFT power array for contiguous runs above threshold that fall
 * within the expected DroneID frame duration.
 * Returns the number of candidates written to cands[].
 */
static uint32_t find_candidates(const float *power, uint32_t n_frames,
                                 float threshold,
                                 uint32_t min_frames, uint32_t max_frames,
                                 lf_candidate_t *cands, uint32_t max_cands) {
    uint32_t count = 0;
    bool in_run = false;
    uint32_t run_start = 0;
    uint32_t i;

    for (i = 0; i <= n_frames; i++) {
        bool above = (i < n_frames) && (power[i] > threshold);
        if (above && !in_run) {
            run_start = i;
            in_run = true;
        } else if (!above && in_run) {
            uint32_t run_len = i - run_start;
            in_run = false;
            if (run_len >= min_frames && run_len <= max_frames) {
                if (count < max_cands) {
                    /* Map STFT frame indices back to sample positions.
                       STFT uses a 50% overlap: step = DSP_STFT_NFFT / 2. */
                    uint32_t stft_step = DSP_STFT_NFFT / 2;
                    cands[count].start_sample = run_start * stft_step;
                    cands[count].end_sample   =
                        (i - 1) * stft_step + (uint32_t)DSP_STFT_NFFT;
                    count++;
                }
            }
        }
    }
    return count;
}

/* ── detect_and_decode_chunk ─────────────────────────────────────────────── */
/**
 * Run the detection pipeline on one 500 ms chunk of raw IQ samples, then
 * invoke locate_droneid() on each candidate that passes bandwidth/CFO checks.
 *
 * Appends successfully decoded telemetry to *results_io / *count_io.
 */
static int32_t detect_and_decode_chunk(
        const dsp_complex_t *chunk, uint32_t n,
        double sample_rate, uint32_t flags,
        telemetry_result_t **results_io, uint32_t *count_io) {

    uint32_t n_frames_max;
    uint32_t n_frames = 0;
    float   *power = NULL;
    float    noise_floor, threshold;
    float    frame_dur_s;
    bool     legacy;
    float    min_dur_us, max_dur_us;
    uint32_t min_frames_pkt, max_frames_pkt;
    lf_candidate_t cands[LF_MAX_CANDS];
    uint32_t n_cands;
    uint32_t guard;
    uint32_t ci;

    clock_t prof_t0 = 0, prof_stft = 0;
    int prof = lf_prof_enabled();

    n_frames_max = (n / (DSP_STFT_NFFT / 2)) + 2;
    if (n_frames_max == 0) return LOCATE_OK;

    power = (float *)malloc(sizeof(float) * n_frames_max);
    if (!power) return LOCATE_ERR_ALLOC;

    if (prof) prof_t0 = clock();
    dsp_stft_power(chunk, n, power, &n_frames);
    if (prof) { prof_stft = clock(); fprintf(stderr, "[prof] STFT detect (%u samp): %.0f ms\n", n, 1000.0*(double)(prof_stft-prof_t0)/CLOCKS_PER_SEC); }

    /* Noise floor = median of per-frame STFT peak power, robust for the long,
       sparse over-the-air captures this library targets.
       See packet_detector.c for the mean-vs-median trade-off. */
    noise_floor = lf_median(power, n_frames);
    threshold   = LF_STFT_THRESHOLD * noise_floor;

    frame_dur_s = (float)(DSP_STFT_NFFT / 2) / (float)sample_rate;

    legacy      = (flags & LOCATE_FLAG_LEGACY) != 0;
    min_dur_us  = legacy ? LF_MIN_DUR_LEG_US : LF_MIN_DUR_STD_US;
    max_dur_us  = legacy ? LF_MAX_DUR_LEG_US : LF_MAX_DUR_STD_US;

    min_frames_pkt = (uint32_t)(min_dur_us * 1e-6f / frame_dur_s);
    max_frames_pkt = (uint32_t)(max_dur_us * 1e-6f / frame_dur_s) + 1;

    n_cands = find_candidates(power, n_frames, threshold,
                               min_frames_pkt, max_frames_pkt,
                               cands, LF_MAX_CANDS);
    free(power);

    if (n_cands == 0) {
        if (prof) fprintf(stderr, "[prof] candidates: 0\n");
        return LOCATE_OK;
    }

    guard = (uint32_t)(LF_GUARD_US * 1e-6 * sample_rate);

    {
    clock_t prof_welch = 0, prof_resamp = 0, prof_locate = 0, prof_c;
    for (ci = 0; ci < n_cands; ci++) {
        uint32_t s0 = (cands[ci].start_sample > guard)
                      ? cands[ci].start_sample - guard : 0;
        uint32_t s1 = cands[ci].end_sample + guard;
        uint32_t seg_len;
        float cfo_hz = 0.0f;
        bool band_ok;
        dsp_complex_t *cfo_corrected = NULL;
        dsp_complex_t *resampled     = NULL;
        uint32_t n_out;
        telemetry_result_t result;
        telemetry_result_t *tmp;
        int32_t rc;

        if (s1 > n) s1 = n;
        seg_len = s1 - s0;
        if (seg_len == 0) continue;

        /* Bandwidth/CFO check — reject candidates outside 8–11 MHz */
        prof_c = clock();
        band_ok = dsp_estimate_cfo(chunk + s0, seg_len, sample_rate,
                                    LF_BW_MIN_HZ, LF_BW_MAX_HZ, &cfo_hz);
        if (prof) prof_welch += clock() - prof_c;
        if (!band_ok) continue;

        /* Apply CFO correction */
        cfo_corrected = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * seg_len);
        if (!cfo_corrected) return LOCATE_ERR_ALLOC;
        memcpy(cfo_corrected, chunk + s0, sizeof(dsp_complex_t) * seg_len);
        dsp_fshift(cfo_corrected, seg_len, -cfo_hz, sample_rate);

        /* Resample to 15.36 MHz */
        n_out = dsp_resample_output_length(seg_len, sample_rate, DSP_FS_TARGET);
        resampled = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * n_out);
        if (!resampled) {
            free(cfo_corrected);
            return LOCATE_ERR_ALLOC;
        }
        prof_c = clock();
        dsp_resample(cfo_corrected, seg_len, resampled, n_out,
                     sample_rate, DSP_FS_TARGET);
        if (prof) prof_resamp += clock() - prof_c;
        free(cfo_corrected);

        /* Decode the candidate frame.
           dsp_complex_t and locate_complex_t share the same underlying type
           (float _Complex / _Fcomplex) — cast is safe on all platforms. */
        prof_c = clock();
        rc = locate_droneid((const locate_complex_t *)resampled, n_out,
                             flags, &result);
        if (prof) prof_locate += clock() - prof_c;
        free(resampled);

        if (rc != LOCATE_OK) continue;   /* skip undecoded candidates */

        /* Append to output array */
        tmp = (telemetry_result_t *)realloc(
            *results_io,
            sizeof(telemetry_result_t) * (*count_io + 1));
        if (!tmp) return LOCATE_ERR_ALLOC;
        *results_io = tmp;
        (*results_io)[(*count_io)++] = result;
    }
    if (prof) fprintf(stderr,
        "[prof] candidates: %u  Welch=%.0f ms  resample=%.0f ms  locate=%.0f ms\n",
        n_cands, 1000.0*(double)prof_welch/CLOCKS_PER_SEC,
        1000.0*(double)prof_resamp/CLOCKS_PER_SEC,
        1000.0*(double)prof_locate/CLOCKS_PER_SEC);
    }

    return LOCATE_OK;
}

/* ── locate_droneid_file ─────────────────────────────────────────────────── */

int32_t locate_droneid_file(const char *path, double sample_rate,
                             uint32_t flags,
                             telemetry_result_t **results_out,
                             uint32_t *num_results_out) {
    FILE    *f;
    long     file_size;
    uint32_t total_samples;
    uint32_t chunk_samples;
    float   *buf = NULL;
    uint32_t samples_read = 0;
    telemetry_result_t *all_results = NULL;
    uint32_t total = 0;
    int32_t  final_rc = LOCATE_ERR_ZC_NOT_FOUND;

    if (!path || !results_out || !num_results_out)
        return LOCATE_ERR_INVALID_ARG;

    *results_out     = NULL;
    *num_results_out = 0;

    /* Open file — returns LOCATE_ERR_INVALID_ARG if missing */
    f = fopen(path, "rb");
    if (!f) return LOCATE_ERR_INVALID_ARG;

    fseek(f, 0, SEEK_END);
    file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (file_size <= 0) {
        fclose(f);
        return LOCATE_ERR_INVALID_ARG;
    }

    /* File is interleaved float32 I/Q pairs: 2 floats per complex sample */
    total_samples  = (uint32_t)(file_size / (2 * sizeof(float)));

    /* 500 ms chunk */
    chunk_samples  = (uint32_t)(0.5 * sample_rate);
    if (chunk_samples == 0 || chunk_samples > total_samples)
        chunk_samples = total_samples;

    buf = (float *)malloc(sizeof(float) * chunk_samples * 2);
    if (!buf) {
        fclose(f);
        return LOCATE_ERR_ALLOC;
    }

    while (samples_read < total_samples) {
        const dsp_complex_t *cx;
        uint32_t to_read = chunk_samples;
        size_t   got;
        int32_t  rc;

        if (samples_read + to_read > total_samples)
            to_read = total_samples - samples_read;

        got = fread(buf, sizeof(float) * 2, to_read, f);
        if (got == 0) break;

        /* LOCATE_FLAG_CONJ: negate Q to undo a recorder spectral inversion. */
        if (flags & LOCATE_FLAG_CONJ) {
            uint32_t k;
            for (k = 0; k < (uint32_t)got; k++)
                buf[2 * k + 1] = -buf[2 * k + 1];
        }

        /* Reinterpret interleaved float32 pairs as complex samples */
        cx = (const dsp_complex_t *)buf;

        /* Run detection + decoding pipeline on this chunk */
        rc = detect_and_decode_chunk(cx, (uint32_t)got, sample_rate, flags,
                                      &all_results, &total);
        if (rc == LOCATE_ERR_ALLOC) {
            free(all_results);
            free(buf);
            fclose(f);
            return LOCATE_ERR_ALLOC;
        }

        samples_read += (uint32_t)got;
    }

    free(buf);
    fclose(f);

    if (total > 0) {
        *results_out     = all_results;
        *num_results_out = total;
        final_rc         = LOCATE_OK;
    } else {
        free(all_results);
    }

    return final_rc;
}

/* ── locate_free_results ─────────────────────────────────────────────────── */

void locate_free_results(telemetry_result_t *results, uint32_t count) {
    (void)count;   /* telemetry_result_t contains no heap pointers */
    free(results);
}
