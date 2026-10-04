/**
 * packet_detector.c - STFT-based DroneID packet detection
 * Mirrors find_packet_candidate_time() from packetizer.py.
 */

#include "packet_detector.h"
#include "../dsp/dsp_internal.h"
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Median of the per-frame STFT peak power — robust noise-floor estimate that
   separates burst frames from noise frames. (mean-over-all-bins is far below
   the per-frame peak, so noise frames falsely read "above threshold".) */
static int pd_cmp_float(const void *a, const void *b) {
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa < fb) ? -1 : (fa > fb) ? 1 : 0;
}
static float pd_median(const float *x, uint32_t n) {
    /* Subsample to ~25k points: the median is only a noise-floor estimate,
       so this stays cheap even on multi-hundred-thousand-frame chunks. */
    uint32_t stride = (n > 25000u) ? (n / 25000u) : 1u;
    uint32_t i, j = 0, cnt = (n + stride - 1) / stride;
    float *c; float m;
    if (n == 0) return 0.0f;
    c = (float *)malloc(sizeof(float) * cnt);
    if (!c) return 0.0f;
    for (i = 0; i < n; i += stride) c[j++] = x[i];
    qsort(c, j, sizeof(float), pd_cmp_float);
    /* Median of per-frame peak power = robust noise floor for sparse,
       over-the-air captures. NOTE: very short, signal-dense clips (where
       bursts fill a large fraction of the frames) push the median into the
       signal level — a known trade-off, see process_chunk(). */
    m = c[j / 2];
    free(c);
    return m;
}

typedef struct {
    uint32_t start_sample;
    uint32_t end_sample;
} candidate_t;

static uint32_t find_candidates(const float *power, uint32_t n_frames,
                                 float threshold,
                                 uint32_t min_frames, uint32_t max_frames,
                                 candidate_t *cands, uint32_t max_cands) {
    uint32_t count = 0;
    bool in_run = false;
    uint32_t run_start = 0;

    for (uint32_t i = 0; i <= n_frames; i++) {
        bool above = (i < n_frames) && (power[i] > threshold);
        if (above && !in_run) {
            run_start = i;
            in_run = true;
        } else if (!above && in_run) {
            in_run = false;
            uint32_t run_len = i - run_start;
            if (run_len >= min_frames && run_len <= max_frames) {
                if (count < max_cands) {
                    uint32_t stft_step = DSP_STFT_NFFT / 2;
                    cands[count].start_sample = run_start * stft_step;
                    cands[count].end_sample   = (i - 1) * stft_step + (uint32_t)DSP_STFT_NFFT;
                    count++;
                }
            }
        }
    }
    return count;
}

static int32_t process_chunk(const dsp_complex_t *chunk, uint32_t n,
                              double sample_rate, uint32_t flags,
                              double chunk_start_time,
                              detection_result_t **results_out,
                              uint32_t *num_results_out) {
    uint32_t n_frames_max = (n / (DSP_STFT_NFFT / 2)) + 2;
    uint32_t n_frames = n_frames_max;
    if (n_frames == 0) return DETECT_STATUS_NO_FRAMES;

    float *power = (float *)malloc(sizeof(float) * n_frames_max);
    if (!power) return DETECT_ERR_ALLOC;

    dsp_stft_power(chunk, n, power, &n_frames);

    /* Noise floor = median of per-frame STFT peak power; threshold =
       1.15 x noise floor. This is what decodes real sparse over-the-air
       captures. A mean-over-all-bins floor suits short signal-dense clips
       better but merges bursts in sparse captures into one over-long run that
       the duration gate rejects, so the median is used. */
    float noise_floor = pd_median(power, n_frames);
    float threshold = PKT_STFT_THRESHOLD * noise_floor;

    float frame_dur_s = (float)(DSP_STFT_NFFT / 2) / (float)sample_rate;
    bool legacy = (flags & DETECT_FLAG_LEGACY) != 0;
    float min_dur_us = legacy ? PKT_MIN_DUR_LEG_US : PKT_MIN_DUR_STD_US;
    float max_dur_us = legacy ? PKT_MAX_DUR_LEG_US : PKT_MAX_DUR_STD_US;
    uint32_t min_frames_pkt = (uint32_t)(min_dur_us * 1e-6f / frame_dur_s);
    uint32_t max_frames_pkt = (uint32_t)(max_dur_us * 1e-6f / frame_dur_s) + 1;

    candidate_t cands[64];
    uint32_t n_cands = find_candidates(power, n_frames, threshold,
                                        min_frames_pkt, max_frames_pkt,
                                        cands, 64);
    free(power);

    if (n_cands == 0) return DETECT_STATUS_NO_FRAMES;

    uint32_t guard = (uint32_t)(PKT_GUARD_US * 1e-6 * sample_rate);
    detection_result_t *results = (detection_result_t *)calloc(n_cands, sizeof(detection_result_t));
    if (!results) return DETECT_ERR_ALLOC;
    uint32_t valid = 0;

    for (uint32_t ci = 0; ci < n_cands; ci++) {
        uint32_t s0 = (cands[ci].start_sample > guard) ? cands[ci].start_sample - guard : 0;
        uint32_t s1 = cands[ci].end_sample + guard;
        if (s1 > n) s1 = n;
        uint32_t seg_len = s1 - s0;

        float cfo_hz = 0.0f;
        bool band_ok = dsp_estimate_cfo(chunk + s0, seg_len, sample_rate,
                                         PKT_BW_MIN_HZ, PKT_BW_MAX_HZ, &cfo_hz);
        if (!band_ok) continue;

        dsp_complex_t *cfo_corrected = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * seg_len);
        if (!cfo_corrected) { free(results); return DETECT_ERR_ALLOC; }
        memcpy(cfo_corrected, chunk + s0, sizeof(dsp_complex_t) * seg_len);
        dsp_fshift(cfo_corrected, seg_len, -cfo_hz, sample_rate);

        uint32_t n_out = dsp_resample_output_length(seg_len, sample_rate, DSP_FS_TARGET);
        dsp_complex_t *resampled = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * n_out);
        if (!resampled) { free(cfo_corrected); free(results); return DETECT_ERR_ALLOC; }

        dsp_resample(cfo_corrected, seg_len, resampled, n_out, sample_rate, DSP_FS_TARGET);
        free(cfo_corrected);

        detection_result_t *r = &results[valid++];
        r->detected              = true;
        r->candidate_samples     = resampled;
        r->num_candidate_samples = n_out;
        r->start_time_s          = chunk_start_time + (double)s0 / sample_rate;
        r->duration_s            = (double)(cands[ci].end_sample - cands[ci].start_sample) / sample_rate;
        r->cfo_hz                = cfo_hz;
    }

    if (valid == 0) { free(results); return DETECT_STATUS_NO_FRAMES; }
    *results_out     = results;
    *num_results_out = valid;
    return DETECT_OK;
}

int32_t packet_detect(const dsp_complex_t *samples, uint32_t n,
                      double sample_rate, uint32_t flags,
                      detection_result_t **results_out,
                      uint32_t *num_results_out) {
    *results_out     = NULL;
    *num_results_out = 0;

    uint32_t chunk_samples = (uint32_t)(PKT_CHUNK_MS * 1e-3 * sample_rate);
    if (chunk_samples == 0) chunk_samples = n;

    detection_result_t *all_results = NULL;
    uint32_t total = 0;

    for (uint32_t offset = 0; offset < n; offset += chunk_samples) {
        uint32_t len = chunk_samples;
        if (offset + len > n) len = n - offset;

        detection_result_t *chunk_results = NULL;
        uint32_t chunk_count = 0;
        double chunk_time = (double)offset / sample_rate;

        int32_t rc = process_chunk(samples + offset, len, sample_rate, flags,
                                    chunk_time, &chunk_results, &chunk_count);
        if (rc == DETECT_OK && chunk_count > 0) {
            detection_result_t *tmp = (detection_result_t *)realloc(
                all_results, sizeof(detection_result_t) * (total + chunk_count));
            if (!tmp) {
                detect_free_results(chunk_results, chunk_count);
                detect_free_results(all_results, total);
                return DETECT_ERR_ALLOC;
            }
            all_results = tmp;
            memcpy(all_results + total, chunk_results,
                   sizeof(detection_result_t) * chunk_count);
            free(chunk_results);
            total += chunk_count;
        }
    }

    if (total == 0) return DETECT_STATUS_NO_FRAMES;
    *results_out     = all_results;
    *num_results_out = total;
    return DETECT_OK;
}
