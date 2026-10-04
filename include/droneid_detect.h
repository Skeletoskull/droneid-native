#ifndef DRONEID_DETECT_H
#define DRONEID_DETECT_H

/**
 * droneid_detect.h — Detection library public API
 *
 * Lightweight continuous DroneID burst detection.
 * Accepts raw IQ samples, returns candidate frames resampled to 15.36 MHz.
 *
 * All functions are stateless — no context lifecycle management required.
 * Thread-safe: concurrent calls with distinct buffers are safe.
 */

#include <stdint.h>
#include <stdbool.h>
#include <complex.h>

/* ── Complex type definition ──────────────────────────────────────────────── */
/* Define detect_complex_t as the working complex float type. On MSVC, use _Fcomplex.
   On GCC/Clang, use float _Complex.                                        */
#ifdef _MSC_VER
   typedef _Fcomplex detect_complex_t;
#else
   typedef float _Complex detect_complex_t;
#endif

/* ── Export macro ─────────────────────────────────────────────────────────── */
#ifdef _WIN32
#  ifdef DRONEID_DETECT_EXPORTS
#    define DETECT_API __declspec(dllexport)
#  else
#    define DETECT_API __declspec(dllimport)
#  endif
#  define DETECT_CALL __cdecl
#else
#  define DETECT_API  __attribute__((visibility("default")))
#  define DETECT_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ── Status codes ─────────────────────────────────────────────────────────── */
#define DETECT_OK                  ((int32_t)  0)
#define DETECT_ERR_INVALID_ARG     ((int32_t) -1)
#define DETECT_ERR_ALLOC           ((int32_t) -2)
#define DETECT_STATUS_NO_FRAMES    ((int32_t)  1)

/* ── Flags ────────────────────────────────────────────────────────────────── */
/** Accept legacy DroneID frame durations (565–600 µs instead of 630–665 µs) */
#define DETECT_FLAG_LEGACY         (0x0001u)
/** Conjugate the input I/Q (negate Q) before processing — undoes a recorder
 *  spectral inversion / I-Q swap (e.g. some 100 Msps capture paths).
 *  Same bit (0x0004) as LOCATE_FLAG_CONJ so one shared `flags` works through the
 *  LabVIEW bridge, and distinct from LOCATE_FLAG_ASSUME_ZC (0x0002). */
#define DETECT_FLAG_CONJ           (0x0004u)

/* ── Result struct ────────────────────────────────────────────────────────── */
/**
 * Detection result for a single DroneID frame candidate.
 * Memory for candidate_samples is allocated by the library.
 * Free with detect_free_results().
 */
typedef struct {
    bool              detected;              /**< true if a DroneID frame was found */
    detect_complex_t *candidate_samples;     /**< resampled to 15.36 MHz; caller owns */
    uint32_t          num_candidate_samples; /**< length of candidate_samples array */
    double            start_time_s;          /**< frame start relative to buffer start (s) */
    double            duration_s;            /**< frame duration in seconds */
    float             cfo_hz;               /**< estimated carrier frequency offset (Hz) */
} detection_result_t;

/* ── API functions ────────────────────────────────────────────────────────── */

/**
 * Scan an IQ buffer (interleaved float32 I/Q pairs) for DroneID frame candidates.
 *
 * @param iq_interleaved  Interleaved I/Q samples: [I0, Q0, I1, Q1, ...]
 * @param num_samples     Number of complex samples (length of iq_interleaved / 2)
 * @param sample_rate     SDR capture sample rate in Hz (e.g. 50e6)
 * @param flags           Bitfield of DETECT_FLAG_* values (0 for defaults)
 * @param results_out     Output: pointer to allocated array of detection_result_t
 * @param num_results_out Output: number of results in results_out
 *
 * @return DETECT_OK on success, DETECT_STATUS_NO_FRAMES if no frames found,
 *         or a negative error code on failure.
 *
 * Caller must free results with detect_free_results().
 */
DETECT_API int32_t DETECT_CALL detect_droneid(
    const float       *iq_interleaved,
    uint32_t           num_samples,
    double             sample_rate,
    uint32_t           flags,
    detection_result_t **results_out,
    uint32_t          *num_results_out);

/**
 * Same as detect_droneid() but accepts 64-bit double IQ samples.
 * Internally converts to float32 before processing.
 *
 * Use this from LabVIEW when the acquisition delivers double-precision IQ.
 */
DETECT_API int32_t DETECT_CALL detect_droneid_d(
    const double      *iq_interleaved,
    uint32_t           num_samples,
    double             sample_rate,
    uint32_t           flags,
    detection_result_t **results_out,
    uint32_t          *num_results_out);

/**
 * Read a raw binary IQ file (interleaved little-endian float32) and return
 * all detected DroneID frame candidates.
 *
 * @param path            Path to the IQ file
 * @param sample_rate     Sample rate the file was recorded at (Hz)
 * @param flags           Bitfield of DETECT_FLAG_* values
 * @param results_out     Output: pointer to allocated array of detection_result_t
 * @param num_results_out Output: number of results
 *
 * @return DETECT_OK, DETECT_STATUS_NO_FRAMES, or a negative error code.
 */
DETECT_API int32_t DETECT_CALL detect_droneid_file(
    const char        *path,
    double             sample_rate,
    uint32_t           flags,
    detection_result_t **results_out,
    uint32_t          *num_results_out);

/**
 * Release memory allocated by detect_droneid() / detect_droneid_d() /
 * detect_droneid_file().
 *
 * @param results  Pointer returned in results_out (may be NULL)
 * @param count    Value returned in num_results_out
 */
DETECT_API void DETECT_CALL detect_free_results(
    detection_result_t *results,
    uint32_t            count);

/**
 * Write the library's semantic version into caller-supplied pointers.
 * Any pointer may be NULL if that component is not needed.
 */
DETECT_API void DETECT_CALL detect_version(
    uint32_t *major,
    uint32_t *minor,
    uint32_t *patch);

#ifdef __cplusplus
}
#endif
#endif /* DRONEID_DETECT_H */
