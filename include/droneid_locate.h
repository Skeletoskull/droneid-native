#ifndef DRONEID_LOCATE_H
#define DRONEID_LOCATE_H

/**
 * droneid_locate.h — Location library public API
 *
 * Full DroneID telemetry decoding from a detected candidate frame.
 * Accepts complex float32 samples at 15.36 MHz (as returned by droneid_detect),
 * returns a fully decoded telemetry struct.
 *
 * All functions are stateless — no context lifecycle management required.
 * Thread-safe: concurrent calls with distinct buffers are safe.
 */

#include <stdint.h>
#include <stdbool.h>
#include <complex.h>

/* ── Complex type definition ──────────────────────────────────────────────── */
/* Define locate_complex_t as the working complex float type. On MSVC, use _Fcomplex.
   On GCC/Clang, use float _Complex.                                        */
#ifdef _MSC_VER
   typedef _Fcomplex locate_complex_t;
#else
   typedef float _Complex locate_complex_t;
#endif

/* ── Export macro ─────────────────────────────────────────────────────────── */
#ifdef _WIN32
#  ifdef DRONEID_LOCATE_EXPORTS
#    define LOCATE_API __declspec(dllexport)
#  else
#    define LOCATE_API __declspec(dllimport)
#  endif
#  define LOCATE_CALL __cdecl
#else
#  define LOCATE_API  __attribute__((visibility("default")))
#  define LOCATE_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ── Status codes ─────────────────────────────────────────────────────────── */
#define LOCATE_OK                    ((int32_t)  0)
#define LOCATE_ERR_INVALID_ARG       ((int32_t) -1)
#define LOCATE_ERR_ZC_NOT_FOUND      ((int32_t) -2)
#define LOCATE_ERR_CRC_FAIL          ((int32_t) -3)
#define LOCATE_ERR_BUFFER_TOO_SMALL  ((int32_t) -4)
#define LOCATE_ERR_ALLOC             ((int32_t) -5)

/* ── Flags ────────────────────────────────────────────────────────────────── */
/** Use 8-symbol CP table and ZC symbol indices [2, 4] for legacy drones */
#define LOCATE_FLAG_LEGACY           (0x0001u)
/** Fast path: skip the 599-root ZC search and assume the standard DroneID roots
 *  (600 at the first ZC symbol, 147 at the second). The CRC then validates the
 *  frame. Removes the dominant per-candidate decode cost and is ideal for
 *  brute-forcing known DroneID channels in a crowded band. */
#define LOCATE_FLAG_ASSUME_ZC        (0x0002u)
/** Conjugate the input I/Q (negate Q) before processing — undoes a recorder
 *  spectral inversion / I-Q swap. Honoured by locate_droneid_file (raw-IQ ingest);
 *  locate_droneid() takes already-detected candidates and ignores it. */
#define LOCATE_FLAG_CONJ             (0x0004u)

/* ── Telemetry result struct ──────────────────────────────────────────────── */
/**
 * Decoded DroneID telemetry from a single DUML payload (91 bytes).
 * Field layout mirrors the Python struct.unpack("<BBBHH16siihhhhhhQiiiiBB20sH").
 */
typedef struct {
    uint8_t  pkt_len;           /**< DUML packet length byte */
    uint8_t  version;           /**< Protocol version */
    uint16_t sequence_number;   /**< Packet sequence counter */
    uint16_t state_info;        /**< Bitmask: alt_valid, gps_valid, in_air, motor_on, etc. */
    char     serial_number[17]; /**< 16-byte drone serial number, null-terminated */
    double   longitude;         /**< Drone longitude, decimal degrees */
    double   latitude;          /**< Drone latitude, decimal degrees */
    double   altitude_m;        /**< Altitude above sea level, metres */
    double   height_m;          /**< Height above ground, metres */
    int16_t  v_north;           /**< Northward velocity, cm/s */
    int16_t  v_east;            /**< Eastward velocity, cm/s */
    int16_t  v_up;              /**< Upward velocity, cm/s */
    int16_t  d_1_angle;         /**< Heading angle */
    uint64_t gps_time;          /**< GPS timestamp */
    double   app_lat;           /**< Operator (app) latitude, decimal degrees */
    double   app_lon;           /**< Operator (app) longitude, decimal degrees */
    double   longitude_home;    /**< Home point longitude, decimal degrees */
    double   latitude_home;     /**< Home point latitude, decimal degrees */
    uint8_t  device_type_id;    /**< DJI drone type ID */
    char     uuid[21];          /**< 20-byte UUID string, null-terminated */
    bool     crc_valid;         /**< true if CRC16 over bytes 0–88 matches bytes 89–90 */
} telemetry_result_t;

/* ── API functions ────────────────────────────────────────────────────────── */

/**
 * Decode a candidate frame into full drone telemetry.
 *
 * @param candidate_samples  Complex float32 samples at 15.36 MHz
 *                           (as returned in detection_result_t.candidate_samples)
 * @param num_samples        Number of complex samples
 * @param flags              Bitfield of LOCATE_FLAG_* values (0 for defaults)
 * @param result_out         Output: caller-allocated telemetry_result_t to fill
 *
 * @return LOCATE_OK on success, or a negative error code on failure.
 */
LOCATE_API int32_t LOCATE_CALL locate_droneid(
    const locate_complex_t *candidate_samples,
    uint32_t                num_samples,
    uint32_t                flags,
    telemetry_result_t     *result_out);

/**
 * Read a raw binary IQ file, detect all DroneID frames, decode each one,
 * and return an array of telemetry structs.
 *
 * @param path            Path to the IQ file (interleaved little-endian float32)
 * @param sample_rate     Sample rate the file was recorded at (Hz)
 * @param flags           Bitfield of LOCATE_FLAG_* values
 * @param results_out     Output: pointer to allocated array of telemetry_result_t
 * @param num_results_out Output: number of results
 *
 * @return LOCATE_OK, or a negative error code on failure.
 *
 * Caller must free results with locate_free_results().
 */
LOCATE_API int32_t LOCATE_CALL locate_droneid_file(
    const char         *path,
    double              sample_rate,
    uint32_t            flags,
    telemetry_result_t **results_out,
    uint32_t           *num_results_out);

/**
 * Release memory allocated by locate_droneid_file().
 *
 * @param results  Pointer returned in results_out (may be NULL)
 * @param count    Value returned in num_results_out
 */
LOCATE_API void LOCATE_CALL locate_free_results(
    telemetry_result_t *results,
    uint32_t            count);

/**
 * Serialise a telemetry struct to a null-terminated JSON string.
 *
 * @param result   Telemetry struct to serialise
 * @param buf      Caller-supplied output buffer
 * @param buf_len  Size of buf in bytes
 *
 * @return LOCATE_OK on success, LOCATE_ERR_BUFFER_TOO_SMALL if buf is too small
 *         (a truncated null-terminated string is still written).
 */
LOCATE_API int32_t LOCATE_CALL telemetry_to_json(
    const telemetry_result_t *result,
    char                     *buf,
    uint32_t                  buf_len);

/**
 * Parse a JSON string (as produced by telemetry_to_json) back into a struct.
 *
 * @param json_str  Null-terminated JSON string
 * @param result    Output: caller-allocated telemetry_result_t to fill
 *
 * @return LOCATE_OK on success, LOCATE_ERR_INVALID_ARG on parse failure.
 */
LOCATE_API int32_t LOCATE_CALL telemetry_from_json(
    const char         *json_str,
    telemetry_result_t *result);

/**
 * Compute reflected CRC-16 (polynomial 0x11021, initial value 0x3692, LSB-first)
 * over len bytes of data.
 *
 * @param data  Input byte array
 * @param len   Number of bytes
 * @return      16-bit CRC value
 */
LOCATE_API uint16_t LOCATE_CALL compute_crc16(
    const uint8_t *data,
    uint32_t       len);

/**
 * Write the library's semantic version into caller-supplied pointers.
 * Any pointer may be NULL if that component is not needed.
 */
LOCATE_API void LOCATE_CALL locate_version(
    uint32_t *major,
    uint32_t *minor,
    uint32_t *patch);

/**
 * Self-test of the built-in LTE turbo FEC engine (no SDR data needed).
 * Encodes a random K=1408 block, adds AWGN at ~1 dB Es/N0, and turbo-decodes.
 *
 * @return 0 if the message was fully recovered (FEC working),
 *         a positive bit-error count if it failed to converge,
 *         or a negative value on internal allocation failure.
 */
LOCATE_API int32_t LOCATE_CALL locate_turbo_self_test(void);

#ifdef __cplusplus
}
#endif
#endif /* DRONEID_LOCATE_H */
