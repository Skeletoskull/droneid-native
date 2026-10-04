/**
 * droneid_bridge.h — LabVIEW-friendly bridge over droneid_detect + droneid_locate
 *
 * Single function that accepts raw IQ samples from the SDR and returns flat
 * scalar/string outputs directly usable by LabVIEW Call Library Function Nodes
 * without any struct layout concerns or pointer management.
 *
 * Usage in LabVIEW CLFN:
 *   Library:   droneid_bridge.dll
 *   Function:  droneid_process
 *   Thread:    Run in any thread
 *   Return:    Signed 32-bit Integer (status code)
 */

#ifndef DRONEID_BRIDGE_H
#define DRONEID_BRIDGE_H

#include <stdint.h>

#ifdef _WIN32
#  ifdef DRONEID_BRIDGE_EXPORTS
#    define BRIDGE_API __declspec(dllexport)
#  else
#    define BRIDGE_API __declspec(dllimport)
#  endif
#  define BRIDGE_CALL __cdecl
#else
#  define BRIDGE_API __attribute__((visibility("default")))
#  define BRIDGE_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ── Status codes ─────────────────────────────────────────────────────────── */
#define BRIDGE_OK               0   /* success, at least one frame decoded    */
#define BRIDGE_NO_FRAMES        1   /* detection ran OK but no DroneID found  */
#define BRIDGE_ERR_INVALID_ARG -1   /* NULL pointer or bad parameter          */
#define BRIDGE_ERR_DETECT      -2   /* detect DLL returned error              */
#define BRIDGE_ERR_ALLOC       -3   /* memory allocation failed               */

/* Maximum number of simultaneous DroneID frames returned */
#define BRIDGE_MAX_FRAMES       8

/**
 * Process a block of IQ samples through the full DroneID detect + decode pipeline.
 *
 * All outputs are flat scalars or fixed-size arrays — no pointers to manage.
 * Memory is allocated and freed entirely inside this function.
 *
 * LabVIEW CLFN parameter configuration:
 *
 *  #  Name                Type        LabVIEW data type           Pass
 *  1  iq_interleaved      Input       Array of DBL (1D)           Adapt to Type
 *  2  num_samples         Input       Unsigned 32-bit Integer     Value
 *  3  sample_rate         Input       Double                      Value
 *  4  flags               Input       Unsigned 32-bit Integer     Value
 *  5  num_frames_out      Output      Signed 32-bit Integer       Pointer to Value
 *  6  crc_valid_out       Output      Signed 32-bit Integer       Pointer to Value
 *  7  serial_out          Output      String (17 bytes min)       Pointer to Value
 *  8  latitude_out        Output      Double                      Pointer to Value
 *  9  longitude_out       Output      Double                      Pointer to Value
 * 10  altitude_m_out      Output      Double                      Pointer to Value
 * 11  height_m_out        Output      Double                      Pointer to Value
 * 12  app_lat_out         Output      Double                      Pointer to Value
 * 13  app_lon_out         Output      Double                      Pointer to Value
 * 14  v_north_out         Output      Signed 16-bit Integer       Pointer to Value
 * 15  v_east_out          Output      Signed 16-bit Integer       Pointer to Value
 * 16  sequence_number_out Output      Unsigned 16-bit Integer     Pointer to Value
 * 17  device_type_id_out  Output      Unsigned 8-bit Integer      Pointer to Value
 * 18  json_out            Output      String (4096 bytes min)     Pointer to Value
 * 19  json_buf_len        Input       Unsigned 32-bit Integer     Value
 *
 * @param iq_interleaved      1D array of interleaved I/Q doubles from the SDR
 *                            Layout: [I0, Q0, I1, Q1, ...]
 *                            In LabVIEW: pass CDB[] directly (memory layout is identical)
 * @param num_samples         Number of complex samples (= Array Size of CDB[] in LabVIEW)
 * @param sample_rate         SDR sample rate in Hz (e.g. 50000000.0)
 * @param flags               Shared DETECT_/LOCATE_ flag bitfield: 0 = standard,
 *                            0x1 = legacy frame format, 0x4 = conjugate input
 * @param num_frames_out      Number of DroneID frames detected (0 if none)
 * @param crc_valid_out       CRC result of first decoded frame:
 *                              1 = valid decode, 0 = decode failed CRC, -1 = no frame
 * @param serial_out          Serial number string of first frame (null-terminated, 17 bytes)
 * @param latitude_out        Drone latitude in decimal degrees (first frame)
 * @param longitude_out       Drone longitude in decimal degrees (first frame)
 * @param altitude_m_out      Drone altitude in metres ASL (first frame)
 * @param height_m_out        Drone height above ground in metres (first frame)
 * @param app_lat_out         Operator (app) latitude in decimal degrees (first frame)
 * @param app_lon_out         Operator (app) longitude in decimal degrees (first frame)
 * @param v_north_out         Northward velocity in cm/s (first frame)
 * @param v_east_out          Eastward velocity in cm/s (first frame)
 * @param sequence_number_out Packet sequence counter (first frame)
 * @param device_type_id_out  DJI device type ID (first frame)
 * @param json_out            Full JSON string of first decoded frame (null-terminated)
 * @param json_buf_len        Size of json_out buffer in bytes (use 4096)
 *
 * @return BRIDGE_OK (0)          At least one frame decoded
 *         BRIDGE_NO_FRAMES (1)   No DroneID frames found — outputs zeroed
 *         BRIDGE_ERR_* (< 0)     Error — outputs zeroed
 *
 * Output fields are zeroed/cleared when no frame is decoded.
 */
BRIDGE_API int32_t BRIDGE_CALL droneid_process(
    const double  *iq_interleaved,
    uint32_t       num_samples,
    double         sample_rate,
    uint32_t       flags,
    int32_t       *num_frames_out,
    int32_t       *crc_valid_out,
    char          *serial_out,
    double        *latitude_out,
    double        *longitude_out,
    double        *altitude_m_out,
    double        *height_m_out,
    double        *app_lat_out,
    double        *app_lon_out,
    int16_t       *v_north_out,
    int16_t       *v_east_out,
    uint16_t      *sequence_number_out,
    uint8_t       *device_type_id_out,
    char          *json_out,
    uint32_t       json_buf_len);

/**
 * Get library version.
 * Configure as: Return=Void, 3 parameters all Pointer to Value U32.
 */
BRIDGE_API void BRIDGE_CALL droneid_bridge_version(
    uint32_t *major, uint32_t *minor, uint32_t *patch);

#ifdef __cplusplus
}
#endif
#endif /* DRONEID_BRIDGE_H */
