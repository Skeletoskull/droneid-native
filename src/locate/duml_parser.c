/**
 * duml_parser.c — DUML payload parser
 *
 * Unpacks the 91-byte DUML binary struct into telemetry_result_t.
 * Mirrors DroneIDPacket.__init__() from droneid_packet.py.
 *
 * Binary layout (little-endian):
 *   0:1   uint8   pkt_len
 *   1:2   uint8   unk (ignored)
 *   2:3   uint8   version
 *   3:5   uint16  sequence_number
 *   5:7   uint16  state_info
 *   7:23  char[16] serial_number
 *  23:27  int32   longitude_raw  → / 174533.0
 *  27:31  int32   latitude_raw   → / 174533.0
 *  31:33  int16   altitude_raw   → / 3.281
 *  33:35  int16   height_raw     → / 3.281
 *  35:37  int16   v_north
 *  37:39  int16   v_east
 *  39:41  int16   v_up
 *  41:43  int16   d_1_angle
 *  43:51  uint64  gps_time
 *  51:55  int32   app_lat_raw    → / 174533.0
 *  55:59  int32   app_lon_raw    → / 174533.0
 *  59:63  int32   longitude_home_raw → / 174533.0
 *  63:67  int32   latitude_home_raw  → / 174533.0
 *  67:68  uint8   device_type_id
 *  68:69  uint8   uuid_len (ignored)
 *  69:89  char[20] uuid
 *  89:91  uint16  crc16
 */

#include "locate_internal.h"

/* Little-endian read helpers */
static uint16_t read_u16le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static int16_t read_i16le(const uint8_t *p) {
    return (int16_t)read_u16le(p);
}
static uint32_t read_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t read_i32le(const uint8_t *p) {
    return (int32_t)read_u32le(p);
}
static uint64_t read_u64le(const uint8_t *p) {
    uint64_t lo = read_u32le(p);
    uint64_t hi = read_u32le(p + 4);
    return lo | (hi << 32);
}

void duml_parse(const uint8_t *payload, telemetry_result_t *out) {
    out->pkt_len         = payload[0];
    out->version         = payload[2];
    out->sequence_number = read_u16le(payload + 3);
    out->state_info      = read_u16le(payload + 5);

    /* Serial number: 16 bytes, null-terminate */
    memcpy(out->serial_number, payload + 7, 16);
    out->serial_number[16] = '\0';

    out->longitude      = read_i32le(payload + 23) / 174533.0;
    out->latitude       = read_i32le(payload + 27) / 174533.0;
    out->altitude_m     = read_i16le(payload + 31) / 3.281;
    out->height_m       = read_i16le(payload + 33) / 3.281;
    out->v_north        = read_i16le(payload + 35);
    out->v_east         = read_i16le(payload + 37);
    out->v_up           = read_i16le(payload + 39);
    out->d_1_angle      = read_i16le(payload + 41);
    out->gps_time       = read_u64le(payload + 43);
    out->app_lat        = read_i32le(payload + 51) / 174533.0;
    out->app_lon        = read_i32le(payload + 55) / 174533.0;
    out->longitude_home = read_i32le(payload + 59) / 174533.0;
    out->latitude_home  = read_i32le(payload + 63) / 174533.0;
    out->device_type_id = payload[67];

    /* UUID: 20 bytes, null-terminate */
    memcpy(out->uuid, payload + 69, 20);
    out->uuid[20] = '\0';

    /* CRC validation */
    uint16_t crc_packet = read_u16le(payload + 89);
    uint16_t crc_calc   = compute_crc16(payload, 89);
    out->crc_valid = (crc_calc == crc_packet);
}
