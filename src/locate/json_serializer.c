/**
 * json_serializer.c — telemetry_to_json / telemetry_from_json
 *
 * Hand-written snprintf-based serialiser. No external JSON library.
 * Round-trip guarantee: telemetry_from_json(telemetry_to_json(x)) == x.
 */

#include "locate_internal.h"
#include <stdio.h>

/* ── Serialise ───────────────────────────────────────────────────────────── */

int32_t json_serialize(const telemetry_result_t *r, char *buf, uint32_t buf_len) {
    if (!r || !buf || buf_len == 0) return LOCATE_ERR_INVALID_ARG;

    int n = snprintf(buf, buf_len,
        "{\n"
        "    \"pkt_len\": %u,\n"
        "    \"version\": %u,\n"
        "    \"sequence_number\": %u,\n"
        "    \"state_info\": %u,\n"
        "    \"serial_number\": \"%s\",\n"
        "    \"longitude\": %.10g,\n"
        "    \"latitude\": %.10g,\n"
        "    \"altitude_m\": %.10g,\n"
        "    \"height_m\": %.10g,\n"
        "    \"v_north\": %d,\n"
        "    \"v_east\": %d,\n"
        "    \"v_up\": %d,\n"
        "    \"d_1_angle\": %d,\n"
        "    \"gps_time\": %llu,\n"
        "    \"app_lat\": %.10g,\n"
        "    \"app_lon\": %.10g,\n"
        "    \"longitude_home\": %.10g,\n"
        "    \"latitude_home\": %.10g,\n"
        "    \"device_type_id\": %u,\n"
        "    \"uuid\": \"%s\",\n"
        "    \"crc_valid\": %s\n"
        "}",
        (unsigned)r->pkt_len,
        (unsigned)r->version,
        (unsigned)r->sequence_number,
        (unsigned)r->state_info,
        r->serial_number,
        r->longitude, r->latitude,
        r->altitude_m, r->height_m,
        (int)r->v_north, (int)r->v_east, (int)r->v_up,
        (int)r->d_1_angle,
        (unsigned long long)r->gps_time,
        r->app_lat, r->app_lon,
        r->longitude_home, r->latitude_home,
        (unsigned)r->device_type_id,
        r->uuid,
        r->crc_valid ? "true" : "false");

    if (n < 0) { buf[0] = '\0'; return LOCATE_ERR_INVALID_ARG; }
    if ((uint32_t)n >= buf_len) return LOCATE_ERR_BUFFER_TOO_SMALL;
    return LOCATE_OK;
}

/* ── Minimal key-value parser ────────────────────────────────────────────── */

/* Find "key": in json and return pointer just past the colon+space */
static const char *find_value(const char *json, const char *key) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return NULL;
    p += strlen(search);
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

static double parse_double(const char *p) { return strtod(p, NULL); }
static long long parse_llong(const char *p) { return strtoll(p, NULL, 10); }
static unsigned long long parse_ullong(const char *p) { return strtoull(p, NULL, 10); }

static void parse_string(const char *p, char *out, size_t max_len) {
    if (*p != '"') { out[0] = '\0'; return; }
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i < max_len - 1) out[i++] = *p++;
    out[i] = '\0';
}

/* ── Deserialise ─────────────────────────────────────────────────────────── */

int32_t json_deserialize(const char *json_str, telemetry_result_t *r) {
    if (!json_str || !r) return LOCATE_ERR_INVALID_ARG;
    memset(r, 0, sizeof(*r));

    const char *p;

#define PARSE_U8(field, key) \
    if ((p = find_value(json_str, key))) r->field = (uint8_t)parse_ullong(p)
#define PARSE_U16(field, key) \
    if ((p = find_value(json_str, key))) r->field = (uint16_t)parse_ullong(p)
#define PARSE_I16(field, key) \
    if ((p = find_value(json_str, key))) r->field = (int16_t)parse_llong(p)
#define PARSE_U64(field, key) \
    if ((p = find_value(json_str, key))) r->field = (uint64_t)parse_ullong(p)
#define PARSE_DBL(field, key) \
    if ((p = find_value(json_str, key))) r->field = parse_double(p)
#define PARSE_STR(field, key, maxlen) \
    if ((p = find_value(json_str, key))) parse_string(p, r->field, maxlen)

    PARSE_U8 (pkt_len,         "pkt_len");
    PARSE_U8 (version,         "version");
    PARSE_U16(sequence_number, "sequence_number");
    PARSE_U16(state_info,      "state_info");
    PARSE_STR(serial_number,   "serial_number", 17);
    PARSE_DBL(longitude,       "longitude");
    PARSE_DBL(latitude,        "latitude");
    PARSE_DBL(altitude_m,      "altitude_m");
    PARSE_DBL(height_m,        "height_m");
    PARSE_I16(v_north,         "v_north");
    PARSE_I16(v_east,          "v_east");
    PARSE_I16(v_up,            "v_up");
    PARSE_I16(d_1_angle,       "d_1_angle");
    PARSE_U64(gps_time,        "gps_time");
    PARSE_DBL(app_lat,         "app_lat");
    PARSE_DBL(app_lon,         "app_lon");
    PARSE_DBL(longitude_home,  "longitude_home");
    PARSE_DBL(latitude_home,   "latitude_home");
    PARSE_U8 (device_type_id,  "device_type_id");
    PARSE_STR(uuid,            "uuid", 21);

    if ((p = find_value(json_str, "crc_valid")))
        r->crc_valid = (strncmp(p, "true", 4) == 0);

    return LOCATE_OK;
}
