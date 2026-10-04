#ifndef LOCATE_INTERNAL_H
#define LOCATE_INTERNAL_H

/**
 * locate_internal.h — Internal interface for location library components
 *
 * Every .c file in src/locate/ includes this header.
 * Defining DRONEID_LOCATE_EXPORTS here ensures all translation units
 * see __declspec(dllexport) on Windows without repeating the define.
 */

#ifndef DRONEID_LOCATE_EXPORTS
#  define DRONEID_LOCATE_EXPORTS
#endif

#include "../../include/droneid_locate.h"
#include "../dsp/dsp_internal.h"
#include <stdint.h>
#include <stdbool.h>
#include <complex.h>
#include <stdlib.h>
#include <string.h>

#define DUML_PAYLOAD_LEN  91
#define DUML_CRC_OFFSET   89

/* Data symbols (ZC symbols 3 and 5 excluded) */
static const int DATA_SYMBOLS_STD[] = {0, 1, 2, 4, 6, 7, 8};
static const int DATA_SYMBOLS_LEG[] = {0, 1, 3, 5, 6, 7};
#define N_DATA_SYMBOLS_STD 7
#define N_DATA_SYMBOLS_LEG 6

/* QPSK rotation tables (from qpsk.py) */
static const int QPSK_TO_BITS[4][4] = {
    {2, 3, 1, 0},   /* 0°   */
    {0, 2, 3, 1},   /* +90° */
    {1, 0, 2, 3},   /* +180° */
    {3, 1, 0, 2},   /* +270° */
};

/* CP lengths at 15.36 MHz */
static const int CP_LENGTHS_STD[9] = {80, 72, 72, 72, 72, 72, 72, 72, 80};
static const int CP_LENGTHS_LEG[8] = {80, 72, 72, 72, 72, 72, 72, 80};

/* ── Function declarations ────────────────────────────────────────────────── */

/* duml_parser.c */
void duml_parse(const uint8_t *payload, telemetry_result_t *out);

/* crc16.c — declared in droneid_locate.h */

/* ofdm_demod.c */
typedef struct {
    dsp_complex_t symbols[9][DSP_NCARRIERS]; /* up to 9 symbols × 601 subcarriers */
    int    n_symbols;
    float  ffo_hz;
    int    start_sample;
} ofdm_frame_t;

int32_t ofdm_demodulate(const dsp_complex_t *samples, uint32_t n,
                         uint32_t flags, int do_fine, ofdm_frame_t *frame_out);

/* zc_sync.c */
int32_t zc_sync_and_equalize(ofdm_frame_t *frame, uint32_t flags,
                               dsp_complex_t equalized[][DSP_NCARRIERS],
                               int *n_data_symbols_out);
bool zc_symbol_matches_147(const dsp_complex_t *symbol_f);  /* cheap root-147 gate */

/* qpsk_decode.c */
int32_t qpsk_decode(dsp_complex_t equalized[][DSP_NCARRIERS],
                     int n_data_symbols, bool legacy,
                     uint8_t *payload_out);

/* json_serializer.c */
int32_t json_serialize(const telemetry_result_t *result, char *buf, uint32_t buf_len);
int32_t json_deserialize(const char *json_str, telemetry_result_t *result);

#endif /* LOCATE_INTERNAL_H */
