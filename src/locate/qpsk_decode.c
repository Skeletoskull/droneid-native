/**
 * qpsk_decode.c — QPSK demodulation and Gold descrambling
 *
 * Mirrors Decoder.raw_data_to_symbol_bits() and Decoder.magic() from qpsk.py.
 * Input is raw (non-equalized) OFDM symbols — Python's Decoder operates on
 * unequalized frequency-domain symbols from get_symbol_data(skip_zc=True).
 * Turbo de-rate-matching is delegated to turbo_deratematch.c (rm_turbo_rx()).
 */

#include "locate_internal.h"
#include "turbo_deratematch.h"
#include "turbo_decode.h"
#include <math.h>

/* ── QPSK quadrant → 2-bit symbol ────────────────────────────────────────── */

static int get_symbol_bits(dsp_complex_t sym, int phase_correction) {
    int quadrant;
    if      (crealf(sym) >= 0 && cimagf(sym) >= 0) quadrant = 0;
    else if (crealf(sym) >= 0 && cimagf(sym) <  0) quadrant = 1;
    else if (crealf(sym) <  0 && cimagf(sym) <  0) quadrant = 2;
    else                                             quadrant = 3;
    return QPSK_TO_BITS[phase_correction][quadrant];
}

/* ── Extract bits from symbols ───────────────────────────────────────────── */
/*
 * Mirrors Decoder.magic() bit extraction:
 *   bits = np.delete(bits, 300, 1)       # remove DC
 *   bits = np.repeat(bits, 2, axis=1)    # double each symbol
 *   bits &= np.tile([1, 2], 600)         # alternating LSB/MSB mask
 *   bits = bits > 0                       # to bool
 * For symbol s: produces [s&1, (s>>1)&1] per subcarrier.
 */
static void symbols_to_bits(dsp_complex_t syms[][DSP_NCARRIERS],
                              int n_syms, int phase,
                              uint8_t *bits_out, uint32_t *n_bits_out) {
    uint32_t idx = 0;
    for (int si = 0; si < n_syms; si++) {
        for (int k = 0; k < DSP_NCARRIERS; k++) {
            if (k == DSP_NCARRIERS_HALF) continue; /* skip DC (index 300) */
            int sym = get_symbol_bits(syms[si][k], phase);
            bits_out[idx++] = (uint8_t)(sym & 1);
            bits_out[idx++] = (uint8_t)((sym >> 1) & 1);
        }
    }
    *n_bits_out = idx;
}

/* ── Gold descrambling ───────────────────────────────────────────────────── */

static void gold_descramble(uint8_t *bits, uint32_t n_bits) {
    uint8_t *gs = (uint8_t *)malloc(n_bits);
    if (!gs) return;
    dsp_gold_sequence(DSP_GOLD_SEED, DSP_GOLD_NC, n_bits, gs);
    for (uint32_t i = 0; i < n_bits; i++) bits[i] ^= gs[i];
    free(gs);
}

/* ── Assemble bytes from bits (big-endian, MSB first) ────────────────────── */

static void bits_to_bytes(const int8_t *bits, uint32_t n_bits,
                           uint8_t *bytes_out, uint32_t *n_bytes_out) {
    uint32_t n_bytes = n_bits / 8;
    for (uint32_t i = 0; i < n_bytes; i++) {
        uint8_t b = 0;
        for (int j = 0; j < 8; j++) {
            if (bits[i * 8 + j] > 0)
                b |= (uint8_t)(1u << (7 - j));
        }
        bytes_out[i] = b;
    }
    *n_bytes_out = n_bytes;
}

/* ── Public entry point ──────────────────────────────────────────────────── */

int32_t qpsk_decode(dsp_complex_t syms[][DSP_NCARRIERS],
                     int n_data_symbols, bool legacy,
                     uint8_t *payload_out) {
    /* 7 symbols × 600 subcarriers × 2 bits/subcarrier = 8400 bits max */
    uint32_t max_bits = (uint32_t)n_data_symbols * (DSP_NCARRIERS - 1) * 2;
    uint8_t *raw_bits = (uint8_t *)malloc(max_bits);
    if (!raw_bits) return LOCATE_ERR_ALLOC;

    for (int phase = 0; phase < 4; phase++) {
        uint32_t n_bits = 0;
        symbols_to_bits(syms, n_data_symbols, phase, raw_bits, &n_bits);

        /* Skip first data symbol bits in standard mode (Gold check symbol).
           Mirrors Python magic(): if len(bits[0:]) > 7200: all_bits = bits[1:] */
        bool has_gold_sym0 = !legacy && (n_bits > 1200);
        uint8_t *all_bits;
        uint32_t all_bits_len;

        if (has_gold_sym0) {
            uint32_t sym0_bits = (DSP_NCARRIERS - 1) * 2; /* 1200 bits */
            all_bits     = raw_bits + sym0_bits;
            all_bits_len = n_bits   - sym0_bits;
        } else {
            all_bits     = raw_bits;
            all_bits_len = n_bits;
        }

        /* Gold descramble */
        uint8_t *descrambled = (uint8_t *)malloc(all_bits_len);
        if (!descrambled) { free(raw_bits); return LOCATE_ERR_ALLOC; }
        memcpy(descrambled, all_bits, all_bits_len);
        gold_descramble(descrambled, all_bits_len);

        /* Extract systematic stream at offset 4148, length 1412.
           Apply cyclic extension if needed (mirrors Python's np.concatenate). */
        const uint32_t offset  = 4148;
        const uint32_t sys_len = 1412;

        uint8_t       *src      = NULL;
        uint8_t       *cyclic   = NULL;
        bool           owns_src = false;

        if (all_bits_len >= offset + sys_len) {
            src = descrambled + offset;
        } else {
            uint32_t cyclic_len = all_bits_len * 2;
            if (cyclic_len < offset + sys_len) { free(descrambled); continue; }
            cyclic = (uint8_t *)malloc(cyclic_len);
            if (!cyclic) { free(descrambled); free(raw_bits); return LOCATE_ERR_ALLOC; }
            memcpy(cyclic,              descrambled, all_bits_len);
            memcpy(cyclic + all_bits_len, descrambled, all_bits_len);
            src = cyclic + offset;
            owns_src = true;
        }

        /* Convert bits to ±1 for turbo de-rate-match */
        int8_t *sys_stream = (int8_t *)malloc(sys_len);
        if (!sys_stream) {
            if (owns_src) free(cyclic);
            free(descrambled); free(raw_bits);
            return LOCATE_ERR_ALLOC;
        }
        for (uint32_t i = 0; i < sys_len; i++)
            sys_stream[i] = (int8_t)(src[i] ? 1 : -1);

        if (owns_src) free(cyclic);
        free(descrambled);

        /* Turbo de-rate-match */
        int8_t   *decoded   = (int8_t *)malloc(sys_len);
        uint32_t  n_decoded = 0;
        if (!decoded) { free(sys_stream); free(raw_bits); return LOCATE_ERR_ALLOC; }
        rm_turbo_rx(sys_stream, sys_len, decoded, &n_decoded);
        free(sys_stream);

        /* Byte assembly and CRC check */
        uint32_t n_bytes = 0;
        bits_to_bytes(decoded, n_decoded, payload_out, &n_bytes);
        free(decoded);

        if (n_bytes >= DUML_PAYLOAD_LEN) {
            uint16_t crc_calc   = compute_crc16(payload_out, 89);
            uint16_t crc_packet = (uint16_t)payload_out[89] | ((uint16_t)payload_out[90] << 8);
            if (crc_calc == crc_packet) {
                free(raw_bits);
                return LOCATE_OK;
            }
        }
    }

    free(raw_bits);

    /* Hard-decision systematic-only decode failed on all 4 phases.
       Fall back to soft-decision + LTE turbo FEC, which recovers beacons
       corrupted by noise / co-channel interference (standard frames only). */
    return turbo_decode_dji(syms, n_data_symbols, legacy, payload_out);
}
