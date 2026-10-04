/**
 * gold_sequence.c — Gold sequence generation (3GPP x1/x2 LFSR)
 *
 * Mirrors gold() from goldgen.py.
 * Parameters for DroneID: seed=0x12345678, Nc=1600.
 */

#include "dsp_internal.h"
#include <stdlib.h>
#include <string.h>

void dsp_gold_sequence(uint32_t seed, uint32_t Nc, uint32_t length, uint8_t *out) {
    uint32_t total = Nc + length + 31;

    /* Allocate state arrays on stack for typical sizes, heap for large */
    uint8_t *x1 = (uint8_t *)calloc(total, 1);
    uint8_t *x2 = (uint8_t *)calloc(total, 1);
    if (!x1 || !x2) {
        free(x1); free(x2);
        return;
    }

    /* Initialise x1: x1[0] = 1 */
    x1[0] = 1;

    /* Initialise x2 from seed */
    for (uint32_t n = 0; n < 32; n++) {
        x2[n] = (seed >> n) & 1u;
    }

    /* Run LFSRs */
    for (uint32_t n = 0; n < Nc + length; n++) {
        x1[n + 31] = (x1[n + 3] ^ x1[n]) & 1u;
        x2[n + 31] = (x2[n + 3] ^ x2[n + 2] ^ x2[n + 1] ^ x2[n]) & 1u;
    }

    /* Output: c[n] = x1[Nc+n] XOR x2[Nc+n] */
    for (uint32_t n = 0; n < length; n++) {
        out[n] = x1[Nc + n] ^ x2[Nc + n];
    }

    free(x1);
    free(x2);
}
