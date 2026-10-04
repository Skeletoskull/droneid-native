/**
 * turbo_deratematch.c — 3GPP turbo rate-matching de-interleaver
 *
 * Implements rm_turbo_rx(), the 32-column RM interleaver de-permutation
 * used to recover the systematic turbo stream from the Gold-descrambled
 * bit sequence in the DroneID location pipeline.
 *
 * Design reference: design.md section 6.11
 * Python reference: rm_turbo_rx() in qpsk.py
 *
 * Algorithm:
 *   ncols  = 32
 *   nrows  = ceil(n_in / 32)
 *   n_dummy = ncols * nrows - n_in
 *
 *   Fill matrix[nrows][ncols] column by column using RM_PERM_TURBO:
 *     if RM_PERM_TURBO[col] < n_dummy:
 *         matrix[0][RM_PERM_TURBO[col]] = -1   (dummy)
 *         matrix[1..nrows-1][RM_PERM_TURBO[col]] = bits_in[p .. p+nrows-2]
 *     else:
 *         matrix[0..nrows-1][RM_PERM_TURBO[col]] = bits_in[p .. p+nrows-1]
 *
 *   bits_out = flatten(matrix)[n_dummy:]
 */

#include "turbo_deratematch.h"
#include <stdlib.h>
#include <string.h>

/* ── 3GPP 32-column turbo rate-matching permutation table ────────────────── */

const int RM_PERM_TURBO[32] = {
     0, 16,  8, 24,  4, 20, 12, 28,
     2, 18, 10, 26,  6, 22, 14, 30,
     1, 17,  9, 25,  5, 21, 13, 29,
     3, 19, 11, 27,  7, 23, 15, 31
};

/* ── De-interleaver ──────────────────────────────────────────────────────── */

void rm_turbo_rx(const int8_t *bits_in, uint32_t n_in,
                 int8_t *bits_out, uint32_t *n_out)
{
    const int ncols = 32;
    int nrows  = ((int)n_in + ncols - 1) / ncols;
    int n_dummy = ncols * nrows - (int)n_in;
    int total  = ncols * nrows;

    int8_t *matrix = (int8_t *)malloc((size_t)total);
    if (!matrix) {
        *n_out = 0;
        return;
    }
    memset(matrix, 0, (size_t)total);

    /* Fill matrix column by column via the RM_PERM_TURBO column permutation */
    int p = 0;
    for (int col = 0; col < ncols; col++) {
        int perm_col = RM_PERM_TURBO[col];
        if (perm_col < n_dummy) {
            /* Leading row of this column is a dummy placeholder */
            matrix[0 * ncols + perm_col] = -1;
            for (int row = 1; row < nrows; row++) {
                matrix[row * ncols + perm_col] = bits_in[p++];
            }
        } else {
            /* All rows of this column are real data */
            for (int row = 0; row < nrows; row++) {
                matrix[row * ncols + perm_col] = bits_in[p++];
            }
        }
    }

    /* Read out row-by-row, skipping the n_dummy leading dummy entries */
    uint32_t out_idx = 0;
    for (int i = n_dummy; i < total; i++) {
        bits_out[out_idx++] = matrix[i];
    }
    *n_out = out_idx;

    free(matrix);
}
