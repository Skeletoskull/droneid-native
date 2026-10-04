/**
 * turbo_deratematch.h — 3GPP turbo rate-matching de-interleaver
 *
 * Implements the 32-column RM interleaver de-permutation used in the
 * DroneID location pipeline to recover the systematic turbo stream
 * from the Gold-descrambled bit sequence.
 *
 * Design reference: design.md section 6.11
 * Python reference: rm_turbo_rx() in qpsk.py
 */

#ifndef TURBO_DERATEMATCH_H
#define TURBO_DERATEMATCH_H

#include <stdint.h>

/**
 * 3GPP turbo rate-matching 32-column permutation table.
 *
 * Used to de-interleave the received bit sequence back to the row-read
 * order of the interleaving matrix. Declared extern so it can be used
 * directly by callers that need the permutation indices (e.g., for testing).
 */
extern const int RM_PERM_TURBO[32];

/**
 * Apply the 3GPP turbo de-rate-match de-interleaver.
 *
 * Fills a matrix column-by-column using RM_PERM_TURBO, inserting dummy
 * bits (-1) at the beginning of columns whose permuted index falls within
 * the leading dummy region. The output is the matrix flattened row-by-row
 * with the leading dummy bits stripped.
 *
 * @param bits_in   Input soft-decision bit array (values -1 or +1).
 *                  Length: n_in elements.
 * @param n_in      Number of input bits.
 * @param bits_out  Caller-supplied output buffer; must be at least n_in
 *                  bytes (the output length equals n_in minus any dummy
 *                  bits, which equals at most n_in).
 * @param n_out     On return, set to the number of valid output bits.
 *                  Always equal to n_in (dummy bits at the start of the
 *                  flattened matrix are stripped, leaving exactly n_in bits).
 */
void rm_turbo_rx(const int8_t *bits_in, uint32_t n_in,
                 int8_t *bits_out, uint32_t *n_out);

#endif /* TURBO_DERATEMATCH_H */
