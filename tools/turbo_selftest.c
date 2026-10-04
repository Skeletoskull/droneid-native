/**
 * turbo_selftest.c — Self-test of the built-in LTE turbo FEC engine.
 *
 * Encodes a random block, adds noise and turbo-decodes it; needs no capture.
 *
 * Usage:
 *   turbo_selftest
 */
#include <stdio.h>
#include "droneid_locate.h"

int main(void) {
    int32_t st = locate_turbo_self_test();
    printf("turbo FEC self-test: %s (rc=%d)\n",
           st == 0 ? "PASS" : "FAIL", (int)st);
    return st == 0 ? 0 : 1;
}
