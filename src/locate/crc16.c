/**
 * crc16.c — Reflected CRC-16 (poly 0x11021, init 0x3692)
 *
 * Mirrors crcmod.mkCrcFun(0x11021, initCrc=0x3692, rev=True) from
 * droneid_packet.py.  "rev=True" = reflected (LSB-first) bit order.
 * Reflected polynomial of 0x11021 is 0x8408.
 */

#include "locate_internal.h"

uint16_t compute_crc16(const uint8_t *data, uint32_t len) {
    if (!data) return 0x3692u;   /* return init value for NULL input */
    uint16_t crc = 0x3692u;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1u)
                crc = (crc >> 1) ^ 0x8408u;
            else
                crc >>= 1;
        }
    }
    return crc;
}
