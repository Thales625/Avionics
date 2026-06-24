#ifndef __CRC_H__
#define __CRC_H__

#include <inttypes.h> // IWYU pragma: keep
#include <unistd.h>

// CRC-16-CCITT
uint16_t crc16(const uint8_t *data, size_t length);

#endif
