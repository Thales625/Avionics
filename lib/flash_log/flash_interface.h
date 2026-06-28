#ifndef __FLASH_INTERFACE_H__
#define __FLASH_INTERFACE_H__

#include <stdint.h>

#define FLASH_USB_MAGIC 0x46555342 // "FUSB"

const uint32_t flash_ack  = 0x4641434B; // "FACK"
const uint32_t flash_nack = 0x4E41434B; // "NACK"

typedef enum {
    CMD_ACK,
    CMD_CLEAR_FLIGHTS,
    CMD_LIST_HEADERS,
    CMD_READ_HEADER,
    CMD_READ_FLIGHT,
} flash_cmd_t;

#endif