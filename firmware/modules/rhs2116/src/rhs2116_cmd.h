#ifndef RHS2116_CMD_H_
#define RHS2116_CMD_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <rhs2116/rhs2116_regs.h>

/*
 * RHS2116 SPI command word encoders (datasheet "SPI Command Words"). Words are
 * returned in CPU order; the transport ring stores them big-endian, see
 * rhs2116_wire().
 */

#define RHS2116_CMD_WRITE   BIT(31)
#define RHS2116_CMD_READ    (BIT(31) | BIT(30))
#define RHS2116_CMD_CLEAR   (BIT(30) | BIT(29) | BIT(27) | BIT(25))

#define RHS2116_FLAG_U      BIT(29) // update triggered registers
#define RHS2116_FLAG_M      BIT(28) // clear compliance monitor
#define RHS2116_FLAG_D      BIT(27) // CONVERT: also sample the DC amplifier
#define RHS2116_FLAG_H      BIT(26) // CONVERT: reset the DSP high-pass filter

// a WRITE echoes its data in the low half under an all-ones high half
#define RHS2116_WRITE_ECHO(data)    (0xFFFF0000u | (uint16_t)(data))

static inline uint32_t rhs2116_cmd_convert(uint8_t chan, uint32_t flags) {
    return flags | ((uint32_t)(chan & 0x3F) << 16);
}

static inline uint32_t rhs2116_cmd_write(uint8_t reg, uint16_t data, bool update, bool clear) {
    return RHS2116_CMD_WRITE | (update ? RHS2116_FLAG_U : 0) | (clear ? RHS2116_FLAG_M : 0) |
           ((uint32_t)reg << 16) | data;
}

static inline uint32_t rhs2116_cmd_read(uint8_t reg, bool update, bool clear) {
    return RHS2116_CMD_READ | (update ? RHS2116_FLAG_U : 0) | (clear ? RHS2116_FLAG_M : 0) |
           ((uint32_t)reg << 16);
}

static inline uint32_t rhs2116_cmd_clear(void) {
    return RHS2116_CMD_CLEAR;
}

// CPU order <-> the MSB-first byte order the SPIM clocks out
static inline uint32_t rhs2116_wire(uint32_t word) {
    return sys_cpu_to_be32(word);
}

static inline uint32_t rhs2116_unwire(uint32_t word) {
    return sys_be32_to_cpu(word);
}

// AC high-gain amplifier sample in a CONVERT result
static inline uint16_t rhs2116_convert_ac(uint32_t result) {
    return (uint16_t)(result >> 16);
}

#endif /* RHS2116_CMD_H_ */
