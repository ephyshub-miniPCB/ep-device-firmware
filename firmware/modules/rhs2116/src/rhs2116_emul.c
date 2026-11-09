#include <stdint.h>

#include <zephyr/sys/util.h>

#include <rhs2116/rhs2116.h>
#include "rhs2116_cmd.h"
#include "rhs2116_emul.h"

/*
 * THIS FILE PROVIDES EMULATOR FUNCTIONALITY FOR TESTING THE FIRMWARE
 * WITHOUT ANY RHS2116 HARDWARE. IT IS NOT INTENDED TO BE USED IN ANY
 * REAL IMPLEMENTATION OF THE SYSTEM
 * */

// mirror of the RAM registers; ROM registers are answered directly
static uint16_t regs[256];

// per-channel phase accumulators: the top 6 bits index the sine table
static uint16_t phase[RHS2116_NUM_CHANNELS];

// one sine period, offset binary, +/-0x2000 around mid-scale
static const uint16_t sine[64] = {
    0x8000, 0x8323, 0x863e, 0x894a, 0x8c3f, 0x8f16, 0x91c7, 0x944d,
    0x96a1, 0x98bd, 0x9a9b, 0x9c39, 0x9d90, 0x9e9f, 0x9f63, 0x9fd9,
    0xa000, 0x9fd9, 0x9f63, 0x9e9f, 0x9d90, 0x9c39, 0x9a9b, 0x98bd,
    0x96a1, 0x944d, 0x91c7, 0x8f16, 0x8c3f, 0x894a, 0x863e, 0x8323,
    0x8000, 0x7cdd, 0x79c2, 0x76b6, 0x73c1, 0x70ea, 0x6e39, 0x6bb3,
    0x695f, 0x6743, 0x6565, 0x63c7, 0x6270, 0x6161, 0x609d, 0x6027,
    0x6000, 0x6027, 0x609d, 0x6161, 0x6270, 0x63c7, 0x6565, 0x6743,
    0x695f, 0x6bb3, 0x6e39, 0x70ea, 0x73c1, 0x76b6, 0x79c2, 0x7cdd,
};

// offset added to a channel's sample while its stimulator is on, so stim
// pulses show up in the recorded data
#define STIM_OFFSET 0x3000

static uint16_t convert(uint8_t ch) {
    if (ch >= RHS2116_NUM_CHANNELS) {
        return 0;
    }

    // channel n completes a period every 1024 / (n + 1) samples
    phase[ch] += (uint16_t)(64u * (ch + 1));
    uint16_t sample = sine[phase[ch] >> 10];

    if (regs[RHS2116_REG_STIM_ON] & BIT(ch)) {
        sample = (regs[RHS2116_REG_STIM_POL] & BIT(ch)) ? sample + STIM_OFFSET
                                                        : sample - STIM_OFFSET;
    }
    return sample;
}

uint32_t rhs2116_emul_response(uint32_t cmd) {
    uint8_t reg = (uint8_t)(cmd >> 16);

    if ((cmd & RHS2116_CMD_READ) == RHS2116_CMD_READ) {
        switch (reg) {
        case RHS2116_REG_CHIP_ID:
            return RHS2116_CHIP_ID;
        case RHS2116_REG_CHIP_SPEC:
            return (1u << 8) | RHS2116_NUM_CHANNELS; // die revision 1
        case RHS2116_REG_COMPLIANCE:
            return 0; // never out of compliance
        default:
            return regs[reg];
        }
    }

    if (cmd & RHS2116_CMD_WRITE) {
        uint16_t data = (uint16_t)cmd;
        regs[reg] = data;
        return RHS2116_WRITE_ECHO(data);
    }

    if ((cmd >> 30) == 1) { // CLEAR (the only command with top bits 01)
        return 0;
    }

    // CONVERT: AC sample in the high half, DC sample (unused) in the low half
    return (uint32_t)convert(reg & 0x3F) << 16;
}
