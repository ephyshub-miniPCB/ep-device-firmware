#ifndef RHS2116_EMUL_H_
#define RHS2116_EMUL_H_

#include <stdint.h>

/*
 * Software model of an RHS2116, for bench testing a board without the chip
 * fitted (CONFIG_RHS2116_EMUL). The SPI transactions still run on the pins so
 * the timing can be checked on a logic analyser; only the MISO words are
 * replaced with what a chip would have answered.
 */

// returns the chip's response (CPU order) to the command word 'cmd' (CPU order).
// call once per command, in the order the commands were clocked out: writes
// update the register shadow and CONVERTs advance the channel's test signal.
uint32_t rhs2116_emul_response(uint32_t cmd);

#endif /* RHS2116_EMUL_H_ */
