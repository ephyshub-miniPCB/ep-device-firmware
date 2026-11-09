#ifndef DEVICE_INFO_H
#define DEVICE_INFO_H

#include <stddef.h>
#include <stdint.h>

/*
 * GET_INFO reply payload (ep_device -> base node, sent over NUS TX after the
 * echoed CMD_GET_INFO opcode byte). Mirrors base_node/src/ep_info.c; keep the
 * two in sync and bump DEVICE_INFO_FORMAT on incompatible changes.
 *
 *   byte 0      format version (DEVICE_INFO_FORMAT)
 *   byte 1..3   firmware version major, minor, patch (from ep_device/VERSION)
 *   byte 4      hardware revision (CONFIG_EP_DEVICE_HW_REVISION)
 *   byte 5      RHS2116 state (enum device_info_rhs2116_state)
 *   byte 6..7   RHS2116 error, s16 LE (negative errno, valid in STATE_ERROR)
 *   byte 8      RHS2116 chip ID         \
 *   byte 9      RHS2116 die revision     > valid in STATE_OK only
 *   byte 10     RHS2116 channel count   /
 *   then three length-prefixed strings ([u8 len][bytes], no NUL):
 *               device name, build version (git describe, -dirty if
 *               ep_device has uncommitted changes), Zephyr version
 */
#define DEVICE_INFO_FORMAT 1

enum device_info_rhs2116_state {
    DEVICE_INFO_RHS2116_UNKNOWN = 0, /* not queried yet */
    DEVICE_INFO_RHS2116_OK      = 1,
    DEVICE_INFO_RHS2116_ERROR   = 2, /* query failed, see error field */
    DEVICE_INFO_RHS2116_EMULATED = 3, /* no chip, CONFIG_RHS2116_EMUL; info is the model's */
};

/* Records the read-only info read from the RHS2116 once it has been verified. */
void device_info_set_rhs2116(uint8_t chip_id, uint8_t die_revision, uint8_t num_channels);

/* Records that querying the RHS2116 failed with err (negative errno). */
void device_info_set_rhs2116_error(int err);

/* Encodes the GET_INFO reply payload (format above) into buf.
 * Returns the number of bytes written, or -ENOMEM if buf is too small. */
int device_info_encode(uint8_t *buf, size_t size);

#endif /* DEVICE_INFO_H */
