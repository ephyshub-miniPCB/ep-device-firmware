#ifndef EP_INFO_H
#define EP_INFO_H

#include <stdint.h>

#include <zephyr/kernel.h>

/* State of the ep_device's RHS2116 as it last reported it. Mirrors enum
 * device_info_rhs2116_state in ep_device/src/device_info.h. */
enum ep_info_rhs2116_state {
    EP_INFO_RHS2116_UNKNOWN = 0, /* ep_device hasn't queried the chip yet */
    EP_INFO_RHS2116_OK      = 1,
    EP_INFO_RHS2116_ERROR   = 2, /* querying the chip failed, see rhs2116_error */
    EP_INFO_RHS2116_EMULATED = 3, /* no chip, ep_device built with CONFIG_RHS2116_EMUL */
};

#define EP_INFO_STR_LEN 48

/* Decoded GET_INFO reply. */
struct ep_info {
    uint8_t format;
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t fw_patch;
    uint8_t hw_revision;

    uint8_t rhs2116_state;    /* enum ep_info_rhs2116_state */
    int16_t rhs2116_error;    /* negative errno, EP_INFO_RHS2116_ERROR only */
    uint8_t rhs2116_chip_id;  /* these three are valid in EP_INFO_RHS2116_OK/EMULATED only */
    uint8_t rhs2116_die_revision;
    uint8_t rhs2116_num_channels;

    /* NUL terminated, truncated to fit */
    char name[EP_INFO_STR_LEN];
    char build[EP_INFO_STR_LEN];  /* git describe of the ep_device build */
    char zephyr[EP_INFO_STR_LEN]; /* Zephyr version the ep_device was built with */
};

/* Sends GET_INFO to the ep_device and blocks until it replies.
 * Returns 0 and fills *info, -ENOTCONN if the ep_device isn't connected,
 * -ETIMEDOUT if no reply arrived in time, or another BLE send error.
 * Must not be called from the BT stack's context. */
int ep_info_query(struct ep_info *info, k_timeout_t timeout);

/* Hook for the BLE central: payload of a GET_INFO reply (opcode stripped).
 * Called from the BT stack's context. */
void ep_info_on_reply(const uint8_t *data, uint16_t len);

#endif /* EP_INFO_H */
