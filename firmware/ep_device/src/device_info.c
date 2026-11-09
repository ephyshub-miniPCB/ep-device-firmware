#include <errno.h>
#include <string.h>

#include <app_version.h>
#include <build_version.h>
#include <zephyr/kernel.h>
#include <zephyr/version.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "device_info.h"

/* Written once by main() after probing the RHS2116, read by the command thread. */
static struct k_spinlock lock;
static uint8_t rhs2116_state = DEVICE_INFO_RHS2116_UNKNOWN;
static int16_t rhs2116_error;
static uint8_t rhs2116_chip_id;
static uint8_t rhs2116_die_revision;
static uint8_t rhs2116_num_channels;

void device_info_set_rhs2116(uint8_t chip_id, uint8_t die_revision, uint8_t num_channels) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    rhs2116_state = IS_ENABLED(CONFIG_RHS2116_EMUL) ? DEVICE_INFO_RHS2116_EMULATED
                                                    : DEVICE_INFO_RHS2116_OK;
    rhs2116_error = 0;
    rhs2116_chip_id = chip_id;
    rhs2116_die_revision = die_revision;
    rhs2116_num_channels = num_channels;
    k_spin_unlock(&lock, key);
}

void device_info_set_rhs2116_error(int err) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    rhs2116_state = DEVICE_INFO_RHS2116_ERROR;
    rhs2116_error = (int16_t)err;
    k_spin_unlock(&lock, key);
}

// appends [u8 len][bytes], truncating str to 255 bytes
static int put_str(uint8_t *buf, size_t size, size_t *pos, const char *str) {
    size_t len = MIN(strlen(str), UINT8_MAX);

    if (*pos + 1 + len > size) {
        return -ENOMEM;
    }
    buf[(*pos)++] = (uint8_t)len;
    memcpy(&buf[*pos], str, len);
    *pos += len;
    return 0;
}

int device_info_encode(uint8_t *buf, size_t size) {
    if (size < 11) {
        return -ENOMEM;
    }

    buf[0] = DEVICE_INFO_FORMAT;
    buf[1] = APP_VERSION_MAJOR;
    buf[2] = APP_VERSION_MINOR;
    buf[3] = APP_PATCHLEVEL;
    buf[4] = CONFIG_EP_DEVICE_HW_REVISION;

    k_spinlock_key_t key = k_spin_lock(&lock);
    buf[5] = rhs2116_state;
    sys_put_le16((uint16_t)rhs2116_error, &buf[6]);
    buf[8] = rhs2116_chip_id;
    buf[9] = rhs2116_die_revision;
    buf[10] = rhs2116_num_channels;
    k_spin_unlock(&lock, key);

    size_t pos = 11;
    int ret = put_str(buf, size, &pos, CONFIG_BT_DEVICE_NAME);
    if (!ret) {
        ret = put_str(buf, size, &pos, EP_BUILD_VERSION);
    }
    if (!ret) {
        ret = put_str(buf, size, &pos, KERNEL_VERSION_STRING);
    }
    return ret ? ret : (int)pos;
}
