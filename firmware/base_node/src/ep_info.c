#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ble_central.h"
#include "ep_cmd.h"
#include "ep_info.h"

LOG_MODULE_REGISTER(ep_info, CONFIG_LOG_DEFAULT_LEVEL);

/* Payload layout, see ep_device/src/device_info.h: 11 fixed bytes followed by
 * three [u8 len][bytes] strings. */
#define EP_INFO_FORMAT    1
#define EP_INFO_FIXED_LEN 11

static K_SEM_DEFINE(reply_sem, 0, 1);
static struct k_spinlock lock;
static struct ep_info last_info;

// reads one [u8 len][bytes] string at *pos into out (NUL terminated, truncated)
static int get_str(const uint8_t *data, uint16_t len, uint16_t *pos, char *out, size_t out_size) {
    if (*pos >= len) {
        return -EINVAL;
    }
    uint8_t str_len = data[(*pos)++];
    if (*pos + str_len > len) {
        return -EINVAL;
    }

    size_t copy = MIN(str_len, out_size - 1);
    memcpy(out, &data[*pos], copy);
    out[copy] = '\0';
    *pos += str_len;
    return 0;
}

void ep_info_on_reply(const uint8_t *data, uint16_t len) {
    if (len < EP_INFO_FIXED_LEN) {
        LOG_WRN("GET_INFO reply too short (%u bytes)", len);
        return;
    }
    if (data[0] != EP_INFO_FORMAT) {
        LOG_WRN("GET_INFO reply has unsupported format %u (expected %u)",
                data[0], EP_INFO_FORMAT);
        return;
    }

    struct ep_info info = {
        .format = data[0],
        .fw_major = data[1],
        .fw_minor = data[2],
        .fw_patch = data[3],
        .hw_revision = data[4],
        .rhs2116_state = data[5],
        .rhs2116_error = (int16_t)sys_get_le16(&data[6]),
        .rhs2116_chip_id = data[8],
        .rhs2116_die_revision = data[9],
        .rhs2116_num_channels = data[10],
    };

    uint16_t pos = EP_INFO_FIXED_LEN;
    if (get_str(data, len, &pos, info.name, sizeof(info.name)) ||
        get_str(data, len, &pos, info.build, sizeof(info.build)) ||
        get_str(data, len, &pos, info.zephyr, sizeof(info.zephyr))) {
        LOG_WRN("GET_INFO reply has malformed strings");
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&lock);
    last_info = info;
    k_spin_unlock(&lock, key);

    k_sem_give(&reply_sem);
}

int ep_info_query(struct ep_info *info, k_timeout_t timeout) {
    // drop any stale reply to an earlier query that timed out
    k_sem_reset(&reply_sem);

    uint8_t cmd = EP_CMD_GET_INFO;
    int ret = ble_central_send(&cmd, sizeof(cmd));
    if (ret) {
        return ret;
    }

    if (k_sem_take(&reply_sem, timeout)) {
        return -ETIMEDOUT;
    }

    k_spinlock_key_t key = k_spin_lock(&lock);
    *info = last_info;
    k_spin_unlock(&lock, key);
    return 0;
}
