#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "ble_central.h"
#include "ep_request.h"

#define REQ_MAX_LEN 244

static K_MUTEX_DEFINE(req_mutex);
static K_SEM_DEFINE(reply_sem, 0, 1);
static struct k_spinlock lock;

// the pending request, guarded by lock
static int16_t pending_opcode = -1;
static uint8_t *reply_buf;
static uint16_t reply_cap;
static uint16_t reply_len;

bool ep_request_on_reply(const uint8_t *data, uint16_t len) {
    if (len < 1) {
        return false;
    }

    k_spinlock_key_t key = k_spin_lock(&lock);
    if (pending_opcode != data[0]) {
        k_spin_unlock(&lock, key);
        return false;
    }
    reply_len = MIN(len - 1, reply_cap);
    memcpy(reply_buf, &data[1], reply_len);
    pending_opcode = -1;
    k_spin_unlock(&lock, key);

    k_sem_give(&reply_sem);
    return true;
}

int ep_request(uint8_t opcode, const uint8_t *payload, uint16_t len,
               uint8_t *reply, uint16_t reply_size, k_timeout_t timeout) {
    uint8_t cmd[REQ_MAX_LEN];

    if (len + 1 > sizeof(cmd)) {
        return -EINVAL;
    }
    cmd[0] = opcode;
    if (len) {
        memcpy(&cmd[1], payload, len);
    }

    k_mutex_lock(&req_mutex, K_FOREVER);

    k_spinlock_key_t key = k_spin_lock(&lock);
    pending_opcode = opcode;
    reply_buf = reply;
    reply_cap = reply_size;
    reply_len = 0;
    k_sem_reset(&reply_sem);
    k_spin_unlock(&lock, key);

    int ret = ble_central_send(cmd, len + 1);
    if (ret == 0 && k_sem_take(&reply_sem, timeout) != 0) {
        ret = -ETIMEDOUT;
    }

    key = k_spin_lock(&lock);
    pending_opcode = -1;
    if (ret == 0) {
        ret = reply_len;
    }
    k_spin_unlock(&lock, key);

    k_mutex_unlock(&req_mutex);
    return ret;
}
