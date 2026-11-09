#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "ble_handler.h"

LOG_MODULE_REGISTER(ble_handler, CONFIG_LOG_DEFAULT_LEVEL);

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define LINK_TEST_STACK     1024
#define LINK_TEST_PRIORITY  8 // below the command thread so STOP is always handled

static const struct ble_handler_cb *app_cb;

// the active connection (ref held), guarded by conn_lock
static struct bt_conn *current_conn;
static struct k_spinlock conn_lock;

static atomic_t link_test_running;
static K_SEM_DEFINE(link_test_sem, 0, 1);

// payload throughput of the last completed link test, 0 if none has run
static atomic_t link_test_last_bps;

/* Credits for data notifications in flight: one per ATT TX buffer, returned by
 * the notify completion callback. Waiting on these instead of retrying
 * bt_gatt_notify() until it stops failing keeps the sender off the host's
 * allocation failure path. Completion callbacks can be skipped when the link
 * drops, so the credits are refilled on every connection. */
#define DATA_TX_CREDITS CONFIG_BT_ATT_TX_COUNT
static K_SEM_DEFINE(data_tx_credits, DATA_TX_CREDITS, DATA_TX_CREDITS);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

// -- NEURAL DATA SERVICE --

/* Notify-only characteristic carrying the sample stream (and link test
 * packets), so NUS is left for commands and their replies. The UUIDs are
 * mirrored in base_node/src/ble_central.c; keep the two in sync. */
#define BT_UUID_DATA_SRV_VAL  BT_UUID_128_ENCODE(0xa3c10000, 0x5f2e, 0x4b8a, 0x9d61, 0x7e4f0c2b9a10)
#define BT_UUID_DATA_CHAR_VAL BT_UUID_128_ENCODE(0xa3c10001, 0x5f2e, 0x4b8a, 0x9d61, 0x7e4f0c2b9a10)

static void data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    LOG_INF("data notifications %s", value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(data_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_DATA_SRV_VAL)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_DATA_CHAR_VAL),
                           BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(data_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CUD("Neural Data", BT_GATT_PERM_READ),
);

// the characteristic declaration; bt_gatt_notify() resolves its value handle
#define DATA_CHAR_ATTR (&data_svc.attrs[1])

static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("BLE connection failed: %u", err);
        return;
    }
    LOG_INF("BLE connected");

    k_spinlock_key_t key = k_spin_lock(&conn_lock);
    current_conn = bt_conn_ref(conn);
    k_spin_unlock(&conn_lock, key);

    k_sem_reset(&data_tx_credits);
    for (int i = 0; i < DATA_TX_CREDITS; i++) {
        k_sem_give(&data_tx_credits);
    }

    if (app_cb && app_cb->connected) {
        app_cb->connected();
    }
}

static void adv_restart_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    int ret = bt_le_adv_start(BT_LE_ADV_CONN_ONE_TIME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Advertising failed to restart: %d", ret);
    } else {
        LOG_INF("BLE advertising restarted");
    }
}

static K_WORK_DEFINE(adv_restart_work, adv_restart_work_handler);

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("BLE disconnected, reason: %u", reason);

    k_spinlock_key_t key = k_spin_lock(&conn_lock);
    if (current_conn == conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    k_spin_unlock(&conn_lock, key);

    ble_handler_link_test_stop();

    if (app_cb && app_cb->disconnected) {
        app_cb->disconnected();
    }

    // Defer to the system workqueue: the connection object isn't fully
    // released until this callback returns, so advertising can't restart here.
    k_work_submit(&adv_restart_work);
}

// negotiated link parameters, logged to check what the streaming rate can rely on
static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                             uint16_t timeout) {
    ARG_UNUSED(conn);
    LOG_INF("conn params: interval %u.%02u ms, latency %u, timeout %u ms",
            interval * 125 / 100, interval * 125 % 100, latency, timeout * 10);
}

static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param) {
    ARG_UNUSED(conn);
    LOG_INF("PHY: tx %s, rx %s", param->tx_phy == BT_GAP_LE_PHY_2M ? "2M" : "1M/coded",
            param->rx_phy == BT_GAP_LE_PHY_2M ? "2M" : "1M/coded");
}

static void le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info) {
    ARG_UNUSED(conn);
    LOG_INF("data length: tx %u B / %u us, rx %u B / %u us", info->tx_max_len,
            info->tx_max_time, info->rx_max_len, info->rx_max_time);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_updated = le_param_updated,
    .le_phy_updated = le_phy_updated,
    .le_data_len_updated = le_data_len_updated,
};

static void nus_notif_enabled(bool enabled, void *ctx) {
    ARG_UNUSED(ctx);
    LOG_INF("NUS notifications %s", enabled ? "enabled" : "disabled");
}

static void nus_received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx) {
    ARG_UNUSED(conn);
    ARG_UNUSED(ctx);
    LOG_HEXDUMP_DBG(data, len, "NUS RX");

    if (app_cb && app_cb->received) {
        app_cb->received(data, len);
    }
}

static struct bt_nus_cb nus_callbacks = {
    .notif_enabled = nus_notif_enabled,
    .received = nus_received,
};

int ble_handler_init(const struct ble_handler_cb *cb) {
    app_cb = cb;

    int ret = bt_nus_cb_register(&nus_callbacks, NULL);
    if (ret) {
        LOG_ERR("Failed to register NUS callbacks: %d", ret);
        return ret;
    }

    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("Bluetooth init failed: %d", ret);
        return ret;
    }

    ret = bt_le_adv_start(BT_LE_ADV_CONN_ONE_TIME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Advertising failed to start: %d", ret);
        return ret;
    }

    LOG_INF("BLE advertising started as \"%s\"", DEVICE_NAME);
    return 0;
}

int ble_handler_send(const void *data, uint16_t len) {
    return bt_nus_send(NULL, data, len);
}

// returns a new ref to the active connection, or NULL; caller must bt_conn_unref()
static struct bt_conn *get_conn(void) {
    struct bt_conn *conn = NULL;

    k_spinlock_key_t key = k_spin_lock(&conn_lock);
    if (current_conn) {
        conn = bt_conn_ref(current_conn);
    }
    k_spin_unlock(&conn_lock, key);

    return conn;
}

static void data_sent(struct bt_conn *conn, void *user_data) {
    ARG_UNUSED(conn);
    ARG_UNUSED(user_data);
    k_sem_give(&data_tx_credits);
}

int ble_handler_send_data(const void *data, uint16_t len, k_timeout_t timeout) {
    if (k_sem_take(&data_tx_credits, timeout) != 0) {
        return -EAGAIN;
    }

    struct bt_conn *conn = get_conn();
    if (conn == NULL) {
        k_sem_give(&data_tx_credits);
        return -ENOTCONN;
    }

    struct bt_gatt_notify_params params = {
        .attr = DATA_CHAR_ATTR,
        .data = data,
        .len = len,
        .func = data_sent,
    };
    int ret = bt_gatt_notify_cb(conn, &params);
    if (ret) {
        k_sem_give(&data_tx_credits);
    }
    bt_conn_unref(conn);
    return ret;
}

uint16_t ble_handler_data_payload_max(void) {
    struct bt_conn *conn = get_conn();
    if (conn == NULL) {
        return 0;
    }
    uint16_t len = bt_gatt_get_mtu(conn) - 3; // ATT notify header
    bt_conn_unref(conn);
    return MIN(len, CONFIG_BT_L2CAP_TX_MTU - 3);
}

uint32_t ble_handler_link_test_last_bps(void) {
    return (uint32_t)atomic_get(&link_test_last_bps);
}

// -- BLE LINK TEST --

/* Sends full-size data notifications back to back. Each packet starts with a
 * little-endian u32 sequence number (so the receiver can count drops) followed
 * by a fixed byte pattern. bt_gatt_notify() blocks while the TX buffers are full,
 * so this runs at whatever rate the link can sustain. */
static void link_test_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    static uint8_t buf[CONFIG_BT_L2CAP_TX_MTU - 3]; // ATT MTU minus the notify header

    for (size_t i = 0; i < sizeof(buf); i++) {
        buf[i] = (uint8_t)i;
    }

    while (1) {
        k_sem_take(&link_test_sem, K_FOREVER);

        struct bt_conn *conn = get_conn();
        if (conn == NULL) {
            LOG_WRN("link test: not connected");
            atomic_clear(&link_test_running);
            continue;
        }

        // the negotiated MTU may be smaller than the configured maximum
        uint16_t len = MIN(bt_gatt_get_mtu(conn) - 3, sizeof(buf));
        uint32_t seq = 0;
        uint64_t bytes = 0;
        int64_t start = k_uptime_get();

        LOG_INF("link test started, %u byte packets", len);

        while (atomic_get(&link_test_running)) {
            sys_put_le32(seq, buf);

            int ret = bt_gatt_notify(conn, DATA_CHAR_ATTR, buf, len);
            if (ret == -EAGAIN || ret == -ENOMEM) {
                k_yield();
                continue;
            }
            if (ret) {
                /* e.g. -ENOTCONN on disconnect (the base restarts the test on
                 * reconnect) or -EINVAL if the peer isn't subscribed */
                LOG_ERR("link test: send failed: %d", ret);
                atomic_clear(&link_test_running);
                break;
            }
            seq++;
            bytes += len;
        }

        /* Don't clear link_test_running here: a stop followed quickly by a new
         * start would otherwise be undone. A pending start re-runs the loop via
         * link_test_sem. */
        bt_conn_unref(conn);

        int64_t elapsed_ms = MAX(k_uptime_get() - start, 1);
        atomic_set(&link_test_last_bps, (atomic_val_t)(bytes * 1000 / elapsed_ms));
        LOG_INF("link test stopped: %u packets, %llu bytes in %lld ms (%llu B/s)",
                seq, bytes, elapsed_ms, bytes * 1000 / elapsed_ms);
    }
}

K_THREAD_DEFINE(link_test_tid, LINK_TEST_STACK, link_test_thread, NULL, NULL, NULL,
                LINK_TEST_PRIORITY, 0, 0);

int ble_handler_link_test_start(void) {
    if (atomic_set(&link_test_running, 1)) {
        return -EALREADY;
    }
    k_sem_give(&link_test_sem);
    return 0;
}

void ble_handler_link_test_stop(void) {
    atomic_clear(&link_test_running);
}
