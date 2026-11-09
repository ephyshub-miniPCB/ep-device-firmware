#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/nus.h>

#include "ble_central.h"

LOG_MODULE_REGISTER(ble_central, CONFIG_LOG_DEFAULT_LEVEL);

// delay between attempts to get a lost ep_device back
#define RECONNECT_DELAY K_SECONDS(1)
#define SCAN_TABLE_SIZE 16

static struct bt_uuid_128 nus_srv_uuid = BT_UUID_INIT_128(BT_UUID_NUS_SRV_VAL);
static struct bt_uuid_128 nus_tx_uuid = BT_UUID_INIT_128(BT_UUID_NUS_TX_CHAR_VAL);
static struct bt_uuid_128 nus_rx_uuid = BT_UUID_INIT_128(BT_UUID_NUS_RX_CHAR_VAL);
/* ep_device's neural data service. Mirrors ep_device/src/ble_handler.c; keep the
 * two in sync. */
static struct bt_uuid_128 data_srv_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0xa3c10000, 0x5f2e, 0x4b8a, 0x9d61, 0x7e4f0c2b9a10));
static struct bt_uuid_128 data_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0xa3c10001, 0x5f2e, 0x4b8a, 0x9d61, 0x7e4f0c2b9a10));
/* Must be static: discover_params keeps a pointer to it across ATT requests, so
 * BT_UUID_GATT_CCC (a compound literal scoped to the callback) would dangle. */
static struct bt_uuid_16 ccc_uuid = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

static const struct ble_central_cb *app_cb;

static struct bt_conn *default_conn;
static struct bt_gatt_exchange_params mtu_params;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params nus_sub_params;
static struct bt_gatt_subscribe_params data_sub_params;
// the subscription whose CCC descriptor is being discovered
static struct bt_gatt_subscribe_params *pending_sub;

// NUS service handle range, kept so the RX characteristic can be found after TX
static uint16_t nus_start_handle;
static uint16_t nus_end_handle;
// value handle of the ep_device's NUS RX characteristic (0 until discovered)
static uint16_t nus_rx_handle;

/* Connection state, written from the BT RX thread, the system workqueue (retries)
 * and the shell thread, so it is only touched under state_lock. target is the
 * device the user chose; it stays set while reconnecting after a lost link. */
static struct k_spinlock state_lock;
static enum ble_central_state state;
static bool have_target;
static struct ble_central_device target;
static int last_err;

static void reconnect_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(reconnect_work, reconnect_work_handler);

static void set_state(enum ble_central_state s) {
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    state = s;
    k_spin_unlock(&state_lock, key);
}

static uint8_t nus_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                          const void *data, uint16_t length) {
    ARG_UNUSED(conn);

    if (!data) {
        LOG_INF("NUS unsubscribed");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    if (app_cb && app_cb->reply) {
        app_cb->reply(data, length);
    }

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t data_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length) {
    ARG_UNUSED(conn);

    if (!data) {
        LOG_INF("Data characteristic unsubscribed");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    if (app_cb && app_cb->data) {
        app_cb->data(data, length);
    }

    return BT_GATT_ITER_CONTINUE;
}

static bool discovering_data_service(const struct bt_gatt_discover_params *params) {
    return params->uuid == &data_srv_uuid.uuid || params->uuid == &data_char_uuid.uuid ||
           (params->type == BT_GATT_DISCOVER_DESCRIPTOR && pending_sub == &data_sub_params);
}

static void discovery_done(struct bt_gatt_discover_params *params) {
    memset(params, 0, sizeof(*params));
    pending_sub = NULL;
    set_state(BLE_CENTRAL_CONNECTED);

    if (app_cb && app_cb->ready) {
        app_cb->ready();
    }
}

static void subscribe(struct bt_conn *conn, struct bt_gatt_subscribe_params *sub,
                      uint16_t ccc_handle, bt_gatt_notify_func_t notify, const char *name) {
    sub->notify = notify;
    sub->value = BT_GATT_CCC_NOTIFY;
    sub->ccc_handle = ccc_handle;

    int err = bt_gatt_subscribe(conn, sub);
    if (err && err != -EALREADY) {
        LOG_ERR("%s subscribe failed: %d", name, err);
    } else {
        LOG_INF("Subscribed to %s notifications", name);
    }
}

/* Walks NUS service -> TX characteristic -> its CCC (subscribe) -> RX
 * characteristic, then data service -> data characteristic -> its CCC
 * (subscribe). An ep_device without the data service still gets commands. */
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params) {
    if (!attr) {
        if (discovering_data_service(params)) {
            LOG_WRN("ep_device has no neural data characteristic, commands only");
            discovery_done(params);
        } else {
            LOG_ERR("NUS discovery incomplete (type %u)", params->type);
            memset(params, 0, sizeof(*params));
        }
        return BT_GATT_ITER_STOP;
    }

    switch (params->type) {
    case BT_GATT_DISCOVER_PRIMARY: {
        const struct bt_gatt_service_val *svc = attr->user_data;

        if (params->uuid == &nus_srv_uuid.uuid) {
            LOG_DBG("NUS service found, handles 0x%04x-0x%04x", attr->handle, svc->end_handle);
            nus_start_handle = attr->handle;
            nus_end_handle = svc->end_handle;
            params->uuid = &nus_tx_uuid.uuid;
        } else {
            LOG_DBG("Data service found, handles 0x%04x-0x%04x", attr->handle, svc->end_handle);
            params->uuid = &data_char_uuid.uuid;
        }
        params->start_handle = attr->handle + 1;
        params->end_handle = svc->end_handle;
        params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
        break;
    }
    case BT_GATT_DISCOVER_CHARACTERISTIC:
        if (params->uuid == &nus_rx_uuid.uuid) {
            nus_rx_handle = bt_gatt_attr_value_handle(attr);
            LOG_INF("NUS RX characteristic found, ready to send commands");

            params->uuid = &data_srv_uuid.uuid;
            params->start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
            params->end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
            params->type = BT_GATT_DISCOVER_PRIMARY;
            break;
        }

        // NUS TX or the data characteristic: find its CCC descriptor next
        LOG_DBG("%s characteristic found, handle 0x%04x",
                params->uuid == &nus_tx_uuid.uuid ? "NUS TX" : "Data", attr->handle);
        pending_sub = params->uuid == &nus_tx_uuid.uuid ? &nus_sub_params : &data_sub_params;
        pending_sub->value_handle = bt_gatt_attr_value_handle(attr);
        params->uuid = &ccc_uuid.uuid;
        params->start_handle = attr->handle + 2;
        params->type = BT_GATT_DISCOVER_DESCRIPTOR;
        break;

    case BT_GATT_DISCOVER_DESCRIPTOR:
        if (pending_sub == &data_sub_params) {
            subscribe(conn, &data_sub_params, attr->handle, data_notify, "data");
            discovery_done(params);
            return BT_GATT_ITER_STOP;
        }

        subscribe(conn, &nus_sub_params, attr->handle, nus_notify, "NUS TX");
        params->uuid = &nus_rx_uuid.uuid;
        params->start_handle = nus_start_handle + 1;
        params->end_handle = nus_end_handle;
        params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
        break;

    default:
        return BT_GATT_ITER_STOP;
    }

    int err = bt_gatt_discover(conn, params);
    if (err) {
        LOG_ERR("GATT discovery step failed: %d", err);
    }
    return BT_GATT_ITER_STOP;
}

static void start_nus_discovery(struct bt_conn *conn) {
    discover_params.uuid = &nus_srv_uuid.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        LOG_ERR("NUS discovery failed to start: %d", err);
    }
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params) {
    ARG_UNUSED(params);

    if (err) {
        LOG_WRN("MTU exchange failed: %u", err);
    } else {
        LOG_INF("MTU exchanged, ATT MTU %u", bt_gatt_get_mtu(conn));
    }

    /* Discover regardless: a failed exchange only limits the payload size. */
    start_nus_discovery(conn);
}

// -- SCANNING --

/* Results of the scan in progress. device_found runs in the BT RX thread and the
 * shell thread copies the table out once scanning has stopped. */
struct scan_entry {
    struct ble_central_device dev;
    bool is_ep; // advertises NUS, so it is an ep_device
};
static struct scan_entry scan_table[SCAN_TABLE_SIZE];
static size_t scan_count;
static struct k_spinlock scan_lock;

struct ad_info {
    char name[BLE_CENTRAL_NAME_MAX + 1];
    bool has_name;
    bool has_nus;
};

static bool parse_ad(struct bt_data *data, void *user_data) {
    struct ad_info *info = user_data;

    switch (data->type) {
    case BT_DATA_NAME_COMPLETE:
    case BT_DATA_NAME_SHORTENED: {
        size_t len = MIN(data->data_len, BLE_CENTRAL_NAME_MAX);
        memcpy(info->name, data->data, len);
        info->name[len] = '\0';
        info->has_name = true;
        break;
    }
    case BT_DATA_UUID128_ALL:
    case BT_DATA_UUID128_SOME:
        for (size_t i = 0; i + BT_UUID_SIZE_128 <= data->data_len; i += BT_UUID_SIZE_128) {
            if (memcmp(&data->data[i], nus_srv_uuid.val, BT_UUID_SIZE_128) == 0) {
                info->has_nus = true;
            }
        }
        break;

    default:
        break;
    }

    return true;
}

/* ep_device puts its name in the ADV_IND and the NUS UUID in the scan response,
 * so reports are merged by address. */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad) {
    if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
        type != BT_GAP_ADV_TYPE_SCAN_RSP) {
        return;
    }

    struct ad_info info = { 0 };
    bt_data_parse(ad, parse_ad, &info);

    k_spinlock_key_t key = k_spin_lock(&scan_lock);
    struct scan_entry *e = NULL;
    for (size_t i = 0; i < scan_count; i++) {
        if (bt_addr_le_eq(&scan_table[i].dev.addr, addr)) {
            e = &scan_table[i];
            break;
        }
    }
    if (!e && scan_count < SCAN_TABLE_SIZE) {
        e = &scan_table[scan_count++];
        *e = (struct scan_entry){ 0 };
        bt_addr_le_copy(&e->dev.addr, addr);
    }
    if (e) {
        e->dev.rssi = rssi;
        e->is_ep |= info.has_nus;
        if (info.has_name) {
            strcpy(e->dev.name, info.name);
        }
    }
    k_spin_unlock(&scan_lock, key);
}

int ble_central_scan(k_timeout_t duration, struct ble_central_device *out, size_t max) {
    k_spinlock_key_t key = k_spin_lock(&scan_lock);
    scan_count = 0;
    k_spin_unlock(&scan_lock, key);

    /* Active scanning so the NUS UUID in ep_device's scan response is seen too. */
    int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
    if (err) {
        LOG_ERR("Scanning failed to start: %d", err);
        return err;
    }
    k_sleep(duration);
    err = bt_le_scan_stop();
    if (err) {
        LOG_WRN("Failed to stop scanning: %d", err);
    }

    // copy out the ep_devices, strongest signal first
    size_t n = 0;
    key = k_spin_lock(&scan_lock);
    for (size_t i = 0; i < scan_count; i++) {
        if (!scan_table[i].is_ep) {
            continue;
        }
        size_t j = MIN(n, max);
        while (j > 0 && out[j - 1].rssi < scan_table[i].dev.rssi) {
            if (j < max) {
                out[j] = out[j - 1];
            }
            j--;
        }
        if (j < max) {
            out[j] = scan_table[i].dev;
        }
        n = MIN(n + 1, max);
    }
    k_spin_unlock(&scan_lock, key);
    return (int)n;
}

// -- CONNECTION --

/* 30-50 ms interval, no latency, 4 s supervision timeout. ep_device asks for the
 * same after connecting (BT_PERIPHERAL_PREF_* in its prj.conf). */
#define CONN_PARAM BT_LE_CONN_PARAM(BT_GAP_INIT_CONN_INT_MIN, BT_GAP_INIT_CONN_INT_MAX, 0, 400)

static int create_conn(const bt_addr_le_t *addr) {
    return bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, CONN_PARAM, &default_conn);
}

static void reconnect_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    k_spinlock_key_t key = k_spin_lock(&state_lock);
    bool retry = state == BLE_CENTRAL_RECONNECTING && have_target && !default_conn;
    bt_addr_le_t addr = target.addr;
    k_spin_unlock(&state_lock, key);

    if (!retry) {
        return;
    }
    int err = create_conn(&addr);
    if (err) {
        // e.g. -EAGAIN while a scan is running; try again later
        LOG_DBG("Reconnect attempt failed to start: %d", err);
        k_work_reschedule(&reconnect_work, RECONNECT_DELAY);
    }
}

/* The link is gone or never came up: retry if the user still wants this device,
 * else go idle. A first connect that fails goes idle too, so the shell can report it. */
static void link_down(int err) {
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    bool retry = have_target && state != BLE_CENTRAL_CONNECTING;
    if (!retry) {
        have_target = false;
    }
    state = retry ? BLE_CENTRAL_RECONNECTING : BLE_CENTRAL_IDLE;
    last_err = err;
    k_spin_unlock(&state_lock, key);

    if (retry) {
        k_work_reschedule(&reconnect_work, RECONNECT_DELAY);
    }
}

static void connected(struct bt_conn *conn, uint8_t err) {
    if (conn != default_conn) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    if (err) {
        LOG_WRN("Failed to connect to %s: 0x%02x", addr_str, err);
        bt_conn_unref(default_conn);
        default_conn = NULL;
        link_down(err);
        return;
    }

    LOG_INF("Connected to %s", addr_str);

    // initiate an MTU exchange, so packets to fit in a single notification.
    mtu_params.func = mtu_exchange_cb;
    int ret = bt_gatt_exchange_mtu(conn, &mtu_params);
    if (ret) {
        LOG_WRN("MTU exchange failed to start: %d", ret);
        start_nus_discovery(conn);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    if (conn != default_conn) {
        return;
    }

    LOG_INF("Disconnected, reason: 0x%02x", reason);

    bt_conn_unref(default_conn);
    default_conn = NULL;
    memset(&nus_sub_params, 0, sizeof(nus_sub_params));
    memset(&data_sub_params, 0, sizeof(data_sub_params));
    pending_sub = NULL;
    nus_rx_handle = 0;

    if (app_cb && app_cb->disconnected) {
        app_cb->disconnected();
    }

    link_down(reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

int ble_central_connect(const struct ble_central_device *dev) {
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    if (state != BLE_CENTRAL_IDLE || default_conn) {
        k_spin_unlock(&state_lock, key);
        return -EALREADY;
    }
    target = *dev;
    have_target = true;
    state = BLE_CENTRAL_CONNECTING;
    last_err = 0;
    k_spin_unlock(&state_lock, key);

    int err = create_conn(&dev->addr);
    if (err) {
        LOG_ERR("Connection create failed: %d", err);
        key = k_spin_lock(&state_lock);
        have_target = false;
        state = BLE_CENTRAL_IDLE;
        last_err = err;
        k_spin_unlock(&state_lock, key);
    }
    return err;
}

int ble_central_disconnect(void) {
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    bool was_idle = state == BLE_CENTRAL_IDLE;
    have_target = false; // no retries from here on
    if (!default_conn) {
        state = BLE_CENTRAL_IDLE;
    }
    k_spin_unlock(&state_lock, key);

    k_work_cancel_delayable(&reconnect_work);
    if (was_idle) {
        return -ENOTCONN;
    }

    struct bt_conn *conn = default_conn ? bt_conn_ref(default_conn) : NULL;
    if (!conn) {
        return 0;
    }
    // also cancels a connection that is still being set up
    int err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    bt_conn_unref(conn);
    return err;
}

void ble_central_get_status(struct ble_central_status *st) {
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    st->state = state;
    st->target = target;
    st->have_target = have_target;
    st->last_err = last_err;
    k_spin_unlock(&state_lock, key);
}

int ble_central_init(const struct ble_central_cb *cb) {
    app_cb = cb;

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed: %d", err);
        return err;
    }

    LOG_INF("Bluetooth initialised, use \"scan\" and \"connect\" to pick an ep_device");
    return 0;
}

int ble_central_send(const void *data, uint16_t len) {
    if (!default_conn || !nus_rx_handle) {
        return -ENOTCONN;
    }

    return bt_gatt_write_without_response(default_conn, nus_rx_handle, data, len, false);
}
