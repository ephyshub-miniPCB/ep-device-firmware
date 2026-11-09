#ifndef BLE_CENTRAL_H
#define BLE_CENTRAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/kernel.h>

#define BLE_CENTRAL_NAME_MAX 29 // longest name that fits in an advertising packet

/* Application hooks, all invoked from the BT stack's context, so they must not
 * block. Any of them may be NULL. */
struct ble_central_cb {
    /* Every notification on the ep_device's neural data characteristic (the
     * sample stream and link test packets). data is only valid for the
     * duration of the call. */
    void (*data)(const uint8_t *data, uint16_t len);
    /* Every NUS TX notification, i.e. replies to commands. Same lifetime. */
    void (*reply)(const uint8_t *data, uint16_t len);
    /* Discovery is done: NUS (and the data characteristic, if the ep_device
     * has it) is subscribed and ble_central_send() can be used. */
    void (*ready)(void);
    void (*disconnected)(void);
};

enum ble_central_state {
    BLE_CENTRAL_IDLE,         // no ep_device chosen
    BLE_CENTRAL_CONNECTING,   // first connection to a chosen ep_device
    BLE_CENTRAL_CONNECTED,    // connected and discovered, ble_central_send() works
    BLE_CENTRAL_RECONNECTING, // link lost; retrying until ble_central_disconnect()
};

struct ble_central_device {
    bt_addr_le_t addr;
    char name[BLE_CENTRAL_NAME_MAX + 1]; // empty if none was advertised
    int8_t rssi;
};

struct ble_central_status {
    enum ble_central_state state;
    bool have_target;
    struct ble_central_device target; // the chosen ep_device, if have_target
    int last_err;                     // HCI reason or -errno of the last failure
};

/* Enables Bluetooth. Nothing is connected until ble_central_connect(). */
int ble_central_init(const struct ble_central_cb *cb);

/* Scans for ep_devices (anything advertising the NUS service) for 'duration',
 * blocking the caller. Fills out[] strongest signal first and returns how many
 * were stored (at most max), or -errno if scanning couldn't start. */
int ble_central_scan(k_timeout_t duration, struct ble_central_device *out, size_t max);

/* Starts connecting to dev, then MTU exchange, discovery and subscription; the
 * state becomes CONNECTED when that is done (poll ble_central_get_status()). If
 * the link is lost later it is retried until ble_central_disconnect(). Returns
 * -EALREADY unless idle. */
int ble_central_connect(const struct ble_central_device *dev);

/* Drops the connection (or the attempt to make one) and stops any retries.
 * Returns -ENOTCONN if already idle. The disconnected hook runs when the link
 * actually goes down. */
int ble_central_disconnect(void);

void ble_central_get_status(struct ble_central_status *st);

/* Writes data to the ep_device's NUS RX characteristic (write without response).
 * Returns -ENOTCONN until connected and NUS RX has been discovered. */
int ble_central_send(const void *data, uint16_t len);

#endif /* BLE_CENTRAL_H */
