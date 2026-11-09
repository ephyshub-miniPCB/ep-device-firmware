#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <stdint.h>

#include <zephyr/kernel.h>

/* Application hooks invoked from the BLE stack's context on link changes. */
struct ble_handler_cb {
    void (*connected)(void);
    void (*disconnected)(void);
    /* Data written by the peer to NUS RX. Runs on the BLE RX thread, so it
     * must not block. */
    void (*received)(const uint8_t *data, uint16_t len);
};

/* Registers NUS callbacks, enables Bluetooth and starts advertising.
 * cb may be NULL; the pointed-to struct must outlive the BLE stack. */
int ble_handler_init(const struct ble_handler_cb *cb);

/* Sends data over NUS to all connected peers; used for command replies.
 * Returns bt_nus_send()'s result (-ENOTCONN when nothing is connected, -EAGAIN
 * when TX buffers are full). */
int ble_handler_send(const void *data, uint16_t len);

/* Notifies data on the neural data characteristic to the connected peer.
 * Waits up to timeout for one of the CONFIG_BT_ATT_TX_COUNT notification
 * credits, which come back as the controller finishes sending earlier ones.
 * Returns -EAGAIN if none came back in time, -ENOTCONN when not connected,
 * -EINVAL if the peer isn't subscribed, or another bt_gatt_notify_cb() error. */
int ble_handler_send_data(const void *data, uint16_t len, k_timeout_t timeout);

/* Largest notification payload on the data characteristic for the current
 * connection (negotiated ATT MTU minus the notify header), or 0 if not connected. */
uint16_t ble_handler_data_payload_max(void);

/* Payload bytes/s achieved by the last completed link test, 0 if none has run. */
uint32_t ble_handler_link_test_last_bps(void);

/* Starts streaming full-size data characteristic packets of dummy data (u32 LE sequence number
 * then a byte pattern) as fast as the link allows, on its own thread.
 * Returns -EALREADY if a test is already running. */
int ble_handler_link_test_start(void);

/* Stops the link test; also called automatically on disconnect. */
void ble_handler_link_test_stop(void);

#endif /* BLE_HANDLER_H */
