#ifndef EP_REQUEST_H
#define EP_REQUEST_H

#include <stdint.h>

#include <zephyr/kernel.h>

/* Sends [opcode][payload] to the ep_device and blocks until a NUS reply with
 * the same opcode arrives. The reply (opcode stripped) is copied to reply, up
 * to reply_size bytes. Returns the reply length, -ENOTCONN if not connected,
 * -ETIMEDOUT, or another BLE send error. One request at a time; must not be
 * called from the BT stack's context. */
int ep_request(uint8_t opcode, const uint8_t *payload, uint16_t len,
               uint8_t *reply, uint16_t reply_size, k_timeout_t timeout);

/* Hook for the BLE central: a NUS reply that no one else claimed.
 * Returns true if it answered the pending request. */
bool ep_request_on_reply(const uint8_t *data, uint16_t len);

#endif /* EP_REQUEST_H */
