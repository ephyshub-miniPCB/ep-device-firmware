#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble_central.h"
#include "ep_cmd.h"
#include "ep_info.h"
#include "ep_request.h"
#include "link_test.h"
#include "usb_stream.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Data characteristic notifications go to the link test while it runs (its
 * dummy packets aren't neural data), otherwise out the USB stream port. */
static void on_ep_data(const uint8_t *data, uint16_t len) {
    if (link_test_active()) {
        link_test_on_packet(data, len);
        return;
    }

    // failures are counted by usb_stream and shown by "status stream"
    (void)usb_stream_send(USB_FRAME_DATA, data, len);
}

/* NUS replies are [opcode of the command][payload]; route them by opcode. */
static void on_ep_reply(const uint8_t *data, uint16_t len) {
    if (len >= 1 && data[0] == EP_CMD_GET_INFO) {
        ep_info_on_reply(&data[1], len - 1);
        return;
    }
    if (ep_request_on_reply(data, len)) {
        return;
    }

    LOG_HEXDUMP_INF(data, len, "ep_device reply");
}

static const struct ble_central_cb ble_cb = {
    .data = on_ep_data,
    .reply = on_ep_reply,
    .ready = link_test_on_ready,
    .disconnected = link_test_on_disconnect,
};

int main(void) {
    int ret = usb_stream_init();
    if (ret) {
        return ret;
    }

    return ble_central_init(&ble_cb);
}
