#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>

#include "usb_stream.h"

LOG_MODULE_REGISTER(usb_stream, CONFIG_LOG_DEFAULT_LEVEL);

/* Room for ~65 full 244-byte notifications, i.e. a few hundred ms of the BLE
 * link's peak rate if the host stalls. */
#define STREAM_BUF_SIZE 16384

static const struct device *const stream_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));

RING_BUF_DECLARE(stream_buf, STREAM_BUF_SIZE);

/* stream_buf and the counters are written by the BT RX thread and drained from
 * the UART callback, so both sides take the lock. */
static struct k_spinlock lock;
static uint16_t next_seq;
static uint32_t sent;
static uint32_t dropped;
static uint32_t discarded;

static bool port_open(void) {
    uint32_t dtr = 0;

    return uart_line_ctrl_get(stream_dev, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr;
}

static void uart_cb(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (!uart_irq_tx_ready(dev)) {
            break;
        }

        k_spinlock_key_t key = k_spin_lock(&lock);
        uint8_t *data;
        uint32_t len = ring_buf_get_claim(&stream_buf, &data, STREAM_BUF_SIZE);
        if (len == 0) {
            uart_irq_tx_disable(dev);
            k_spin_unlock(&lock, key);
            break;
        }
        int filled = uart_fifo_fill(dev, data, len);
        ring_buf_get_finish(&stream_buf, MAX(filled, 0));
        k_spin_unlock(&lock, key);

        if (filled <= 0) {
            // the CDC ACM TX buffer is full; we're called again once it drains
            break;
        }
    }
}

int usb_stream_init(void) {
    if (!device_is_ready(stream_dev)) {
        LOG_ERR("stream port %s not ready", stream_dev->name);
        return -ENODEV;
    }

    uart_irq_callback_set(stream_dev, uart_cb);
    return 0;
}

int usb_stream_send(enum usb_frame_type type, const uint8_t *payload, uint16_t len) {
    uint8_t header[USB_FRAME_HEADER_LEN] = { USB_FRAME_SYNC0, USB_FRAME_SYNC1, type };
    bool open = port_open();

    k_spinlock_key_t key = k_spin_lock(&lock);

    // seq advances for every frame so the host sees a gap for each lost one
    sys_put_le16(next_seq++, &header[3]);
    sys_put_le16(len, &header[5]);

    if (!open) {
        discarded++;
        k_spin_unlock(&lock, key);
        return -EPIPE;
    }
    if (ring_buf_space_get(&stream_buf) < sizeof(header) + len) {
        dropped++;
        k_spin_unlock(&lock, key);
        return -ENOBUFS;
    }

    ring_buf_put(&stream_buf, header, sizeof(header));
    ring_buf_put(&stream_buf, payload, len);
    sent++;

    k_spin_unlock(&lock, key);

    uart_irq_tx_enable(stream_dev);
    return 0;
}

void usb_stream_get_stats(struct usb_stream_stats *stats) {
    stats->port_open = port_open();

    k_spinlock_key_t key = k_spin_lock(&lock);
    stats->sent = sent;
    stats->dropped = dropped;
    stats->discarded = discarded;
    stats->buf_used = ring_buf_size_get(&stream_buf);
    stats->buf_size = ring_buf_capacity_get(&stream_buf);
    k_spin_unlock(&lock, key);
}
