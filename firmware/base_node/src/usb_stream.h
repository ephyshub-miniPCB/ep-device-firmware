#ifndef USB_STREAM_H
#define USB_STREAM_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Binary frame stream to the PC over the second USB CDC ACM port
 * (cdc_acm_uart1; the shell and logs stay on cdc_acm_uart0).
 *
 * Frame format, multi-byte fields little-endian:
 *
 *   byte 0..1   sync     0xA5 0x5A
 *   byte 2      type     enum usb_frame_type
 *   byte 3..4   seq      u16, +1 per frame, including frames that were dropped
 *   byte 5..6   len      u16, payload length
 *   byte 7..    payload
 *
 * A gap in seq means frames were dropped (buffer full). There is no CRC: USB
 * already checks and retries every packet, so a parser resyncing on the sync
 * bytes should confirm the next frame's sync follows at 7 + len.
 */

#define USB_FRAME_SYNC0      0xA5
#define USB_FRAME_SYNC1      0x5A
#define USB_FRAME_HEADER_LEN 7

enum usb_frame_type {
    /* one notification from the ep_device's neural data characteristic */
    USB_FRAME_DATA = 0x01,
};

struct usb_stream_stats {
    bool port_open;        /* host has the port open (DTR set) */
    uint32_t sent;         /* frames queued for the host */
    uint32_t dropped;      /* frames lost because the buffer was full */
    uint32_t discarded;    /* frames thrown away while the port was closed */
    uint32_t buf_used;     /* bytes waiting in the buffer */
    uint32_t buf_size;
};

/* Sets up the stream port. Returns -ENODEV if it isn't ready. */
int usb_stream_init(void);

/* Queues one frame without blocking, so it is safe from the BT RX thread.
 * Returns -EPIPE if the host doesn't have the port open, -ENOBUFS if the
 * buffer can't fit the whole frame (it is dropped, never truncated). */
int usb_stream_send(enum usb_frame_type type, const uint8_t *payload, uint16_t len);

void usb_stream_get_stats(struct usb_stream_stats *stats);

#endif /* USB_STREAM_H */
