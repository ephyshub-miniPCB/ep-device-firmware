#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ble_handler.h"
#include "recording.h"
#include <rhs2116/rhs2116_stream.h>

LOG_MODULE_REGISTER(recording, CONFIG_LOG_DEFAULT_LEVEL);

#define PKT_MAX_LEN      (CONFIG_BT_L2CAP_TX_MTU - 3)
#define PKT_POOL_SIZE    24
#define SEND_THREAD_STACK 1024
#define SEND_THREAD_PRIO  5

struct pkt_ref {
    uint8_t *buf;
    uint16_t len;
};

K_MEM_SLAB_DEFINE_STATIC(pkt_slab, PKT_MAX_LEN, PKT_POOL_SIZE, 4);
K_MSGQ_DEFINE(send_q, sizeof(struct pkt_ref), PKT_POOL_SIZE, 4);

static atomic_t active;
static atomic_t stim_active;
static atomic_t dropped_pending; // a drop happened since the last packet went out
static atomic_t seq;
static atomic_t packets_sent;
static atomic_t packets_dropped;
static atomic_ptr_t fault_hook;

// sample packer state, only touched on the stream thread (and by stop once the
// stream thread is done with it)
static struct packer {
    uint8_t bits;
    uint8_t n_ch;
    uint16_t chan_mask;
    uint8_t frames_per_packet;
    uint8_t *buf;
    uint16_t pos;
    uint8_t n_frames;
    uint32_t acc;
    uint8_t acc_bits;
} pk;

static uint8_t next_flags(void) {
    uint8_t flags = atomic_get(&stim_active) ? REC_FLAG_STIM_ACTIVE : 0;
    if (atomic_clear(&dropped_pending)) {
        flags |= REC_FLAG_DROPPED;
    }
    return flags;
}

static void count_drop(void) {
    atomic_inc(&seq);
    atomic_inc(&packets_dropped);
    atomic_set(&dropped_pending, 1);
}

// stamps the common header and queues the packet for the sender thread
static void submit(uint8_t *buf, uint16_t len) {
    buf[1] = next_flags();
    sys_put_le16((uint16_t)atomic_inc(&seq), &buf[2]);

    struct pkt_ref ref = { .buf = buf, .len = len };
    if (k_msgq_put(&send_q, &ref, K_NO_WAIT) != 0) {
        k_mem_slab_free(&pkt_slab, buf);
        atomic_inc(&packets_dropped);
        atomic_set(&dropped_pending, 1);
    }
}

// -- SAMPLE PACKING --

static void pack_finish(void) {
    if (pk.buf == NULL) {
        return;
    }
    if (pk.acc_bits) {
        pk.buf[pk.pos++] = (uint8_t)pk.acc;
    }
    pk.buf[8] = pk.n_frames;
    submit(pk.buf, pk.pos);
    pk.buf = NULL;
}

static void on_frame(uint32_t frame_idx, const uint16_t *samples, uint8_t n_ch) {
    if (pk.buf == NULL) {
        void *mem;
        if (k_mem_slab_alloc(&pkt_slab, &mem, K_NO_WAIT) != 0) {
            // no buffer: drop this packet's worth of frames, leaving a seq gap
            if (frame_idx % pk.frames_per_packet == 0) {
                count_drop();
            }
            return;
        }
        pk.buf = mem;
        pk.buf[0] = REC_PKT_SAMPLES;
        sys_put_le32(frame_idx, &pk.buf[4]);
        pk.buf[9] = pk.bits;
        sys_put_le16(pk.chan_mask, &pk.buf[10]);
        pk.pos = REC_SAMPLES_HDR_LEN;
        pk.n_frames = 0;
        pk.acc = 0;
        pk.acc_bits = 0;
    }

    const uint8_t shift = 16 - pk.bits;
    for (uint8_t k = 0; k < n_ch; k++) {
        pk.acc |= (uint32_t)(samples[k] >> shift) << pk.acc_bits;
        pk.acc_bits += pk.bits;
        while (pk.acc_bits >= 8) {
            pk.buf[pk.pos++] = (uint8_t)pk.acc;
            pk.acc >>= 8;
            pk.acc_bits -= 8;
        }
    }

    if (++pk.n_frames == pk.frames_per_packet) {
        pack_finish();
    }
}

static void on_status(enum rhs2116_stream_status status, uint32_t frame, uint32_t value) {
    switch (status) {
    case RHS2116_STREAM_STATUS_SPI_ERROR:
        recording_event(REC_EVT_SPI_ERROR, frame, (uint16_t)value, (uint16_t)(value >> 16));
        break;
    case RHS2116_STREAM_STATUS_COMPLIANCE:
        recording_event(REC_EVT_COMPLIANCE, frame, (uint16_t)value, 0);
        break;
    case RHS2116_STREAM_STATUS_STOPPED:
        atomic_clear(&active);
        pack_finish();
        recording_event(REC_EVT_STREAM_STOPPED, frame, (uint16_t)value, 0);
        break;
    }

    rec_fault_hook_t hook = (rec_fault_hook_t)atomic_ptr_get(&fault_hook);
    if (hook) {
        hook(status, frame, value);
    }
}

static const struct rhs2116_stream_sink sink = {
    .frame = on_frame,
    .status = on_status,
};

// -- SENDER --

static void send_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct pkt_ref ref;

    while (1) {
        if (k_msgq_get(&send_q, &ref, K_FOREVER) != 0) {
            continue;
        }

        /* Wait for a notification credit. They keep coming back while the link
         * is up, and a lost link ends the wait with -ENOTCONN, so this doesn't
         * spin or log. -ENOMEM shouldn't happen with credits; it's retried too. */
        int ret;
        do {
            ret = ble_handler_send_data(ref.buf, ref.len, K_MSEC(100));
        } while (ret == -EAGAIN || ret == -ENOMEM);

        if (ret == 0) {
            atomic_inc(&packets_sent);
        } else {
            atomic_inc(&packets_dropped);
        }
        k_mem_slab_free(&pkt_slab, ref.buf);
    }
}

K_THREAD_DEFINE(rec_send_tid, SEND_THREAD_STACK, send_thread, NULL, NULL, NULL,
                SEND_THREAD_PRIO, 0, 0);

// -- CONTROL --

static bool bits_valid(uint8_t bits) {
    return bits == 8 || bits == 10 || bits == 12 || bits == 16;
}

int recording_start(const struct device *dev, const struct rec_params *p, struct rec_info *info) {
    uint8_t n_ch = (uint8_t)POPCOUNT(p->chan_mask);

    *info = (struct rec_info){
        .link_bps = ble_handler_link_test_last_bps(),
        .max_rate_hz = rhs2116_stream_max_rate_hz(MAX(n_ch, 1)),
    };

    if (atomic_get(&active)) {
        info->reject = REC_BUSY;
        return -EBUSY;
    }
    if (!bits_valid(p->bits)) {
        info->reject = REC_BAD_BITS;
        return -EINVAL;
    }

    uint16_t payload_max = ble_handler_data_payload_max();
    if (payload_max == 0) {
        payload_max = PKT_MAX_LEN;
    }
    payload_max = MIN(payload_max, PKT_MAX_LEN);

    uint32_t fpp = 0;
    if (n_ch) {
        fpp = (uint32_t)(payload_max - REC_SAMPLES_HDR_LEN) * 8 / (n_ch * p->bits);
        fpp = CLAMP(fpp, 1, UINT8_MAX);
    }

    pk = (struct packer){
        .bits = p->bits,
        .n_ch = n_ch,
        .chan_mask = p->chan_mask,
        .frames_per_packet = (uint8_t)fpp,
    };
    atomic_clear(&seq);
    atomic_clear(&packets_sent);
    atomic_clear(&packets_dropped);
    atomic_clear(&dropped_pending);

    struct rhs2116_stream_layout layout;
    enum rhs2116_stream_reject why;

    // set before the stream thread can deliver its first frame
    atomic_set(&active, 1);
    int ret = rhs2116_stream_start(dev, p->chan_mask, p->rate_hz, &sink, &layout, &why);
    if (ret < 0) {
        atomic_clear(&active);
        info->reject = (enum rec_reject)why;
        return ret;
    }

    uint32_t pkt_len = REC_SAMPLES_HDR_LEN + DIV_ROUND_UP(fpp * n_ch * p->bits, 8);
    info->reject = REC_OK;
    info->rate_mhz = layout.rate_mhz;
    info->slot_ticks = layout.slot_ticks;
    info->slots_per_frame = layout.slots_per_frame;
    info->frames_per_packet = (uint8_t)fpp;
    info->required_bps = (uint32_t)DIV_ROUND_UP((uint64_t)layout.rate_mhz * pkt_len,
                                                1000ull * fpp);

    if (info->link_bps && info->required_bps > info->link_bps) {
        LOG_WRN("recording needs %u B/s but the last link test managed %u B/s",
                info->required_bps, info->link_bps);
    }
    return 0;
}

void recording_stop(void) {
    if (!atomic_get(&active)) {
        return;
    }
    rhs2116_stream_stop();
    // the stream thread is finished with the packer once stop returns
    atomic_clear(&active);
    pack_finish();
}

bool recording_active(void) {
    return atomic_get(&active);
}

void recording_event(enum rec_event_code code, uint32_t frame, uint16_t a, uint16_t b) {
    void *mem;
    if (k_mem_slab_alloc(&pkt_slab, &mem, K_NO_WAIT) != 0) {
        count_drop();
        return;
    }

    uint8_t *buf = mem;
    buf[0] = REC_PKT_EVENT;
    buf[4] = code;
    sys_put_le32(frame, &buf[5]);
    sys_put_le16(a, &buf[9]);
    sys_put_le16(b, &buf[11]);
    submit(buf, REC_EVENT_LEN);
}

void recording_set_stim_active(bool on) {
    atomic_set(&stim_active, on);
}

void recording_set_fault_hook(rec_fault_hook_t hook) {
    atomic_ptr_set(&fault_hook, (atomic_ptr_val_t)hook);
}

void recording_get_stats(struct rec_stats *out) {
    out->packets_sent = atomic_get(&packets_sent);
    out->packets_dropped = atomic_get(&packets_dropped);
    rhs2116_stream_get_stats(&out->stream);
}
