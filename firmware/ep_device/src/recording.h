#ifndef RECORDING_H_
#define RECORDING_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#include <rhs2116/rhs2116_stream.h>

/*
 * Recording: runs the RHS2116 stream and ships its samples over the BLE data
 * characteristic. Mirrored in base_node (shell decoding) and pc_side
 * (rec_packets.py); keep them in sync.
 *
 * Every data characteristic notification starts with:
 *
 *   byte 0      type     enum rec_pkt_type
 *   byte 1      flags    REC_FLAG_*
 *   byte 2..3   seq      u16 LE, +1 per packet including ones dropped on the device
 *
 * SAMPLES packet (type 0x01), header REC_SAMPLES_HDR_LEN bytes:
 *   byte 4..7   first frame index, u32 LE (frame 0 = first sample of the recording)
 *   byte 8      n_frames
 *   byte 9      bits per sample
 *   byte 10..11 channel mask, u16 LE
 *   byte 12..   n_frames frames of n_ch samples (ascending channel order),
 *               bit-packed LSB first with no padding between samples; the last
 *               byte is zero padded. A sample is the top 'bits' bits of the
 *               RHS2116's 16-bit offset-binary AC amplifier code.
 *
 * EVENT packet (type 0x02), one event, REC_EVENT_LEN bytes:
 *   byte 4      code     enum rec_event_code
 *   byte 5..8   frame index the event refers to, u32 LE
 *   byte 9..10  a        u16 LE (code specific)
 *   byte 11..12 b        u16 LE (code specific)
 */

#define REC_SAMPLES_HDR_LEN 12
#define REC_EVENT_LEN       13

enum rec_pkt_type {
    REC_PKT_SAMPLES = 0x01,
    REC_PKT_EVENT   = 0x02,
};

#define REC_FLAG_STIM_ACTIVE BIT(0) // a stim program is running
#define REC_FLAG_DROPPED     BIT(1) // packets were dropped on the device since the last one

enum rec_event_code {
    REC_EVT_STIM_EDGE      = 0x01, // a: stim on mask, b: polarity mask (takes effect after frame)
    REC_EVT_RECOVERY       = 0x02, // a: charge recovery switch mask
    REC_EVT_STIM_DONE      = 0x03, // a: pulses delivered
    REC_EVT_STIM_ABORT     = 0x04, // a: enum stim_abort_reason
    REC_EVT_COMPLIANCE     = 0x05, // a: compliance monitor bits
    REC_EVT_SPI_ERROR      = 0x06, // a: low half, b: high half of the bad response
    REC_EVT_STREAM_STOPPED = 0x07, // a: -errno
};

// START_RECORDING result codes; 0-4 match enum rhs2116_stream_reject
enum rec_reject {
    REC_OK            = RHS2116_STREAM_OK,
    REC_BAD_MASK      = RHS2116_STREAM_BAD_MASK,
    REC_RATE_TOO_LOW  = RHS2116_STREAM_RATE_TOO_LOW,
    REC_RATE_TOO_HIGH = RHS2116_STREAM_RATE_TOO_HIGH, // over the SPI budget: hard limit
    REC_BUSY          = RHS2116_STREAM_BUSY,
    REC_BAD_BITS      = 5,
};

struct rec_params {
    uint16_t chan_mask;
    uint32_t rate_hz;
    uint8_t bits; // 8, 10, 12 or 16
};

struct rec_info {
    enum rec_reject reject;
    uint32_t rate_mhz;          // actual per-channel rate, milli-Hz
    uint32_t slot_ticks;
    uint8_t slots_per_frame;
    uint8_t frames_per_packet;
    uint32_t required_bps;      // BLE payload bytes/s this recording needs
    uint32_t link_bps;          // last link test result on this device, 0 if none
    uint32_t max_rate_hz;       // SPI limit for this channel count
};

/* Validates params against the SPI budget (hard limit) and starts recording.
 * *info is filled in either way. The BLE budget is not enforced here: the caller
 * compares required_bps with a measured link rate and warns. */
int recording_start(const struct device *dev, const struct rec_params *params,
                     struct rec_info *info);

void recording_stop(void);

bool recording_active(void);

/* Queues an event packet; callable from any thread (not ISRs). */
void recording_event(enum rec_event_code code, uint32_t frame, uint16_t a, uint16_t b);

/* Sets/clears REC_FLAG_STIM_ACTIVE on following packets. */
void recording_set_stim_active(bool active);

/* Hook for stream problems (stream thread context); the stim module uses it to
 * abort on faults. */
typedef void (*rec_fault_hook_t)(enum rhs2116_stream_status status, uint32_t frame,
                                 uint32_t value);
void recording_set_fault_hook(rec_fault_hook_t hook);

struct rec_stats {
    uint32_t packets_sent;
    uint32_t packets_dropped;
    struct rhs2116_stream_stats stream;
};
void recording_get_stats(struct rec_stats *stats);

#endif /* RECORDING_H_ */
