#ifndef RHS2116_STREAM_H_
#define RHS2116_STREAM_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#include "rhs2116_transport.h"

/*
 * Continuous RHS2116 command stream. Every frame (one sample period) is
 *
 *   CONVERT(ch0) .. CONVERT(chN-1) | misc | misc | WRITE(44, pol) | WRITE(42, on, U)
 *
 * clocked out back to back at one word per slot by the transport. The misc slots
 * carry stim-event writes, queued register reads/writes and housekeeping reads; the
 * last two set the stimulators, and their U flag makes every triggered register
 * change written during the frame take effect together at the end of it.
 *
 * Each ring word has a tag saying what was sent, so the response two words later
 * can be routed: CONVERT results to the sample sink, WRITE echoes are checked,
 * READ results go to whoever asked.
 *
 * Fail safe: as soon as a block has been clocked out, its misc and stim slots are
 * reset (in the ISR) to dummy reads and "stimulators off" before the stream thread
 * refills them, so a late refill can never replay an old stim command.
 */

#define RHS2116_STREAM_OVERHEAD_SLOTS 4 // 2 misc + 2 stim per frame

// 16 MHz pacing ticks per word; 32 bits at 8 MHz is 4 us plus START latency and CS gap.
// Tighten once measured on a scope.
#define RHS2116_STREAM_MIN_SLOT_TICKS 88
#define RHS2116_STREAM_TICK_HZ        16000000u
#define RHS2116_STREAM_MIN_RATE_HZ    100

enum rhs2116_stream_reject {
    RHS2116_STREAM_OK = 0,
    RHS2116_STREAM_BAD_MASK,       // no channels selected
    RHS2116_STREAM_RATE_TOO_LOW,   // below RHS2116_STREAM_MIN_RATE_HZ
    RHS2116_STREAM_RATE_TOO_HIGH,  // SPI can't fit the frame in one sample period
    RHS2116_STREAM_BUSY,           // already streaming
};

struct rhs2116_stream_layout {
    uint16_t chan_mask;
    uint8_t n_ch;
    uint8_t chans[16];          // channel number of each CONVERT slot, in mask order
    uint8_t slots_per_frame;
    uint32_t slot_ticks;
    uint32_t rate_mhz;          // actual per-channel sample rate, milli-Hz
    uint16_t frames_per_block;
    uint16_t guard_frames;
};

// per-frame stimulation commands, supplied by the stim provider
struct rhs2116_frame_cmds {
    uint16_t stim_on;
    uint16_t stim_pol;
    uint8_t n_extra;            // up to 2 extra register writes in this frame's misc slots
    struct {
        uint8_t reg;
        uint16_t val;
    } extra[2];
};

enum rhs2116_stream_status {
    RHS2116_STREAM_STATUS_SPI_ERROR,   // a WRITE echo or dummy read came back wrong (value: word)
    RHS2116_STREAM_STATUS_COMPLIANCE,  // compliance monitor bits set (value: register 40)
    RHS2116_STREAM_STATUS_STOPPED,     // the stream stopped itself on a fault (value: -errno)
};

struct rhs2116_stream_sink {
    // a complete frame of raw AC amplifier codes, slot order (stream thread)
    void (*frame)(uint32_t frame_idx, const uint16_t *samples, uint8_t n_ch);
    // problems seen by the stream (stream thread)
    void (*status)(enum rhs2116_stream_status status, uint32_t frame_idx, uint32_t value);
};

/* Called on the stream thread for every frame as it is written into the ring, in
 * frame order, some milliseconds ahead of being clocked out. Fills *cmds (which
 * starts as all-off). */
typedef void (*rhs2116_stim_provider_t)(uint32_t frame_idx, struct rhs2116_frame_cmds *cmds);

struct rhs2116_stream_stats {
    uint32_t frames;
    uint32_t blocks;
    uint32_t spi_errors;
    uint32_t compliance_events;
    struct rhs2116_transport_stats transport;
};

/* Works out the frame layout for a channel mask and requested per-channel rate.
 * Returns 0, or -EINVAL with *why set. */
int rhs2116_stream_plan(uint16_t chan_mask, uint32_t rate_hz, struct rhs2116_stream_layout *out,
                        enum rhs2116_stream_reject *why);

// the highest per-channel rate the SPI can sustain for n_ch channels
uint32_t rhs2116_stream_max_rate_hz(uint8_t n_ch);

int rhs2116_stream_start(const struct device *dev, uint16_t chan_mask, uint32_t rate_hz,
                         const struct rhs2116_stream_sink *sink,
                         struct rhs2116_stream_layout *layout, enum rhs2116_stream_reject *why);

void rhs2116_stream_stop(void);

bool rhs2116_stream_active(void);

// frame index of the most recently processed (received) frame
uint32_t rhs2116_stream_current_frame(void);

const struct rhs2116_stream_layout *rhs2116_stream_get_layout(void);

void rhs2116_stream_get_stats(struct rhs2116_stream_stats *stats);

// install (or clear, with NULL) the per-frame stimulation command provider
void rhs2116_stream_set_stim_provider(rhs2116_stim_provider_t provider);

/* Register access through a misc slot while streaming; blocks until the response
 * arrives. For a write, *result (if given) gets the echo. Used by rhs2116_*_reg. */
int rhs2116_stream_reg_access(bool write, uint8_t reg, uint16_t data, bool update, bool clear,
                              uint32_t *result);

#endif /* RHS2116_STREAM_H_ */
