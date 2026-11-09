#ifndef STIM_H_
#define STIM_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

/*
 * Frame-timed stimulation on top of the RHS2116 stream.
 *
 * A program is a list of events at frame offsets from the start of a pulse. An
 * event sets the stimulator on/polarity masks (registers 42/44) and up to two extra
 * triggered registers (charge recovery 46/48, amplifier settle 10/12); all of it
 * takes effect together at the end of that frame (the frame's U flag). Between
 * events the last state is held. The pulse repeats every period_frames, count times
 * (0 = until stopped).
 *
 * Every program is checked before it can run: masks within chan_mask, the last
 * event leaves the stimulators off and recovery switches open, and (unless
 * STIM_FLAG_ALLOW_IMBALANCE) each channel's anodic and cathodic charge per pulse
 * match exactly. Amplitudes are set once (all channels in chan_mask) at start.
 *
 * Stopping, a compliance or SPI error, or the stream itself faulting all abort:
 * stimulators off at once, recovery switch closed for recovery_frames, then the
 * stimulators are disabled (registers 32/33).
 */

#define STIM_MAX_EVENTS 16

#define STIM_FLAG_ANODIC_FIRST    BIT(0) // biphasic only
#define STIM_FLAG_ALLOW_IMBALANCE BIT(1)

struct stim_event {
    uint32_t frame;     // offset from pulse start
    uint16_t on;
    uint16_t pol;       // 1 = anodic (positive), 0 = cathodic
    uint8_t n_extra;
    struct {
        uint8_t reg;
        uint16_t val;
    } extra[2];
};

struct stim_program {
    uint16_t chan_mask;
    uint8_t neg_mag;           // cathodic amplitude, in stim steps (1 uA by default)
    uint8_t pos_mag;           // anodic amplitude
    uint8_t flags;
    uint32_t period_frames;    // 0 only allowed with count == 1
    uint16_t count;            // 0 = until stopped
    uint16_t recovery_frames;  // recovery switch time on abort
    uint8_t n_events;
    struct stim_event events[STIM_MAX_EVENTS];
};

struct stim_biphasic {
    uint16_t chan_mask;
    uint8_t neg_mag;
    uint8_t pos_mag;
    uint8_t flags;
    uint16_t phase1_us;
    uint16_t interphase_us;
    uint16_t phase2_us;
    uint16_t recovery_us;      // charge recovery switch closed after phase 2
    uint32_t period_us;
    uint16_t count;
};

// frame counts the biphasic parameters rounded to
struct stim_biphasic_frames {
    uint16_t phase1;
    uint16_t interphase;
    uint16_t phase2;
    uint16_t recovery;
    uint32_t period;
};

enum stim_reject {
    STIM_OK = 0,
    STIM_NOT_RECORDING,   // stim timing needs the stream running
    STIM_BUSY,            // a program is running
    STIM_BAD_MASK,        // empty mask, or events drive channels outside it
    STIM_TOO_SHORT,       // a phase rounds to zero frames
    STIM_PERIOD_TOO_SHORT,
    STIM_UNBALANCED,      // anodic and cathodic charge differ
    STIM_BAD_EVENTS,      // unordered, too many, forbidden register, or not off at the end
    STIM_NO_PROGRAM,
    STIM_RATE_CHANGED,    // program was built for a different sample rate
};

enum stim_abort_reason {
    STIM_ABORT_STOPPED = 1,
    STIM_ABORT_COMPLIANCE,
    STIM_ABORT_SPI_ERROR,
    STIM_ABORT_STREAM_FAULT,
};

/* Compiles a biphasic pulse train against the running stream's frame rate and
 * loads it. Requires recording to be active. */
int stim_load_biphasic(const struct stim_biphasic *p, struct stim_biphasic_frames *frames,
                       enum stim_reject *why);

/* Loads a raw event program (frame offsets at the current rate). */
int stim_load_program(const struct stim_program *prog, enum stim_reject *why);

/* Sets amplitudes, enables the stimulators and starts the loaded program at the
 * next frame to be scheduled. Blocks while the setup register writes go out. */
int stim_start(const struct device *dev, enum stim_reject *why);

/* Aborts a running program and waits (up to ~1 s) for the safe shutdown. */
void stim_stop(void);

bool stim_running(void);

#endif /* STIM_H_ */
