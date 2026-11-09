#ifndef RHS2116_H_
#define RHS2116_H_

/*
 * RHS2116 driver API, in two layers.
 *
 *   Task level (this file)  configure and drive the chip in physical units. The
 *                           driver writes the register sequences and keeps the
 *                           datasheet's rules (matching step size and bias,
 *                           stimulators disabled while they're reconfigured,
 *                           compliance monitor cleared before arming).
 *   Register level          rhs2116_write_reg() / rhs2116_read_reg() below, plus the
 *                           register map and field encoders in rhs2116_regs.h. Gives
 *                           full control of the chip, with none of those checks.
 *
 * Every function takes the devicetree device, and all driver state (step size,
 * trims) is per instance. NOTE: the SPI transport and the stream are still single
 * instances, so only one RHS2116 works in practice for now.
 *
 * All calls may sleep and must not be made from an ISR. They work both when idle
 * (direct SPI) and while streaming (each register access goes into an auxiliary slot
 * of the stream and the call blocks until its response comes back). Unless noted,
 * they return 0 or a negative errno: -EIO if the chip's write echo is wrong.
 *
 * Frame-timed stimulation (pulse trains with exact per-frame timing) is not done
 * here. It runs through rhs2116_stream_set_stim_provider() in rhs2116_stream.h. A
 * typical program sets up with this API (step, amplitudes, arm), then lets its
 * stim provider switch the stimulators on and off frame by frame (see stim.c).
 */

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

#include "rhs2116_regs.h"

// the RHS2116 has a fixed 16 amplifier/stimulation channels
#define RHS2116_NUM_CHANNELS 16


/*
    -- SETUP AND IDENTIFICATION --
*/

// read the chip ID register and verify it matches the expected RHS2116 chip ID.
// stores the value read in *chip_id if chip_id is not NULL.
// returns 0 on a match, -ENODEV if the value doesn't match (or the SPI transaction failed)
int rhs2116_verify_chip_id(const struct device *dev, uint8_t *chip_id);

// read the chip die revision and channel number register and store the die revision
// and number of channels.
int rhs2116_get_chip_info(const struct device *dev, uint8_t *die_revision, uint8_t *num_channels);

/* Run the power-up register initialisation. Idle mode only (-EBUSY while streaming).
 * Leaves: all amplifiers on, 7.5 kHz / 5 Hz bandwidth (1 kHz alternate lower cutoff),
 * 1 uA stim step, all magnitudes zero with centred trims, CL charge recovery at
 * 0 V / 1 nA, stimulators off and disarmed, compliance monitor cleared. */
int rhs2116_chip_init(const struct device *dev);

// set the Register 0 ADC/MUX bias currents for a total ADC rate (all channels)
int rhs2116_set_adc_rate(const struct device *dev, uint32_t adc_rate_hz);


/*
    -- AMPLIFIERS --
*/

struct rhs2116_bandwidth {
    uint32_t upper_hz;       // 100 Hz to 20 kHz
    uint32_t lower_mhz;      // 0.1 Hz to 1 kHz, in millihertz (Register 6)
    uint32_t lower_alt_mhz;  // alternate lower cutoff used for artifact recovery (Register 7)
};

/* Set the amplifier bandwidth (Registers 4-7). Each corner snaps to the nearest
 * entry (on a log scale) in the datasheet tables; the values actually used are
 * stored in *actual if it is not NULL. -EINVAL if a corner is zero or a lower
 * cutoff isn't below the upper one. */
int rhs2116_set_bandwidth(const struct device *dev, const struct rhs2116_bandwidth *req,
                          struct rhs2116_bandwidth *actual);

/* Power the AC amplifiers in chan_mask on and the rest off (Register 8). The DC
 * amplifiers (Register 38) always stay powered: switching them off triggers a
 * hardware bug that increases the supply current. */
int rhs2116_set_amp_power(const struct device *dev, uint16_t chan_mask);


/*
    -- STIMULATOR CONFIGURATION --
*/

// stimulation current per magnitude step. These are the only values the datasheet supports.
enum rhs2116_stim_step {
    RHS2116_STIM_STEP_10NA,   // +/-2.55 uA full scale
    RHS2116_STIM_STEP_20NA,   // +/-5.10 uA
    RHS2116_STIM_STEP_50NA,   // +/-12.75 uA
    RHS2116_STIM_STEP_100NA,  // +/-25.5 uA
    RHS2116_STIM_STEP_200NA,  // +/-51.0 uA
    RHS2116_STIM_STEP_500NA,  // +/-127.5 uA
    RHS2116_STIM_STEP_1UA,    // +/-255 uA
    RHS2116_STIM_STEP_2UA,    // +/-510 uA
    RHS2116_STIM_STEP_5UA,    // +/-1.275 mA
    RHS2116_STIM_STEP_10UA,   // +/-2.55 mA
};

// current limit of the current-limited (CL) charge recovery drivers
enum rhs2116_cl_ilim {
    RHS2116_CL_ILIM_1NA,
    RHS2116_CL_ILIM_2NA,
    RHS2116_CL_ILIM_5NA,
    RHS2116_CL_ILIM_10NA,
    RHS2116_CL_ILIM_20NA,
    RHS2116_CL_ILIM_50NA,
    RHS2116_CL_ILIM_100NA,
    RHS2116_CL_ILIM_200NA,
    RHS2116_CL_ILIM_500NA,
    RHS2116_CL_ILIM_1UA,
};

/* Set the global stimulation step size (Register 34) together with the matching
 * bias voltages (Register 35). -EBUSY while the stimulators are armed, -EINVAL for
 * an unknown step. */
int rhs2116_stim_set_step(const struct device *dev, enum rhs2116_stim_step step);

// the step size last set through rhs2116_stim_set_step() (or by rhs2116_chip_init())
enum rhs2116_stim_step rhs2116_stim_get_step(const struct device *dev);

// size of a step in nanoamps (0 for an unknown step)
uint32_t rhs2116_stim_step_na(enum rhs2116_stim_step step);

/* Set the CL charge recovery target voltage (Register 36, -1225 to +1215 mV in
 * 9.57 mV steps, rounded to the nearest) and current limit (Register 37). -EINVAL
 * if target_mv or ilim is out of range. */
int rhs2116_stim_set_cl_recovery(const struct device *dev, int16_t target_mv,
                                 enum rhs2116_cl_ilim ilim);

/* Store a per-channel calibration trim for the negative and positive current DACs,
 * as an offset from nominal (-128..127, roughly +/-28%). The trims are applied by
 * every later rhs2116_stim_set_amplitude() on that channel. -EINVAL for a bad channel. */
int rhs2116_stim_set_trim(const struct device *dev, uint8_t channel, int8_t neg_trim,
                          int8_t pos_trim);

/* Set the cathodic (negative) and anodic (positive) amplitude, in steps of the
 * current step size, for every channel in chan_mask (Registers 64-79, 96-111).
 * Applies immediately when idle; while streaming, at the end of the current frame. */
int rhs2116_stim_set_amplitude(const struct device *dev, uint16_t chan_mask, uint8_t neg_steps,
                               uint8_t pos_steps);


/*
    -- ARMING --
*/

/* Clear the compliance monitor, then enable the stimulators (the Register 32/33
 * magic values). The stim_en pin must also be high. -EBUSY if any stimulator is
 * currently switched on. */
int rhs2116_stim_arm(const struct device *dev);

/* Return to the safe rest state: all stimulators off, charge recovery switches and
 * CL drivers open (Registers 42, 46, 48, applied together), then the stimulators
 * disabled (Registers 32/33). Any charge recovery the caller wants has to happen
 * before this, e.g. with rhs2116_stim_output(). Every write is attempted even if
 * an earlier one fails. Safe to call when already disarmed. */
int rhs2116_stim_disarm(const struct device *dev);

/* Read Registers 32/33 back from the chip and report whether they hold the arming
 * values, so the answer is right whoever wrote them (this API, a stim provider or
 * raw writes). */
int rhs2116_stim_is_armed(const struct device *dev, bool *armed);


/*
    -- IMMEDIATE OUTPUT AND MONITORING --
*/

// one bit per channel for each triggered stimulation control register
struct rhs2116_stim_state {
    uint16_t on;           // Register 42: current source on
    uint16_t pol;          // Register 44: 1 = anodic (positive), 0 = cathodic (negative)
    uint16_t recovery;     // Register 46: charge recovery switch to stim_GND closed
    uint16_t cl_recovery;  // Register 48: CL charge recovery driver connected
};

/* Write all four stimulation control registers so they take effect together. For
 * ad hoc use (testing, manual recovery); when idle the timing is only as good as
 * the calling thread's, and while streaming the stream rewrites Registers 42/44
 * every frame, so use the stream's stim provider for timed pulses. -EPERM if s->on
 * is non-zero while the stimulators are disarmed. */
int rhs2116_stim_output(const struct device *dev, const struct rhs2116_stim_state *s);

/* Read the compliance monitor (Register 40) into *mask if mask is not NULL: a bit
 * is set for each stimulator that hit its voltage limit since the last clear.
 * Clears it afterwards if 'clear'. */
int rhs2116_stim_read_compliance(const struct device *dev, uint16_t *mask, bool clear);

// read the fault current detector (Register 50). Live state, not latched.
int rhs2116_read_fault_current(const struct device *dev, bool *fault);


/*
    -- REGISTER LEVEL --
*/

/* Write 'data' to 'reg' and check the chip's echo of it. Use the encoders in
 * rhs2116_regs.h to build 'data'. 'update' sets the U flag (triggered registers
 * take their new value), 'clear' sets the M flag (clears the compliance monitor).
 * While streaming every frame ends with a U flag, so triggered registers take
 * effect at the end of the frame regardless of 'update'.
 *
 * These calls skip the task-level checks above. After raw writes to Registers
 * 34/35 or the magnitudes, rhs2116_stim_get_step() and the stored trims no longer
 * describe the chip. */
int rhs2116_write_reg(const struct device *dev, uint8_t reg, uint16_t data, bool update, bool clear);

// read 'reg' and store the 32-bit result in *out_word
int rhs2116_read_reg(const struct device *dev, uint8_t reg, bool update, bool clear, uint32_t *out_word);

#endif /* RHS2116_H_ */
