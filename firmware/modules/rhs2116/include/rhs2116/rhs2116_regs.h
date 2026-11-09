#ifndef RHS2116_REGS_H_
#define RHS2116_REGS_H_

/*
 * Register level of the RHS2116 driver API: the RHS2116 register map and pure field encoders. Nothing here touches hardware:
 * each encoder packs datasheet fields into the 16-bit data word for one register,
 * to be written with rhs2116_write_reg(). They have no Zephyr dependencies, so they
 * can be unit-tested on a PC.
 *
 * Encoder arguments follow the column order of the datasheet tables (sel1, sel2,
 * sel3), so table rows can be copied across directly. Registers whose data is a
 * plain per-channel mask or a single 8-bit value (3, 8, 10, 12, 32, 33, 36, 38, 42,
 * 44, 46, 48) have no encoder: write the value as is.
 *
 * Writing registers directly bypasses the checks in the task-level API
 * (rhs2116.h), e.g. that the Register 35 bias matches the Register 34 step size.
 */

#include <stdbool.h>
#include <stdint.h>

/*
    -- REGISTER MAP --
*/
// amplifier control
#define RHS2116_REG_ADC_BIAS           0
#define RHS2116_REG_ADC_FORMAT         1
#define RHS2116_REG_ZCHECK_CTRL        2
#define RHS2116_REG_ZCHECK_DAC         3
#define RHS2116_REG_AMP_BW_RH1         4
#define RHS2116_REG_AMP_BW_RH2         5
#define RHS2116_REG_AMP_BW_RL_A        6
#define RHS2116_REG_AMP_BW_RL_B        7
#define RHS2116_REG_AC_AMP_POWER       8
#define RHS2116_REG_FAST_SETTLE        10  // triggered
#define RHS2116_REG_FL_SELECT          12  // triggered

// stimulation control
#define RHS2116_REG_STIM_ENABLE_A      32
#define RHS2116_REG_STIM_ENABLE_B      33
#define RHS2116_REG_STIM_STEP          34
#define RHS2116_REG_STIM_BIAS          35
#define RHS2116_REG_CL_RECOVERY_TARGET 36
#define RHS2116_REG_CL_RECOVERY_ILIM   37
#define RHS2116_REG_DC_AMP_POWER       38
#define RHS2116_REG_COMPLIANCE         40  // read only, cleared by the M flag
#define RHS2116_REG_STIM_ON            42  // triggered
#define RHS2116_REG_STIM_POL           44  // triggered
#define RHS2116_REG_CHARGE_RECOVERY    46  // triggered
#define RHS2116_REG_CL_RECOVERY_EN     48  // triggered
#define RHS2116_REG_FAULT_CURRENT      50  // read only, not latched

// stimulation magnitudes, one register per channel (triggered)
#define RHS2116_REG_NEG_MAG_BASE       64
#define RHS2116_REG_POS_MAG_BASE       96
#define RHS2116_REG_NEG_MAG(ch)        (RHS2116_REG_NEG_MAG_BASE + ((ch) & 0x0F))
#define RHS2116_REG_POS_MAG(ch)        (RHS2116_REG_POS_MAG_BASE + ((ch) & 0x0F))

// read-only ROM
#define RHS2116_REG_CHIP_SPEC          254 // die revision [15:8], channel count [7:0]
#define RHS2116_REG_CHIP_ID            255

#define RHS2116_CHIP_ID                0x20

// magic values that arm the stimulators (both must be written)
#define RHS2116_STIM_ENABLE_A_VAL      0xAAAA
#define RHS2116_STIM_ENABLE_B_VAL      0x00FF

// nominal (centred) magnitude trim; the trim adjusts the DAC by roughly +/-28%
#define RHS2116_MAG_TRIM_CENTRE        0x80

// triggered registers only take their new value when a command with the U flag is sent
static inline bool rhs2116_reg_is_triggered(uint8_t reg) {
    return reg == RHS2116_REG_FAST_SETTLE || reg == RHS2116_REG_FL_SELECT ||
           reg == RHS2116_REG_STIM_ON || reg == RHS2116_REG_STIM_POL ||
           reg == RHS2116_REG_CHARGE_RECOVERY || reg == RHS2116_REG_CL_RECOVERY_EN ||
           (reg >= RHS2116_REG_NEG_MAG_BASE && reg < RHS2116_REG_NEG_MAG_BASE + 16) ||
           (reg >= RHS2116_REG_POS_MAG_BASE && reg < RHS2116_REG_POS_MAG_BASE + 16);
}


/*
    -- AMPLIFIER CONTROL ENCODERS (REG 0-7) --
*/
// Register 0: Supply Sensor and ADC Buffer Bias Current
static inline uint16_t rhs2116_enc_adc_bias(uint8_t adc_buffer_bias, uint8_t mux_bias) {
    return ((adc_buffer_bias & 0x3F) << 6) | (mux_bias & 0x3F);
}

// Register 1: ADC Output Format, DSP Offset Removal, and Auxiliary Digital Outputs
struct rhs2116_adc_format {
    bool digout_od;
    bool digout2;
    bool digout2_hiz;
    bool digout1;
    bool digout1_hiz;
    bool weak_miso;
    bool twos_comp;
    bool abs_mode;
    bool dsp_en;
    uint8_t dsp_cutoff;     // 4-bit index into the datasheet DSP cutoff table
};

static inline uint16_t rhs2116_enc_adc_format(const struct rhs2116_adc_format *f) {
    return (f->digout_od << 12) | (f->digout2 << 11) | (f->digout2_hiz << 10) |
           (f->digout1 << 9) | (f->digout1_hiz << 8) | (f->weak_miso << 7) |
           (f->twos_comp << 6) | (f->abs_mode << 5) | (f->dsp_en << 4) |
           (f->dsp_cutoff & 0x0F);
}

// Register 2: Impedance Check Control
static inline uint16_t rhs2116_enc_zcheck_ctrl(uint8_t select, bool dac_power, bool load,
                                               uint8_t scale, bool en) {
    return ((select & 0x3F) << 8) | (dac_power << 6) | (load << 5) | ((scale & 0x03) << 3) |
           (uint16_t)en;
}

// Registers 4 (RH1) and 5 (RH2): upper cutoff, values from the "Setting Upper Bandwidth" table
static inline uint16_t rhs2116_enc_amp_upper_bw(uint8_t sel1, uint8_t sel2) {
    return ((sel2 & 0x1F) << 6) | (sel1 & 0x3F);
}

// Registers 6 (RL_A) and 7 (RL_B): lower cutoff, values from the "Setting Lower Bandwidth" table
static inline uint16_t rhs2116_enc_amp_lower_bw(uint8_t sel1, uint8_t sel2, uint8_t sel3) {
    return ((sel3 & 0x01) << 13) | ((sel2 & 0x3F) << 7) | (sel1 & 0x7F);
}


/*
    -- STIMULATION CONTROL ENCODERS (REG 34-37, 64-111) --
*/
// Register 34: Stimulation Current Step Size, values from the step size table
static inline uint16_t rhs2116_enc_stim_step(uint8_t sel1, uint8_t sel2, uint8_t sel3) {
    return ((sel3 & 0x03) << 13) | ((sel2 & 0x3F) << 7) | (sel1 & 0x7F);
}

// Register 35: Stimulation Bias Voltages; must match the step size (see the datasheet table)
static inline uint16_t rhs2116_enc_stim_bias(uint8_t pbias, uint8_t nbias) {
    return ((pbias & 0x0F) << 4) | (nbias & 0x0F);
}

// Register 37: Charge Recovery Current Limit, values from the recovery current limit table
static inline uint16_t rhs2116_enc_cl_recovery_ilim(uint8_t sel1, uint8_t sel2, uint8_t sel3) {
    return ((sel3 & 0x03) << 13) | ((sel2 & 0x3F) << 7) | (sel1 & 0x7F);
}

// Registers 64-79 / 96-111: stimulation magnitude in steps, with trim (RHS2116_MAG_TRIM_CENTRE = nominal)
static inline uint16_t rhs2116_enc_stim_mag(uint8_t magnitude, uint8_t trim) {
    return ((uint16_t)trim << 8) | magnitude;
}

#endif /* RHS2116_REGS_H_ */
