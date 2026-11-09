/*
 * Task-level RHS2116 API (see rhs2116.h), built only on rhs2116_write_reg() /
 * rhs2116_read_reg() and the encoders in rhs2116_regs.h. The values in the tables
 * are copied from the datasheet's register value tables.
 */
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <rhs2116/rhs2116.h>
#include <rhs2116/rhs2116_stream.h>

#include "rhs2116_priv.h"

LOG_MODULE_DECLARE(rhs2116, CONFIG_RHS2116_LOG_LEVEL);

// triggered registers need the U flag when idle; while streaming, the frame's own
// U flag applies them at the end of the frame, together with the stim writes
static bool need_update(void) {
    return !rhs2116_stream_active();
}


/*
    -- AMPLIFIERS --
*/

// "Setting Upper Bandwidth" table, descending
static const struct {
    uint32_t hz;
    uint8_t rh1_sel1, rh1_sel2, rh2_sel1, rh2_sel2;
} upper_bw[] = {
    { 20000,  8,  0,  4,  0 },
    { 15000, 11,  0,  8,  0 },
    { 10000, 17,  0, 16,  0 },
    {  7500, 22,  0, 23,  0 },
    {  5000, 33,  0, 37,  0 },
    {  3000,  3,  1, 13,  1 },
    {  2500, 13,  1, 25,  1 },
    {  2000, 27,  1, 44,  1 },
    {  1500,  1,  2, 23,  2 },
    {  1000, 46,  2, 30,  3 },
    {   750, 41,  3, 36,  4 },
    {   500, 30,  5, 43,  6 },
    {   300,  6,  9,  2, 11 },
    {   250, 42, 10,  5, 13 },
    {   200, 24, 13,  7, 16 },
    {   150, 44, 17,  8, 21 },
    {   100, 38, 26,  5, 31 },
};

// "Setting Lower Bandwidth" table, descending, in millihertz
static const struct {
    uint32_t mhz;
    uint8_t sel1, sel2, sel3;
} lower_bw[] = {
    { 1000000, 10,  0, 0 },
    {  500000, 13,  0, 0 },
    {  300000, 15,  0, 0 },
    {  250000, 17,  0, 0 },
    {  200000, 18,  0, 0 },
    {  150000, 21,  0, 0 },
    {  100000, 25,  0, 0 },
    {   75000, 28,  0, 0 },
    {   50000, 34,  0, 0 },
    {   30000, 44,  0, 0 },
    {   25000, 48,  0, 0 },
    {   20000, 54,  0, 0 },
    {   15000, 62,  0, 0 },
    {   10000,  5,  1, 0 },
    {    7500, 18,  1, 0 },
    {    5000, 40,  1, 0 },
    {    3000, 20,  2, 0 },
    {    2500, 42,  2, 0 },
    {    2000,  8,  3, 0 },
    {    1500,  9,  4, 0 },
    {    1000, 44,  6, 0 },
    {     750, 49,  9, 0 },
    {     500, 35, 17, 0 },
    {     300,  1, 40, 0 },
    {     250, 56, 54, 0 },
    {     100, 16, 60, 1 },
};

// index of the entry nearest 'want' on a log scale, in a table sorted descending
// (the boundary between two entries is their geometric mean)
static size_t nearest(uint32_t want, size_t n, uint32_t (*value)(size_t)) {
    for (size_t i = 0; i < n; i++) {
        uint32_t v = value(i);
        if (v <= want) {
            if (i > 0 && (uint64_t)want * want > (uint64_t)v * value(i - 1)) {
                return i - 1;
            }
            return i;
        }
    }
    return n - 1;
}

static uint32_t upper_hz(size_t i) {
    return upper_bw[i].hz;
}

static uint32_t lower_mhz(size_t i) {
    return lower_bw[i].mhz;
}

int rhs2116_set_bandwidth(const struct device *dev, const struct rhs2116_bandwidth *req,
                          struct rhs2116_bandwidth *actual) {
    if (req->upper_hz == 0 || req->lower_mhz == 0 || req->lower_alt_mhz == 0) {
        return -EINVAL;
    }

    size_t u = nearest(req->upper_hz, ARRAY_SIZE(upper_bw), upper_hz);
    size_t l = nearest(req->lower_mhz, ARRAY_SIZE(lower_bw), lower_mhz);
    size_t la = nearest(req->lower_alt_mhz, ARRAY_SIZE(lower_bw), lower_mhz);

    uint64_t upper_mhz = (uint64_t)upper_bw[u].hz * 1000;
    if (lower_bw[l].mhz >= upper_mhz || lower_bw[la].mhz >= upper_mhz) {
        return -EINVAL;
    }

    int ret = rhs2116_write_reg(dev, RHS2116_REG_AMP_BW_RH1,
                                rhs2116_enc_amp_upper_bw(upper_bw[u].rh1_sel1, upper_bw[u].rh1_sel2),
                                false, false);
    if (ret == 0) {
        ret = rhs2116_write_reg(dev, RHS2116_REG_AMP_BW_RH2,
                                rhs2116_enc_amp_upper_bw(upper_bw[u].rh2_sel1,
                                                         upper_bw[u].rh2_sel2),
                                false, false);
    }
    if (ret == 0) {
        ret = rhs2116_write_reg(dev, RHS2116_REG_AMP_BW_RL_A,
                                rhs2116_enc_amp_lower_bw(lower_bw[l].sel1, lower_bw[l].sel2,
                                                         lower_bw[l].sel3),
                                false, false);
    }
    if (ret == 0) {
        ret = rhs2116_write_reg(dev, RHS2116_REG_AMP_BW_RL_B,
                                rhs2116_enc_amp_lower_bw(lower_bw[la].sel1, lower_bw[la].sel2,
                                                         lower_bw[la].sel3),
                                false, false);
    }
    if (ret < 0) {
        return ret;
    }

    if (actual) {
        actual->upper_hz = upper_bw[u].hz;
        actual->lower_mhz = lower_bw[l].mhz;
        actual->lower_alt_mhz = lower_bw[la].mhz;
    }
    return 0;
}

int rhs2116_set_amp_power(const struct device *dev, uint16_t chan_mask) {
    return rhs2116_write_reg(dev, RHS2116_REG_AC_AMP_POWER, chan_mask, false, false);
}


/*
    -- STIMULATOR CONFIGURATION --
*/

// step size table, with the matching Register 35 bias voltages; indexed by enum rhs2116_stim_step
static const struct {
    uint32_t na;
    uint8_t sel1, sel2, sel3;
    uint8_t pbias, nbias;
} stim_steps[] = {
    [RHS2116_STIM_STEP_10NA]  = {    10,  64, 19, 3,  6,  6 },
    [RHS2116_STIM_STEP_20NA]  = {    20,  40, 40, 1,  7,  7 },
    [RHS2116_STIM_STEP_50NA]  = {    50,  64, 40, 0,  7,  7 },
    [RHS2116_STIM_STEP_100NA] = {   100,  30, 20, 0,  7,  7 },
    [RHS2116_STIM_STEP_200NA] = {   200,  25, 10, 0,  8,  8 },
    [RHS2116_STIM_STEP_500NA] = {   500, 101,  3, 0,  9,  9 },
    [RHS2116_STIM_STEP_1UA]   = {  1000,  98,  1, 0, 10, 10 },
    [RHS2116_STIM_STEP_2UA]   = {  2000,  94,  0, 0, 11, 11 },
    [RHS2116_STIM_STEP_5UA]   = {  5000,  38,  0, 0, 14, 14 },
    [RHS2116_STIM_STEP_10UA]  = { 10000,  15,  0, 0, 15, 15 },
};

// recovery current limit table; indexed by enum rhs2116_cl_ilim
static const struct {
    uint8_t sel1, sel2, sel3;
} cl_ilims[] = {
    [RHS2116_CL_ILIM_1NA]   = {  0, 30, 2 },
    [RHS2116_CL_ILIM_2NA]   = {  0, 15, 1 },
    [RHS2116_CL_ILIM_5NA]   = {  0, 31, 0 },
    [RHS2116_CL_ILIM_10NA]  = { 50, 15, 0 },
    [RHS2116_CL_ILIM_20NA]  = { 78,  7, 0 },
    [RHS2116_CL_ILIM_50NA]  = { 22,  3, 0 },
    [RHS2116_CL_ILIM_100NA] = { 56,  1, 0 },
    [RHS2116_CL_ILIM_200NA] = { 71,  0, 0 },
    [RHS2116_CL_ILIM_500NA] = { 26,  0, 0 },
    [RHS2116_CL_ILIM_1UA]   = {  9,  0, 0 },
};

int rhs2116_stim_set_step(const struct device *dev, enum rhs2116_stim_step step) {
    struct rhs2116_data *drv = dev->data;

    if ((unsigned int)step >= ARRAY_SIZE(stim_steps)) {
        return -EINVAL;
    }

    bool armed;
    int ret = rhs2116_stim_is_armed(dev, &armed);
    if (ret < 0) {
        return ret;
    }
    if (armed) {
        return -EBUSY;
    }

    ret = rhs2116_write_reg(dev, RHS2116_REG_STIM_STEP,
                            rhs2116_enc_stim_step(stim_steps[step].sel1, stim_steps[step].sel2,
                                                  stim_steps[step].sel3),
                            false, false);
    if (ret == 0) {
        ret = rhs2116_write_reg(dev, RHS2116_REG_STIM_BIAS,
                                rhs2116_enc_stim_bias(stim_steps[step].pbias,
                                                      stim_steps[step].nbias),
                                false, false);
    }
    if (ret == 0) {
        drv->step = step;
    }
    return ret;
}

enum rhs2116_stim_step rhs2116_stim_get_step(const struct device *dev) {
    const struct rhs2116_data *drv = dev->data;
    return drv->step;
}

uint32_t rhs2116_stim_step_na(enum rhs2116_stim_step step) {
    if ((unsigned int)step >= ARRAY_SIZE(stim_steps)) {
        return 0;
    }
    return stim_steps[step].na;
}

int rhs2116_stim_set_cl_recovery(const struct device *dev, int16_t target_mv,
                                 enum rhs2116_cl_ilim ilim) {
    if (target_mv < -1225 || target_mv > 1215 || (unsigned int)ilim >= ARRAY_SIZE(cl_ilims)) {
        return -EINVAL;
    }

    // 128 is 0 V, 9.57 mV per code, rounded to the nearest code
    int32_t centi = (int32_t)target_mv * 100;
    int32_t offset = (centi >= 0 ? centi + 478 : centi - 478) / 957;
    uint16_t dac = (uint16_t)CLAMP(128 + offset, 0, 255);

    int ret = rhs2116_write_reg(dev, RHS2116_REG_CL_RECOVERY_TARGET, dac, false, false);
    if (ret == 0) {
        ret = rhs2116_write_reg(dev, RHS2116_REG_CL_RECOVERY_ILIM,
                                rhs2116_enc_cl_recovery_ilim(cl_ilims[ilim].sel1,
                                                             cl_ilims[ilim].sel2,
                                                             cl_ilims[ilim].sel3),
                                false, false);
    }
    return ret;
}

int rhs2116_stim_set_trim(const struct device *dev, uint8_t channel, int8_t neg_trim,
                          int8_t pos_trim) {
    struct rhs2116_data *drv = dev->data;

    if (channel >= RHS2116_NUM_CHANNELS) {
        return -EINVAL;
    }
    drv->neg_trim[channel] = neg_trim;
    drv->pos_trim[channel] = pos_trim;
    return 0;
}

int rhs2116_stim_set_amplitude(const struct device *dev, uint16_t chan_mask, uint8_t neg_steps,
                               uint8_t pos_steps) {
    const struct rhs2116_data *drv = dev->data;
    bool update = need_update();
    int ret = 0;

    for (uint8_t ch = 0; ret == 0 && ch < RHS2116_NUM_CHANNELS; ch++) {
        if (!(chan_mask & BIT(ch))) {
            continue;
        }
        uint8_t neg_trim = (uint8_t)(RHS2116_MAG_TRIM_CENTRE + drv->neg_trim[ch]);
        uint8_t pos_trim = (uint8_t)(RHS2116_MAG_TRIM_CENTRE + drv->pos_trim[ch]);

        ret = rhs2116_write_reg(dev, RHS2116_REG_NEG_MAG(ch),
                                rhs2116_enc_stim_mag(neg_steps, neg_trim), update, false);
        if (ret == 0) {
            ret = rhs2116_write_reg(dev, RHS2116_REG_POS_MAG(ch),
                                    rhs2116_enc_stim_mag(pos_steps, pos_trim), update, false);
        }
    }
    return ret;
}


/*
    -- ARMING --
*/

int rhs2116_stim_arm(const struct device *dev) {
    // reading Register 42 with the M flag also clears the compliance monitor
    uint32_t on;
    int ret = rhs2116_read_reg(dev, RHS2116_REG_STIM_ON, false, true, &on);
    if (ret < 0) {
        return ret;
    }
    if ((uint16_t)on != 0) {
        return -EBUSY;
    }

    ret = rhs2116_write_reg(dev, RHS2116_REG_STIM_ENABLE_A, RHS2116_STIM_ENABLE_A_VAL,
                            false, false);
    if (ret == 0) {
        ret = rhs2116_write_reg(dev, RHS2116_REG_STIM_ENABLE_B, RHS2116_STIM_ENABLE_B_VAL,
                                false, false);
    }
    if (ret < 0) {
        // don't leave it half armed
        (void)rhs2116_write_reg(dev, RHS2116_REG_STIM_ENABLE_A, 0, false, false);
    }
    return ret;
}

int rhs2116_stim_disarm(const struct device *dev) {
    static const uint8_t regs[] = {
        RHS2116_REG_STIM_ON, RHS2116_REG_CHARGE_RECOVERY, RHS2116_REG_CL_RECOVERY_EN,
    };
    int first_err = 0;

    // keep going on errors: each write that lands makes the chip safer
    for (size_t i = 0; i < ARRAY_SIZE(regs); i++) {
        bool last = i == ARRAY_SIZE(regs) - 1;
        int ret = rhs2116_write_reg(dev, regs[i], 0, last && need_update(), false);
        first_err = first_err ? first_err : ret;
    }

    int ret = rhs2116_write_reg(dev, RHS2116_REG_STIM_ENABLE_A, 0, false, false);
    first_err = first_err ? first_err : ret;
    ret = rhs2116_write_reg(dev, RHS2116_REG_STIM_ENABLE_B, 0, false, false);
    first_err = first_err ? first_err : ret;

    if (first_err < 0) {
        LOG_ERR("stim disarm: a write failed (%d)", first_err);
    }
    return first_err;
}

int rhs2116_stim_is_armed(const struct device *dev, bool *armed) {
    uint32_t a, b;

    int ret = rhs2116_read_reg(dev, RHS2116_REG_STIM_ENABLE_A, false, false, &a);
    if (ret == 0) {
        ret = rhs2116_read_reg(dev, RHS2116_REG_STIM_ENABLE_B, false, false, &b);
    }
    if (ret < 0) {
        return ret;
    }
    *armed = (uint16_t)a == RHS2116_STIM_ENABLE_A_VAL && (uint16_t)b == RHS2116_STIM_ENABLE_B_VAL;
    return 0;
}


/*
    -- IMMEDIATE OUTPUT AND MONITORING --
*/

int rhs2116_stim_output(const struct device *dev, const struct rhs2116_stim_state *s) {
    if (s->on) {
        bool armed;
        int ret = rhs2116_stim_is_armed(dev, &armed);
        if (ret < 0) {
            return ret;
        }
        if (!armed) {
            return -EPERM;
        }
    }

    // buffer the other three, then the U flag on Register 42 applies all four together
    int ret = rhs2116_write_reg(dev, RHS2116_REG_STIM_POL, s->pol, false, false);
    if (ret == 0) {
        ret = rhs2116_write_reg(dev, RHS2116_REG_CHARGE_RECOVERY, s->recovery, false, false);
    }
    if (ret == 0) {
        ret = rhs2116_write_reg(dev, RHS2116_REG_CL_RECOVERY_EN, s->cl_recovery, false, false);
    }
    if (ret == 0) {
        ret = rhs2116_write_reg(dev, RHS2116_REG_STIM_ON, s->on, need_update(), false);
    }
    return ret;
}

int rhs2116_stim_read_compliance(const struct device *dev, uint16_t *mask, bool clear) {
    uint32_t word;
    int ret = rhs2116_read_reg(dev, RHS2116_REG_COMPLIANCE, false, clear, &word);
    if (ret == 0 && mask) {
        *mask = (uint16_t)word;
    }
    return ret;
}

int rhs2116_read_fault_current(const struct device *dev, bool *fault) {
    uint32_t word;
    int ret = rhs2116_read_reg(dev, RHS2116_REG_FAULT_CURRENT, false, false, &word);
    if (ret == 0) {
        *fault = word & BIT(0);
    }
    return ret;
}
