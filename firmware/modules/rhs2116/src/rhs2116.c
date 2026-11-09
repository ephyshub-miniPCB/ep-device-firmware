
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <rhs2116/rhs2116.h>
#include <rhs2116/rhs2116_stream.h>
#include <rhs2116/rhs2116_transport.h>

#include "rhs2116_cmd.h"
#include "rhs2116_emul.h"
#include "rhs2116_priv.h"

#define DT_DRV_COMPAT intan_rhs2116

LOG_MODULE_REGISTER(rhs2116, CONFIG_RHS2116_LOG_LEVEL);

/*
    -- IDLE MODE SPI --
*/

// clock one command and return the word clocked in alongside it, which is the
// response to the command sent two transfers earlier
static int idle_xfer(uint32_t cmd, uint32_t *rx) {
    uint32_t word;
    int ret = rhs2116_transport_xfer(rhs2116_wire(cmd), &word);

#ifdef CONFIG_RHS2116_EMUL
    // no chip: answer for the command clocked out two transfers ago
    #define EMUL_PIPE_IDLE (RHS2116_CMD_READ | ((uint32_t)RHS2116_REG_CHIP_ID << 16))
    static uint32_t pipe[2] = { EMUL_PIPE_IDLE, EMUL_PIPE_IDLE };
    if (ret == 0) {
        word = rhs2116_wire(rhs2116_emul_response(pipe[0]));
        pipe[0] = pipe[1];
        pipe[1] = cmd;
    }
#endif

    if (ret == 0 && rx) {
        *rx = rhs2116_unwire(word);
    }
    return ret;
}

// send cmd, then two dummy ROM reads to flush the 3-transaction pipeline, and
// return the response to cmd
static int idle_command(uint32_t cmd, uint32_t *result) {
    uint32_t dummy = rhs2116_cmd_read(RHS2116_REG_CHIP_ID, false, false);
    int ret = idle_xfer(cmd, NULL);
    if (ret == 0) {
        ret = idle_xfer(dummy, NULL);
    }
    if (ret == 0) {
        ret = idle_xfer(dummy, result);
    }
    return ret;
}

// send a WRITE command word and check the chip echoed its data back
static int idle_write_cmd(uint32_t cmd) {
    uint32_t result;
    int ret = idle_command(cmd, &result);
    if (ret < 0) {
        return ret;
    }
    if (result != RHS2116_WRITE_ECHO(cmd)) {
        LOG_ERR("write 0x%08x: bad echo 0x%08x", cmd, result);
        return -EIO;
    }
    return 0;
}


/*
    -- PUBLIC API --
*/

int rhs2116_write_reg(const struct device *dev, uint8_t reg, uint16_t data, bool update, bool clear) {
    struct rhs2116_data *drv = dev->data;

    if (rhs2116_stream_active()) {
        return rhs2116_stream_reg_access(true, reg, data, update, clear, NULL);
    }

    k_mutex_lock(&drv->bus_lock, K_FOREVER);
    int ret = idle_write_cmd(rhs2116_cmd_write(reg, data, update, clear));
    k_mutex_unlock(&drv->bus_lock);
    return ret;
}

// the RHS2116 SPI pipeline is 3 transactions deep: the command is clocked out on
// transaction N, the chip processes it during transaction N+1, and the response for
// the transaction-N command is only returned on transaction N+2.
int rhs2116_read_reg(const struct device *dev, uint8_t reg, bool update, bool clear, uint32_t *out_word) {
    struct rhs2116_data *drv = dev->data;

    if (rhs2116_stream_active()) {
        return rhs2116_stream_reg_access(false, reg, 0, update, clear, out_word);
    }

    k_mutex_lock(&drv->bus_lock, K_FOREVER);
    int ret = idle_command(rhs2116_cmd_read(reg, update, clear), out_word);
    k_mutex_unlock(&drv->bus_lock);
    return ret;
}

// read the chip ID register and verify the attached peripheral is really an RHS2116.
// calling program should invoke this explicitly once the device bus is ready is ready.
int rhs2116_verify_chip_id(const struct device *dev, uint8_t *out_chip_id) {
    uint32_t chip_id;

    int ret = rhs2116_read_reg(dev, RHS2116_REG_CHIP_ID, false, false, &chip_id);
    if (ret < 0) {
        LOG_ERR("Failed to read RHS2116 chip ID register: %d\n", ret);
        return ret;
    }

    if (chip_id != RHS2116_CHIP_ID) {
        LOG_ERR("Unexpected RHS2116 chip ID: got 0x%02x, expected 0x%02x\n", chip_id, RHS2116_CHIP_ID);
        return -ENODEV;
    }

    LOG_INF("RHS2116 chip ID verified: 0x%02x\n", chip_id);

    if (out_chip_id) {
        *out_chip_id = (uint8_t)chip_id;
    }
    return 0;
}

int rhs2116_get_chip_info(const struct device *dev, uint8_t *die_revision, uint8_t *num_channels) {
    uint32_t chip_spec_reg_data;

    int ret = rhs2116_read_reg(dev, RHS2116_REG_CHIP_SPEC, false, false, &chip_spec_reg_data);

    if (ret < 0) {
        LOG_ERR("Failed to read RHS2116 chip specification register: %d\n", ret);
        return ret;
    }

    *num_channels = (uint8_t)chip_spec_reg_data;
    *die_revision = (uint8_t)(chip_spec_reg_data >> 8);

    LOG_INF("Chip Die Revision Number: %d\n", *die_revision);
    LOG_INF("Number of aplifier/stimulation channels: %d\n", *num_channels);

    return 0;
}

// the datasheet's "Example Chip Initialization Procedure", except that the
// stimulators are left disabled (registers 32/33) until a stim session arms them
int rhs2116_chip_init(const struct device *dev) {
    struct rhs2116_data *drv = dev->data;
    int ret = 0;

    if (rhs2116_stream_active()) {
        return -EBUSY;
    }

    const struct rhs2116_adc_format adc_format = {
        .digout2_hiz = true,
        .digout1_hiz = true,
        .dsp_en = true,
        .dsp_cutoff = 10,
    };
    const uint32_t seq[] = {
        // disable stimulation until everything stim related is configured
        rhs2116_cmd_write(RHS2116_REG_STIM_ENABLE_A, 0x0000, false, false),
        rhs2116_cmd_write(RHS2116_REG_STIM_ENABLE_B, 0x0000, false, false),
        // power up all DC amplifiers, otherwise a hardware bug draws an extra 30.9 mA
        rhs2116_cmd_write(RHS2116_REG_DC_AMP_POWER, 0xFFFF, false, false),
        // ADC/MUX bias for 480 kS/s; re-set for the real rate when streaming starts
        rhs2116_cmd_write(RHS2116_REG_ADC_BIAS, rhs2116_enc_adc_bias(3, 5), false, false),
        // aux outputs high-Z, offset binary, DSP high-pass at 4.665 Hz (0x051A)
        rhs2116_cmd_write(RHS2116_REG_ADC_FORMAT, rhs2116_enc_adc_format(&adc_format),
                          false, false),
        // impedance check DAC powered but check disabled, DAC at midrange
        rhs2116_cmd_write(RHS2116_REG_ZCHECK_CTRL,
                          rhs2116_enc_zcheck_ctrl(0, true, false, 0, false), false, false),
        rhs2116_cmd_write(RHS2116_REG_ZCHECK_DAC, 0x80, false, false),
        // amplifier upper cutoff 7.5 kHz
        rhs2116_cmd_write(RHS2116_REG_AMP_BW_RH1, rhs2116_enc_amp_upper_bw(22, 0), false, false),
        rhs2116_cmd_write(RHS2116_REG_AMP_BW_RH2, rhs2116_enc_amp_upper_bw(23, 0), false, false),
        // lower cutoff 5 Hz, alternate (artifact recovery) cutoff 1 kHz
        rhs2116_cmd_write(RHS2116_REG_AMP_BW_RL_A, rhs2116_enc_amp_lower_bw(40, 1, 0),
                          false, false),
        rhs2116_cmd_write(RHS2116_REG_AMP_BW_RL_B, rhs2116_enc_amp_lower_bw(10, 0, 0),
                          false, false),
        // power up all AC amplifiers
        rhs2116_cmd_write(RHS2116_REG_AC_AMP_POWER, 0xFFFF, false, false),
        // fast settle off, all amplifiers on the register 6 lower cutoff
        rhs2116_cmd_write(RHS2116_REG_FAST_SETTLE, 0x0000, true, false),
        rhs2116_cmd_write(RHS2116_REG_FL_SELECT, 0xFFFF, true, false),
        // 1 uA stim step (+/-255 uA range) and matching bias voltages
        rhs2116_cmd_write(RHS2116_REG_STIM_STEP, rhs2116_enc_stim_step(98, 1, 0), false, false),
        rhs2116_cmd_write(RHS2116_REG_STIM_BIAS, rhs2116_enc_stim_bias(10, 10), false, false),
        // current-limited charge recovery: 0 V target, 1 nA limit
        rhs2116_cmd_write(RHS2116_REG_CL_RECOVERY_TARGET, 0x80, false, false),
        rhs2116_cmd_write(RHS2116_REG_CL_RECOVERY_ILIM, rhs2116_enc_cl_recovery_ilim(0, 30, 2),
                          false, false),
        // all stimulators off, negative polarity, recovery switches open
        rhs2116_cmd_write(RHS2116_REG_STIM_ON, 0x0000, true, false),
        rhs2116_cmd_write(RHS2116_REG_STIM_POL, 0x0000, true, false),
        rhs2116_cmd_write(RHS2116_REG_CHARGE_RECOVERY, 0x0000, true, false),
        rhs2116_cmd_write(RHS2116_REG_CL_RECOVERY_EN, 0x0000, true, false),
    };

    k_mutex_lock(&drv->bus_lock, K_FOREVER);

    // dummy command so the digital controller is in a known state after power-up
    (void)idle_command(rhs2116_cmd_read(RHS2116_REG_CHIP_ID, false, false), NULL);

    // CLEAR initialises the ADC; its result isn't a write echo, so it isn't checked
    ret = idle_command(rhs2116_cmd_clear(), NULL);

    for (size_t i = 0; ret == 0 && i < ARRAY_SIZE(seq); i++) {
        ret = idle_write_cmd(seq[i]);
    }

    // zero all stim magnitudes with centred trims
    for (uint8_t ch = 0; ret == 0 && ch < RHS2116_NUM_CHANNELS; ch++) {
        uint16_t zero = rhs2116_enc_stim_mag(0, RHS2116_MAG_TRIM_CENTRE);
        ret = idle_write_cmd(rhs2116_cmd_write(RHS2116_REG_NEG_MAG(ch), zero, true, false));
        if (ret == 0) {
            ret = idle_write_cmd(rhs2116_cmd_write(RHS2116_REG_POS_MAG(ch), zero, true, false));
        }
    }

    // clear the compliance monitor
    if (ret == 0) {
        ret = idle_command(rhs2116_cmd_read(RHS2116_REG_CHIP_ID, false, true), NULL);
    }

    // the state the sequence left the stimulators in, for the task-level API
    drv->step = RHS2116_STIM_STEP_1UA;
    memset(drv->neg_trim, 0, sizeof(drv->neg_trim));
    memset(drv->pos_trim, 0, sizeof(drv->pos_trim));

    k_mutex_unlock(&drv->bus_lock);

    if (ret < 0) {
        LOG_ERR("RHS2116 init sequence failed: %d", ret);
    } else {
        LOG_INF("RHS2116 initialised");
    }
    return ret;
}

// Register 0 ADC buffer / MUX bias from the datasheet table, indexed by total ADC rate
int rhs2116_set_adc_rate(const struct device *dev, uint32_t adc_rate_hz) {
    static const struct {
        uint32_t max_rate;
        uint8_t adc_buffer_bias;
        uint8_t mux_bias;
    } table[] = {
        { 120000, 32, 40 },
        { 140000, 16, 40 },
        { 175000,  8, 40 },
        { 220000,  8, 32 },
        { 280000,  8, 26 },
        { 350000,  4, 18 },
        { 440000,  3, 16 },
        { UINT32_MAX, 3, 5 },
    };

    size_t i = 0;
    while (adc_rate_hz > table[i].max_rate) {
        i++;
    }

    return rhs2116_write_reg(dev, RHS2116_REG_ADC_BIAS,
                             rhs2116_enc_adc_bias(table[i].adc_buffer_bias, table[i].mux_bias),
                             false, false);
}

static int rhs2116_init(const struct device *dev) {
    struct rhs2116_data *data = dev->data;

    k_mutex_init(&data->bus_lock);
    data->step = RHS2116_STIM_STEP_1UA; // the chip's real state is unknown until chip_init

    int ret = rhs2116_transport_init();
    if (ret < 0) {
        LOG_ERR("RHS2116 transport init failed: %d", ret);
        return ret;
    }

    LOG_INF("RHS2116 SPI transport initialised");
    return 0;
}

#define RHS2116_INIT(INST)                                                     \
    static struct rhs2116_data rhs2116_data_##INST;                            \
                                                                               \
    DEVICE_DT_INST_DEFINE(INST,                                                \
                          rhs2116_init,                                        \
                          NULL, /*power management*/                           \
                          &rhs2116_data_##INST,                                \
                          NULL,                                                \
                          POST_KERNEL,                                         \
                          CONFIG_RHS2116_INIT_PRIORITY,                        \
                          NULL);

// start the init function generation for each instance in dt
DT_INST_FOREACH_STATUS_OKAY(RHS2116_INIT)
