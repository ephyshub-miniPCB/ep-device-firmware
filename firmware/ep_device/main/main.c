#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble_handler.h"
#include "cmd_handler.h"
#include "device_info.h"
#include <rhs2116/rhs2116.h>
#include <rhs2116/rhs2116_transport.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *rhs2116_dev;

// store info about peripheral
// read-only (ROM) information retrieved once from the RHS2116 after its chip ID is verified
struct rhs2116_chip_info {
    uint8_t chip_id;
    uint8_t die_revision;
    uint8_t num_channels;
};

static struct rhs2116_chip_info rhs2116_info;

static void on_ble_rx(const uint8_t *data, uint16_t len) {
    (void)cmd_handler_submit(data, len);
}

// stimulation must not outlive the link that controls it
static void on_ble_disconnected(void) {
    cmd_handler_stop_all();
}

static const struct ble_handler_cb ble_cb = {
    .received = on_ble_rx,
    .disconnected = on_ble_disconnected,
};

#ifdef CONFIG_RHS2116_DMA_PROBE
#include <rhs2116/rhs2116_stream.h>

static uint32_t soak_next_frame;
static uint32_t soak_gaps;
static int soak_stopped;

static void soak_frame(uint32_t frame_idx, const uint16_t *samples, uint8_t n_ch) {
    ARG_UNUSED(samples);
    ARG_UNUSED(n_ch);
    if (frame_idx != soak_next_frame) {
        soak_gaps++;
    }
    soak_next_frame = frame_idx + 1;
}

static void soak_status(enum rhs2116_stream_status status, uint32_t frame_idx, uint32_t value) {
    ARG_UNUSED(frame_idx);
    if (status == RHS2116_STREAM_STATUS_STOPPED) {
        soak_stopped = (int)value;
    }
}

/* Streams a spread of layouts for a few seconds each (no BLE traffic, just
 * advertising) and logs the stream and rewind counters, to check the transport on
 * its own. "ok" with pos_faults 0 and gaps 0 is a pass. */
static void stream_soak(void) {
    static const struct rhs2116_stream_sink sink = { soak_frame, soak_status };
    static const struct { uint16_t mask; uint32_t rate; } cases[] = {
        { 0x0007, 20000 }, { 0x000f, 20000 }, { 0x0007, 20000 }, { 0x000f, 20000 },
        { 0x0001, 20000 }, { 0x0003, 20000 }, { 0x0007, 15000 }, { 0x0007, 16000 },
        { 0x001f, 20000 }, { 0x007f, 10000 }, { 0x00ff, 10000 }, { 0xffff, 9090 },
        // long slots (>= REWIND_LONG_SLOT_TICKS): rewinds confirmed from a compare
        { 0xffff, 1000 }, { 0x0001, 1000 }, { 0x0001, 100 },
    };

    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        struct rhs2116_stream_layout layout;
        enum rhs2116_stream_reject why;
        struct rhs2116_stream_stats st;

        soak_next_frame = 0;
        soak_gaps = 0;
        soak_stopped = 0;
        int ret = rhs2116_stream_start(rhs2116_dev, cases[i].mask, cases[i].rate, &sink, &layout,
                                       &why);
        if (ret < 0) {
            LOG_ERR("soak 0x%04x @ %u: start failed %d (why %d)", cases[i].mask, cases[i].rate,
                    ret, why);
            continue;
        }
        k_msleep(i < 4 ? 5000 : (cases[i].rate >= 1000 ? 2000 : 3000));
        rhs2116_stream_stop();
        rhs2116_stream_get_stats(&st);
        LOG_INF("soak 0x%04x @ %5u (%u x %u ticks): %s, frames %u gaps %u spi_err %u | "
                "rewinds %u gap_misses %u max_guard %u/%u pos_faults %u",
                cases[i].mask, cases[i].rate, layout.slots_per_frame, layout.slot_ticks,
                soak_stopped ? "FAULT" : "ok", st.frames, soak_gaps, st.spi_errors,
                st.transport.rewinds, st.transport.late_rewinds, st.transport.max_guard_used,
                layout.guard_frames * layout.slots_per_frame, st.transport.pos_faults);
        if (soak_stopped) {
            LOG_ERR("soak stopped with %d", soak_stopped);
        }
        k_msleep(50);
    }
    LOG_INF("soak done");
}
#endif

int main(void) {
#ifdef CONFIG_RHS2116_DMA_PROBE
    // before BLE starts: the probe locks interrupts for ~100 us at a time.
    // A long slot down to the stream's shortest (RHS2116_STREAM_MIN_SLOT_TICKS).
    rhs2116_transport_dma_probe(160);
    rhs2116_transport_dma_probe(114);
    rhs2116_transport_dma_probe(100);
    rhs2116_transport_dma_probe(88);
#endif

	// initialise ble connection to base
    int ble_ret = ble_handler_init(&ble_cb);
    if (ble_ret < 0) {
        return ble_ret;
    }

	// -- INITIALISE HARDWARE --

#ifdef CONFIG_RHS2116_EMUL
    LOG_WRN("RHS2116 EMULATION ENABLED: MISO is ignored, all chip data is synthetic");
#endif
	
    rhs2116_dev = DEVICE_DT_GET(DT_NODELABEL(ep_dev));

    if (!device_is_ready(rhs2116_dev)) {
        LOG_ERR("RHS2116 device is not ready");
        device_info_set_rhs2116_error(-ENODEV);
        return -ENODEV;
    }

    LOG_INF("RHS2116 device is ready");

    int ret = rhs2116_verify_chip_id(rhs2116_dev, &rhs2116_info.chip_id);
    if (ret < 0) {
        LOG_ERR("RHS2116 chip ID verification failed: %d", ret);
        device_info_set_rhs2116_error(ret);
        return ret;
    }

    ret = rhs2116_get_chip_info(rhs2116_dev, &rhs2116_info.die_revision, &rhs2116_info.num_channels);
    if (ret < 0) {
        LOG_ERR("RHS2116 chip info read failed: %d", ret);
        device_info_set_rhs2116_error(ret);
        return ret;
    }

    ret = rhs2116_chip_init(rhs2116_dev);
    if (ret < 0) {
        LOG_ERR("RHS2116 register init failed: %d", ret);
        device_info_set_rhs2116_error(ret);
        return ret;
    }

    device_info_set_rhs2116(rhs2116_info.chip_id, rhs2116_info.die_revision,
                            rhs2116_info.num_channels);

#ifdef CONFIG_RHS2116_DMA_PROBE
    stream_soak();
#endif
    return 0;
}
