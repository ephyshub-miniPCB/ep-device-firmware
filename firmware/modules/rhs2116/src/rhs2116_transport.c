#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/onoff.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_spim.h>
#include <hal/nrf_timer.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_gpiote.h>

#include <rhs2116/rhs2116_transport.h>

LOG_MODULE_REGISTER(rhs2116_transport, CONFIG_RHS2116_LOG_LEVEL);

#define RHS_NODE  DT_INST(0, intan_rhs2116)
#define SPIM_NODE DT_PHANDLE(RHS_NODE, spim)

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(intan_rhs2116) == 1,
             "the transport owns fixed peripherals, so only one RHS2116 is supported");
BUILD_ASSERT(DT_REG_ADDR(SPIM_NODE) != NRF_SPIM3_BASE,
             "SPIM3 corrupts TX data while the CPU writes its RAM block (anomaly 198)");
BUILD_ASSERT(!DT_NODE_HAS_STATUS(SPIM_NODE, okay),
             "the RHS2116 SPIM node must be disabled so Zephyr's SPI driver leaves it alone");

PINCTRL_DT_DEFINE(SPIM_NODE);

static NRF_SPIM_Type *const spim = (NRF_SPIM_Type *)DT_REG_ADDR(SPIM_NODE);
static NRF_GPIOTE_Type *const gpiote = NRF_GPIOTE;
static const nrfx_gpiote_t gpiote_inst = NRFX_GPIOTE_INSTANCE(0);

// TIMER0/1 (or TIMER4) belong to the BT controller, so pacing and counting use 2 and 3
#define PACE_TIMER  NRF_TIMER2
#define COUNT_TIMER NRF_TIMER3
#define COUNT_IRQn  TIMER3_IRQn
#define COUNT_IRQ_PRIO 1

// TIMER3 compare channels
#define CC_A_DONE  0 // end of block A
#define CC_B_DONE  1 // end of block B, rewind into A
#define CC_TRAP    2 // guard exhausted: PPI stops the pace timer
#define CC_CAPTURE 3 // ISR snapshot of the word count
#define CC_CONFIRM 4 // long slots: one word after a rewind, confirm the switch into A

#define WORD_BYTES 4

// worst case wait for the next END inside the rewind ISR, well over one slot
#define REWIND_SPIN_LIMIT 10000

/* When writes to the ArrayList DMA pointers take effect, measured on the nRF52840
 * (CONFIG_RHS2116_DMA_PROBE and the boot soak test, 2026-10-07):
 *  - a word's transfer takes ~86 ticks from START to END (8 MHz SCLK plus
 *    overhead), STARTED comes 7..42 ticks after START;
 *  - written after a word's STARTED, once the SPIM has stepped the pointers past
 *    it, the pointers apply to the next word, every time;
 *  - written between START and that step, they can be overwritten (TX and RX step
 *    at different times; RX occasionally late, with the radio active).
 * So the rewind writes only once both pointers have visibly stepped past the word in
 * flight, and only with REWIND_MARGIN_TICKS to spare before the next START. */
#define REWIND_MARGIN_TICKS 12

// on slots this long, past this tick the current word's STARTED has certainly come
#define REWIND_LONG_SLOT_TICKS 512
#define REWIND_PAST_STARTED_TICKS 96
/* Long slots leave room for a wider margin before the next START. With a BLE
 * connection active the rewind ISR can enter late in the slot, and the locked check
 * and write was measured taking 44 ticks there, past REWIND_MARGIN_TICKS. */
#define REWIND_LONG_MARGIN_TICKS 128

// TIMER2 channel for reading where in the slot the pace timer is
#define PACE_CC_CAPTURE 2

static uint32_t cs_pin;
static uint8_t cs_ch;
static uint8_t ppi_cs;    // TIMER2 COMPARE1 -> CS low
static uint8_t ppi_start; // TIMER2 COMPARE0 -> SPIM START
static uint8_t ppi_end;   // SPIM END -> CS high, fork TIMER3 COUNT
static uint8_t ppi_trap;  // TIMER3 COMPARE2 -> TIMER2 STOP

// CS falls this many 16 MHz ticks before START (tCS1: CS low >= 20 ns before the
// first SCLK edge, plus GPIOTE latency). The ADC samples on this CS edge.
#define CS_LEAD_TICKS 2

static atomic_t streaming;
static struct rhs2116_ring ring;
static uint32_t slot_ticks_cur;
static rhs2116_block_cb_t block_cb;
static uint32_t cycle_base; // TIMER3 count at the first word of the current A pass
static uint32_t guard_used; // A words clocked from the guard by the last rewind
static uint32_t confirm_g;  // long slots: guard word the pending rewind aimed at A[g]
static bool drift_reported; // hw_pos_in_ring() has warned about this stream
static struct rhs2116_transport_stats stats;

static struct onoff_client hfxo_cli;
static bool hfxo_held;

// single-word idle mode buffers (EasyDMA needs them in RAM)
static uint32_t idle_tx;
static uint32_t idle_rx;

static inline void cs_high(void) {
    nrf_gpiote_task_trigger(gpiote, nrf_gpiote_set_task_get(cs_ch));
}

static inline void cs_low(void) {
    nrf_gpiote_task_trigger(gpiote, nrf_gpiote_clr_task_get(cs_ch));
}

static void set_cycle_compares(uint32_t base) {
    uint32_t b = ring.block_words;

    cycle_base = base;
    nrf_timer_cc_set(COUNT_TIMER, CC_A_DONE, base + b);
    nrf_timer_cc_set(COUNT_TIMER, CC_B_DONE, base + 2 * b);
    nrf_timer_cc_set(COUNT_TIMER, CC_TRAP, base + 2 * b + ring.guard_words);
}

static uint32_t word_count(void) {
    nrf_timer_task_trigger(COUNT_TIMER, nrf_timer_capture_task_get(CC_CAPTURE));
    return nrf_timer_cc_get(COUNT_TIMER, CC_CAPTURE);
}

// 16 MHz ticks since the pace timer last fired START
static inline uint32_t ticks_since_start(void) {
    nrf_timer_task_trigger(PACE_TIMER, nrf_timer_capture_task_get(PACE_CC_CAPTURE));
    return nrf_timer_cc_get(PACE_TIMER, PACE_CC_CAPTURE);
}

/* Ring words the SPIM's DMA pointers are at. In ArrayList mode the hardware steps
 * RXD.PTR/TXD.PTR on by MAXCNT per word, though exactly when relative to START/END
 * isn't documented (CONFIG_RHS2116_DMA_PROBE measures it). */
static inline uint32_t hw_rx_pos(void) {
    return (spim->RXD.PTR - (uint32_t)ring.rx) / WORD_BYTES;
}

static inline uint32_t hw_tx_pos(void) {
    return (spim->TXD.PTR - (uint32_t)ring.tx) / WORD_BYTES;
}

/* EasyDMA writes bypass the MPU, so an RX pointer that has left the ring would
 * silently overwrite whatever follows it in RAM (the BT stack, the kernel). This
 * only fails when a pointer is outside the ring; smaller differences from the word
 * count are reported once per stream (see drift_reported) so they can be studied
 * without stopping the stream. */
static bool hw_pos_in_ring(uint32_t expected) {
    uint32_t rx = hw_rx_pos();
    uint32_t tx = hw_tx_pos();
    uint32_t t = ticks_since_start();
    // words can END while this runs (it may be interrupted): bracket with a 2nd count
    uint32_t again = word_count() - cycle_base;
    uint32_t end = 2 * ring.block_words + ring.guard_words;

    if (rx >= end || tx >= end) {
        nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_STOP);
        stats.pos_faults++;
        LOG_ERR("SPIM DMA left the ring: count says word %u, rx at %u, tx at %u (ring %u)",
                expected, rx, tx, end);
        return false;
    }
    /* Each pointer names the word in flight + 1 (or the next word when idle), and TX
     * steps before RX, so it must lie within [expected, again + 1]. */
    if (!drift_reported && (rx < expected || rx > again + 1 || tx < expected ||
                            tx > again + 1)) {
        drift_reported = true;
        LOG_WRN("SPIM DMA %d/%d words from the word count (count says %u then %u, rx %u, "
                "tx %u, %u ticks into the slot, after rewind %u, guard used %u)",
                (int)(rx - expected), (int)(tx - expected), expected, again, rx, tx, t,
                stats.rewinds, guard_used);
    }
    return true;
}

static void rewind_fault(const char *what, uint32_t g, uint32_t rx, uint32_t tx) {
    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_STOP);
    stats.pos_faults++;
    LOG_ERR("rewind %s: guard word %u, rx at %u, tx at %u (block %u, guard %u)", what, g, rx,
            tx, ring.block_words, ring.guard_words);
}

/* Step 2 of the rewind: once the SPIM has been aimed at A[g], see whether it went
 * there. Returns 0 and moves the compares if it did, -EFAULT (stream stopped) if it
 * is still in the guard, or 1 if it is not yet past A[g]. */
static int rewind_check(uint32_t g) {
    const uint32_t g0 = 2 * ring.block_words;
    uint32_t rx = hw_rx_pos();
    uint32_t tx = hw_tx_pos();

    /* Both past A[g] and still in A: switched. If this was interrupted the SPIM may
     * be some words further on, and TX steps on before RX. */
    if (rx > g && tx > g && rx < g0 && tx < g0) {
        guard_used = g;
        stats.rewinds++;
        stats.max_guard_used = MAX(stats.max_guard_used, g);
        // the guard mirrors A, so this A pass began with the first guard word
        set_cycle_compares(cycle_base + g0);
        return 0;
    }
    // still in the guard past g: the write didn't take
    if (rx >= g0 || tx >= g0) {
        rewind_fault("didn't take", g, rx, tx);
        return -EFAULT;
    }
    return 1;
}

/* Runs just after the last word of B, while the SPIM steps through the guard (a copy
 * of A's head). Points the SPIM at the same place in A, in two steps:
 *
 *   1. Write (see REWIND_MARGIN_TICKS for when that is safe). The next word to be
 *      clocked is guard word g; the pointers are aimed at A[g], the same command, so
 *      the chip sees no seam. If a word's safe window is missed, that word goes out
 *      from the guard and the next one is tried.
 *   2. Confirm. Once that word has started, both pointers must have stepped on inside
 *      A. Only then are the compares moved: until the switch is seen, the trap at the
 *      end of the guard stays armed, so a write that didn't take stops the stream in
 *      hardware before the DMA can leave the ring.
 *
 * Interrupts are locked for at most about one slot on short slots, and only for the
 * check and write on long ones, where step 2 is deferred to the CC_CONFIRM compare
 * rather than spun for. Returns 0 when switched, 1 when the confirmation is pending
 * (long slots), -EOVERFLOW if the guard ran out first, -EFAULT if the pointers
 * weren't where they must be, or -ETIMEDOUT. */
static int rewind_into_a(void) {
    const uint32_t g0 = 2 * ring.block_words;
    const uint32_t limit = k_us_to_cyc_ceil32(4 * slot_ticks_cur / 16 + 1000);
    const uint32_t t0 = k_cycle_get_32();
    const bool long_slot = slot_ticks_cur >= REWIND_LONG_SLOT_TICKS;
    const uint32_t spin_max = 8 * slot_ticks_cur + REWIND_SPIN_LIMIT;
    uint32_t rx = 0;
    uint32_t tx = 0;
    uint32_t g = 0;

    for (;;) {
        if (k_cycle_get_32() - t0 > limit) {
            return -ETIMEDOUT;
        }

        unsigned int key = irq_lock();
        uint32_t next;
        uint32_t t;

        if (long_slot) {
            // the window is long: just check where in the slot we are
            t = ticks_since_start();
            if (t < REWIND_PAST_STARTED_TICKS || t + REWIND_LONG_MARGIN_TICKS > slot_ticks_cur) {
                irq_unlock(key);
                continue;
            }
            // in flight (count not yet stepped) or idle after END: either way the
            // pointers have stepped and name the next word
            rx = hw_rx_pos();
            tx = hw_tx_pos();
            next = rx;
        } else {
            // wait for the next word to start; it is word 'in_flight' of this pass
            nrf_spim_event_clear(spim, NRF_SPIM_EVENT_STARTED);
            uint32_t i = 0;
            while (!nrf_spim_event_check(spim, NRF_SPIM_EVENT_STARTED) && ++i < spin_max) {
            }
            uint32_t in_flight = word_count() - cycle_base;
            next = in_flight + 1;

            // then for both pointers to step past it
            do {
                rx = hw_rx_pos();
                tx = hw_tx_pos();
                t = ticks_since_start();
            } while ((rx != next || tx != next) && t + REWIND_MARGIN_TICKS <= slot_ticks_cur &&
                     ++i < spin_max);
            if (i >= spin_max) {
                irq_unlock(key);
                return -ETIMEDOUT;
            }
            if (rx != next || tx != next) {
                // stepped too late for this word: let it go from the guard
                irq_unlock(key);
                stats.late_rewinds++;
                continue;
            }
        }

        if (rx != tx || next < g0 || rx < g0) {
            irq_unlock(key);
            rewind_fault("found the SPIM off course", next - g0, rx, tx);
            return -EFAULT;
        }
        g = next - g0;
        if (g >= ring.guard_words) {
            irq_unlock(key);
            return -EOVERFLOW;
        }
        spim->TXD.PTR = (uint32_t)&ring.tx[g];
        spim->RXD.PTR = (uint32_t)&ring.rx[g];

        // the next START must not have come before the write
        uint32_t t_after = ticks_since_start();
        irq_unlock(key);
        if (t_after < t) {
            LOG_ERR("rewind write at tick %u ran into the next slot (%u)", t, t_after);
            rewind_fault("missed its window", g, rx, tx);
            return -EFAULT;
        }
        break;
    }

    if (long_slot) {
        /* A[g] starts at the end of this slot, up to a whole slot away (2 ms at
         * 100 S/s). Spinning for it here, at the BT controller's ULL priority,
         * delayed radio events enough to trip its prepare-latency assert. Instead,
         * confirm from the compare one word on, when A[g] has ENDed. Until then the
         * trap stays armed, as in the short-slot path. */
        confirm_g = g;
        nrf_timer_event_clear(COUNT_TIMER, nrf_timer_compare_event_get(CC_CONFIRM));
        nrf_timer_cc_set(COUNT_TIMER, CC_CONFIRM, word_count() + 1);
        nrf_timer_int_enable(COUNT_TIMER, NRF_TIMER_INT_COMPARE4_MASK);
        return 1;
    }

    for (;;) {
        int ret = rewind_check(g);

        if (ret <= 0) {
            return ret;
        }
        if (k_cycle_get_32() - t0 > limit) {
            rewind_fault("not confirmed in time", g, hw_rx_pos(), hw_tx_pos());
            return -ETIMEDOUT;
        }
    }
}

// stop the stream after a failed rewind; the trap compare is still armed, so the
// ring was never overrun
static void rewind_failed(int ret) {
    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_STOP);
    atomic_clear(&streaming);
    if (block_cb) {
        block_cb(1, 0, ret == -EFAULT ? -EFAULT : -EOVERFLOW);
    }
}

static void count_timer_isr(const void *arg) {
    ARG_UNUSED(arg);

    if (nrf_timer_event_check(COUNT_TIMER, NRF_TIMER_EVENT_COMPARE0)) {
        nrf_timer_event_clear(COUNT_TIMER, NRF_TIMER_EVENT_COMPARE0);
        if (atomic_get(&streaming)) {
            uint32_t expected = word_count() - cycle_base;

            if (!hw_pos_in_ring(expected)) {
                atomic_clear(&streaming);
                if (block_cb) {
                    block_cb(0, 0, -EFAULT);
                }
                return;
            }
            if (block_cb) {
                block_cb(0, guard_used, 0);
            }
        }
    }

    if (nrf_timer_event_check(COUNT_TIMER, NRF_TIMER_EVENT_COMPARE1)) {
        nrf_timer_event_clear(COUNT_TIMER, NRF_TIMER_EVENT_COMPARE1);
        if (atomic_get(&streaming)) {
            int ret = rewind_into_a();
            if (ret < 0) {
                rewind_failed(ret);
                return;
            }
            // a pending long-slot rewind reports B from the CC_CONFIRM compare
            if (ret == 0 && block_cb) {
                block_cb(1, 0, 0);
            }
        }
    }

    // only while a rewind is pending: CC_CONFIRM may hold a stale value otherwise
    if (nrf_timer_int_enable_check(COUNT_TIMER, NRF_TIMER_INT_COMPARE4_MASK) &&
        nrf_timer_event_check(COUNT_TIMER, nrf_timer_compare_event_get(CC_CONFIRM))) {
        nrf_timer_event_clear(COUNT_TIMER, nrf_timer_compare_event_get(CC_CONFIRM));
        nrf_timer_int_disable(COUNT_TIMER, NRF_TIMER_INT_COMPARE4_MASK);
        if (atomic_get(&streaming)) {
            // A[g] has ENDed, so a write that took has both pointers past it
            int ret = rewind_check(confirm_g);
            if (ret != 0) {
                if (ret > 0) {
                    rewind_fault("not confirmed after a word", confirm_g, hw_rx_pos(),
                                 hw_tx_pos());
                }
                rewind_failed(-EFAULT);
                return;
            }
            if (block_cb) {
                block_cb(1, 0, 0);
            }
        }
    }

    if (nrf_timer_event_check(COUNT_TIMER, NRF_TIMER_EVENT_COMPARE2)) {
        nrf_timer_event_clear(COUNT_TIMER, NRF_TIMER_EVENT_COMPARE2);
        if (atomic_cas(&streaming, 1, 0) && block_cb) {
            block_cb(1, 0, -EOVERFLOW);
        }
    }
}

static int hfxo_request(void) {
    struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    int res;

    sys_notify_init_spinwait(&hfxo_cli.notify);
    int ret = onoff_request(mgr, &hfxo_cli);
    if (ret < 0) {
        return ret;
    }

    while (sys_notify_fetch_result(&hfxo_cli.notify, &res) == -EAGAIN) {
        k_sleep(K_USEC(100));
    }
    hfxo_held = true;
    return res;
}

static void hfxo_release(void) {
    if (hfxo_held) {
        onoff_release(z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF));
        hfxo_held = false;
    }
}

int rhs2116_transport_init(void) {
    int ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(SPIM_NODE), PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("pinctrl failed: %d", ret);
        return ret;
    }

    // CS: GPIOTE task channel, idling high (inactive)
    cs_pin = NRF_DT_GPIOS_TO_PSEL(RHS_NODE, cs_gpios);
    if (nrfx_gpiote_channel_alloc(&gpiote_inst, &cs_ch) != NRFX_SUCCESS) {
        LOG_ERR("no free GPIOTE channel for CS");
        return -ENOMEM;
    }
    nrf_gpio_pin_set(cs_pin);
    nrf_gpio_cfg_output(cs_pin);
    nrf_gpiote_task_configure(gpiote, cs_ch, cs_pin, NRF_GPIOTE_POLARITY_TOGGLE,
                              NRF_GPIOTE_INITIAL_VALUE_HIGH);
    nrf_gpiote_task_enable(gpiote, cs_ch);

    // SPIM: mode 0 (CPOL=0, CPHA=0), MSB first, 8 MHz (the SPIM2 maximum)
    nrf_spim_disable(spim);
    nrf_spim_int_disable(spim, ~0UL);
    nrf_spim_shorts_disable(spim, ~0UL);
    nrf_spim_configure(spim, NRF_SPIM_MODE_0, NRF_SPIM_BIT_ORDER_MSB_FIRST);
    nrf_spim_frequency_set(spim, NRF_SPIM_FREQ_8M);
    nrf_spim_orc_set(spim, 0);
    nrf_spim_enable(spim);

    // pace timer: 16 MHz, restarts every slot
    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_STOP);
    nrf_timer_mode_set(PACE_TIMER, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(PACE_TIMER, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_prescaler_set(PACE_TIMER, NRF_TIMER_FREQ_16MHz);
    nrf_timer_shorts_set(PACE_TIMER, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);

    // word counter
    nrf_timer_task_trigger(COUNT_TIMER, NRF_TIMER_TASK_STOP);
    nrf_timer_mode_set(COUNT_TIMER, NRF_TIMER_MODE_COUNTER);
    nrf_timer_bit_width_set(COUNT_TIMER, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_shorts_set(COUNT_TIMER, 0);

    IRQ_CONNECT(COUNT_IRQn, COUNT_IRQ_PRIO, count_timer_isr, NULL, 0);
    irq_enable(COUNT_IRQn);

    if (nrfx_gppi_channel_alloc(&ppi_cs) != NRFX_SUCCESS ||
        nrfx_gppi_channel_alloc(&ppi_start) != NRFX_SUCCESS ||
        nrfx_gppi_channel_alloc(&ppi_end) != NRFX_SUCCESS ||
        nrfx_gppi_channel_alloc(&ppi_trap) != NRFX_SUCCESS) {
        LOG_ERR("no free PPI channels");
        return -ENOMEM;
    }

    nrfx_gppi_channel_endpoints_setup(ppi_cs,
        nrf_timer_event_address_get(PACE_TIMER, NRF_TIMER_EVENT_COMPARE1),
        nrf_gpiote_task_address_get(gpiote, nrf_gpiote_clr_task_get(cs_ch)));

    nrfx_gppi_channel_endpoints_setup(ppi_start,
        nrf_timer_event_address_get(PACE_TIMER, NRF_TIMER_EVENT_COMPARE0),
        nrf_spim_task_address_get(spim, NRF_SPIM_TASK_START));

    nrfx_gppi_channel_endpoints_setup(ppi_end,
        nrf_spim_event_address_get(spim, NRF_SPIM_EVENT_END),
        nrf_gpiote_task_address_get(gpiote, nrf_gpiote_set_task_get(cs_ch)));
    nrfx_gppi_fork_endpoint_setup(ppi_end,
        nrf_timer_task_address_get(COUNT_TIMER, NRF_TIMER_TASK_COUNT));

    nrfx_gppi_channel_endpoints_setup(ppi_trap,
        nrf_timer_event_address_get(COUNT_TIMER, NRF_TIMER_EVENT_COMPARE2),
        nrf_timer_task_address_get(PACE_TIMER, NRF_TIMER_TASK_STOP));

    LOG_INF("transport ready: SPIM @ 0x%08x, CS pin %u, GPIOTE ch %u, PPI %u/%u/%u/%u",
            (uint32_t)spim, cs_pin, cs_ch, ppi_cs, ppi_start, ppi_end, ppi_trap);
    return 0;
}

int rhs2116_transport_xfer(uint32_t tx, uint32_t *rx) {
    if (atomic_get(&streaming)) {
        return -EBUSY;
    }

    idle_tx = tx;
    nrf_spim_tx_list_disable(spim);
    nrf_spim_rx_list_disable(spim);
    nrf_spim_tx_buffer_set(spim, (uint8_t *)&idle_tx, WORD_BYTES);
    nrf_spim_rx_buffer_set(spim, (uint8_t *)&idle_rx, WORD_BYTES);
    nrf_spim_event_clear(spim, NRF_SPIM_EVENT_END);

    cs_low();
    k_busy_wait(1);
    nrf_spim_task_trigger(spim, NRF_SPIM_TASK_START);
    while (!nrf_spim_event_check(spim, NRF_SPIM_EVENT_END)) {
    }
    nrf_spim_event_clear(spim, NRF_SPIM_EVENT_END);
    cs_high();

    if (rx) {
        *rx = idle_rx;
    }
    // the GPIO write and the next call's overhead easily cover tCSOFF (100 ns)
    return 0;
}

int rhs2116_transport_stream_start(const struct rhs2116_ring *r, uint32_t slot_ticks,
                                   rhs2116_block_cb_t cb) {
    if (r->guard_words == 0 || r->guard_words > r->block_words) {
        return -EINVAL;
    }
    if (atomic_get(&streaming)) {
        return -EALREADY;
    }

    int ret = hfxo_request();
    if (ret < 0) {
        LOG_ERR("HFXO request failed: %d", ret);
        hfxo_release();
        return ret;
    }

    ring = *r;
    slot_ticks_cur = slot_ticks;
    block_cb = cb;
    drift_reported = false;
    guard_used = 0;
    stats = (struct rhs2116_transport_stats){0};

    nrf_spim_tx_list_enable(spim);
    nrf_spim_rx_list_enable(spim);
    nrf_spim_tx_buffer_set(spim, (uint8_t *)ring.tx, WORD_BYTES);
    nrf_spim_rx_buffer_set(spim, (uint8_t *)ring.rx, WORD_BYTES);
    nrf_spim_event_clear(spim, NRF_SPIM_EVENT_END);
    nrf_spim_event_clear(spim, NRF_SPIM_EVENT_STARTED);

    nrf_timer_task_trigger(COUNT_TIMER, NRF_TIMER_TASK_CLEAR);
    set_cycle_compares(0);
    for (int cc = CC_A_DONE; cc <= CC_CONFIRM; cc++) {
        nrf_timer_event_clear(COUNT_TIMER, nrf_timer_compare_event_get(cc));
    }
    nrf_timer_int_enable(COUNT_TIMER, NRF_TIMER_INT_COMPARE0_MASK |
                                      NRF_TIMER_INT_COMPARE1_MASK |
                                      NRF_TIMER_INT_COMPARE2_MASK);
    nrf_timer_task_trigger(COUNT_TIMER, NRF_TIMER_TASK_START);

    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_CLEAR);
    // from a cleared timer COMPARE1 (CS low) comes before COMPARE0 (START, restart)
    nrf_timer_cc_set(PACE_TIMER, NRF_TIMER_CC_CHANNEL0, slot_ticks);
    nrf_timer_cc_set(PACE_TIMER, NRF_TIMER_CC_CHANNEL1, slot_ticks - CS_LEAD_TICKS);

    atomic_set(&streaming, 1);
    nrfx_gppi_channels_enable(BIT(ppi_cs) | BIT(ppi_start) | BIT(ppi_end) | BIT(ppi_trap));
    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_START);
    return 0;
}

void rhs2116_transport_stream_stop(void) {
    atomic_clear(&streaming);
    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_STOP);

    // let a word already in flight finish (and raise CS) through PPI. If the timer
    // stopped between CS low and START, CS is raised below without a transfer.
    k_busy_wait(10);

    nrfx_gppi_channels_disable(BIT(ppi_cs) | BIT(ppi_start) | BIT(ppi_end) | BIT(ppi_trap));
    nrf_timer_int_disable(COUNT_TIMER, ~0UL);
    nrf_timer_task_trigger(COUNT_TIMER, NRF_TIMER_TASK_STOP);
    nrf_spim_tx_list_disable(spim);
    nrf_spim_rx_list_disable(spim);
    nrf_spim_event_clear(spim, NRF_SPIM_EVENT_END);
    cs_high();

    hfxo_release();
}

#ifdef CONFIG_RHS2116_DMA_PROBE
/*
 * Finds out which moment of writing the ArrayList DMA pointers takes effect cleanly.
 * Runs the real pacing (PPI, TIMER2/3) over a small probe ring with interrupts
 * locked. After the 3rd word has ENDed, the pointers are written to word 20 at one
 * of three moments:
 *   A  right after that END (what the stream's rewind does)
 *   B  right after the next STARTED
 *   C  in the idle gap, 30 ticks before the next START
 * The RX ring is pre-filled with a sentinel, so afterwards the words the SPIM
 * actually used show as written. A clean switch reads "0 1 2 20 21 ..." (A, C) or
 * "0 1 2 3 20 ..." (B), with as many words written as were clocked.
 */
#define PROBE_WORDS     64 // room to spare if a pointer jumps unexpectedly
#define PROBE_SENTINEL  0xA5A5A5A5u
#define PROBE_SWITCH_AT 3  // words ENDed before the write
#define PROBE_TARGET    20
#define PROBE_STOP_AT   9  // words to clock in total

static uint32_t probe_tx[PROBE_WORDS];
static uint32_t probe_rx[PROBE_WORDS];

static void probe_run(uint32_t slot_ticks, char mode) {
    for (int i = 0; i < PROBE_WORDS; i++) {
        probe_tx[i] = i;
        probe_rx[i] = PROBE_SENTINEL;
    }
    nrf_spim_tx_list_enable(spim);
    nrf_spim_rx_list_enable(spim);
    nrf_spim_tx_buffer_set(spim, (uint8_t *)probe_tx, WORD_BYTES);
    nrf_spim_rx_buffer_set(spim, (uint8_t *)probe_rx, WORD_BYTES);
    nrf_spim_event_clear(spim, NRF_SPIM_EVENT_END);
    nrf_spim_event_clear(spim, NRF_SPIM_EVENT_STARTED);

    nrf_timer_int_disable(COUNT_TIMER, ~0UL);
    nrf_timer_task_trigger(COUNT_TIMER, NRF_TIMER_TASK_CLEAR);
    nrf_timer_task_trigger(COUNT_TIMER, NRF_TIMER_TASK_START);
    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_CLEAR);
    nrf_timer_cc_set(PACE_TIMER, NRF_TIMER_CC_CHANNEL0, slot_ticks);
    nrf_timer_cc_set(PACE_TIMER, NRF_TIMER_CC_CHANNEL1, slot_ticks - CS_LEAD_TICKS);

    uint32_t wrote_at_t2 = 0;
    uint32_t count = 0;
    bool armed = false; // the switch word has ENDed (B: waiting for STARTED)
    bool done = false;

    unsigned int key = irq_lock();
    nrfx_gppi_channels_enable(BIT(ppi_cs) | BIT(ppi_start) | BIT(ppi_end));
    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_START);

    for (uint32_t spins = 0; count < PROBE_STOP_AT && spins < 1000000; spins++) {
        count = word_count();
        if (done) {
            continue;
        }
        if (!armed && count == PROBE_SWITCH_AT) {
            armed = true;
            nrf_spim_event_clear(spim, NRF_SPIM_EVENT_STARTED);
            if (mode == 'C') {
                uint32_t t;
                do {
                    nrf_timer_task_trigger(PACE_TIMER,
                                           nrf_timer_capture_task_get(PACE_CC_CAPTURE));
                    t = nrf_timer_cc_get(PACE_TIMER, PACE_CC_CAPTURE);
                } while (t + 30 < slot_ticks && word_count() == PROBE_SWITCH_AT);
            }
            if (mode == 'A' || mode == 'C') {
                nrf_spim_tx_buffer_set(spim, (uint8_t *)&probe_tx[PROBE_TARGET], WORD_BYTES);
                nrf_spim_rx_buffer_set(spim, (uint8_t *)&probe_rx[PROBE_TARGET], WORD_BYTES);
                done = true;
            }
        } else if (armed && mode == 'B' && nrf_spim_event_check(spim, NRF_SPIM_EVENT_STARTED)) {
            nrf_spim_tx_buffer_set(spim, (uint8_t *)&probe_tx[PROBE_TARGET], WORD_BYTES);
            nrf_spim_rx_buffer_set(spim, (uint8_t *)&probe_rx[PROBE_TARGET], WORD_BYTES);
            done = true;
        }
        if (done) {
            nrf_timer_task_trigger(PACE_TIMER, nrf_timer_capture_task_get(PACE_CC_CAPTURE));
            wrote_at_t2 = nrf_timer_cc_get(PACE_TIMER, PACE_CC_CAPTURE);
        }
    }

    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_STOP);
    k_busy_wait(10);
    count = word_count();
    nrfx_gppi_channels_disable(BIT(ppi_cs) | BIT(ppi_start) | BIT(ppi_end));
    nrf_timer_task_trigger(COUNT_TIMER, NRF_TIMER_TASK_STOP);
    uint32_t rx_end = (spim->RXD.PTR - (uint32_t)probe_rx) / WORD_BYTES;
    uint32_t tx_end = (spim->TXD.PTR - (uint32_t)probe_tx) / WORD_BYTES;
    nrf_spim_tx_list_disable(spim);
    nrf_spim_rx_list_disable(spim);
    cs_high();
    irq_unlock(key);

    char list[3 * PROBE_WORDS + 1];
    size_t len = 0;
    uint32_t written = 0;
    for (int i = 0; i < PROBE_WORDS; i++) {
        if (probe_rx[i] != PROBE_SENTINEL) {
            written++;
            len += snprintk(&list[len], sizeof(list) - len, " %d", i);
        }
    }
    list[len] = '\0';
    LOG_INF("probe %c slot %3u: clocked %u, wrote %u rx words:%s | ptr after: rx %u tx %u | "
            "write at t2 %u", mode, slot_ticks, count, written, list, rx_end, tx_end,
            wrote_at_t2);
    k_msleep(20); // let deferred logging drain
}

/* Exact pace-timer tick of each word's STARTED and END over 40 paced words: PPI
 * captures TIMER2 on the events, so there is no polling latency. END is also when
 * CS goes high, and CS falls again CS_LEAD_TICKS before the next START. */
static void probe_timing(uint32_t slot_ticks) {
    uint8_t ch_started, ch_end;
    uint32_t s_lo = UINT32_MAX, s_hi = 0, e_lo = UINT32_MAX, e_hi = 0;

    if (nrfx_gppi_channel_alloc(&ch_started) != NRFX_SUCCESS ||
        nrfx_gppi_channel_alloc(&ch_end) != NRFX_SUCCESS) {
        LOG_ERR("timing probe: no PPI channels");
        return;
    }
    nrfx_gppi_channel_endpoints_setup(ch_started,
        nrf_spim_event_address_get(spim, NRF_SPIM_EVENT_STARTED),
        nrf_timer_task_address_get(PACE_TIMER, NRF_TIMER_TASK_CAPTURE3));
    nrfx_gppi_channel_endpoints_setup(ch_end,
        nrf_spim_event_address_get(spim, NRF_SPIM_EVENT_END),
        nrf_timer_task_address_get(PACE_TIMER, NRF_TIMER_TASK_CAPTURE2));

    nrf_spim_tx_list_disable(spim);
    nrf_spim_rx_list_disable(spim);
    nrf_spim_tx_buffer_set(spim, (uint8_t *)probe_tx, WORD_BYTES);
    nrf_spim_rx_buffer_set(spim, (uint8_t *)probe_rx, WORD_BYTES);
    nrf_timer_int_disable(COUNT_TIMER, ~0UL);
    nrf_timer_task_trigger(COUNT_TIMER, NRF_TIMER_TASK_CLEAR);
    nrf_timer_task_trigger(COUNT_TIMER, NRF_TIMER_TASK_START);
    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_CLEAR);
    nrf_timer_cc_set(PACE_TIMER, NRF_TIMER_CC_CHANNEL0, slot_ticks);
    nrf_timer_cc_set(PACE_TIMER, NRF_TIMER_CC_CHANNEL1, slot_ticks - CS_LEAD_TICKS);

    unsigned int key = irq_lock();
    nrfx_gppi_channels_enable(BIT(ppi_cs) | BIT(ppi_start) | BIT(ppi_end) | BIT(ch_started) |
                              BIT(ch_end));
    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_START);

    uint32_t seen = 0;
    for (uint32_t spins = 0; seen < 40 && spins < 2000000; spins++) {
        uint32_t c = word_count();
        if (c != seen) {
            // the word just ENDed: both captures belong to it (the next START is later)
            seen = c;
            uint32_t ts = nrf_timer_cc_get(PACE_TIMER, NRF_TIMER_CC_CHANNEL3);
            uint32_t te = nrf_timer_cc_get(PACE_TIMER, NRF_TIMER_CC_CHANNEL2);
            if (c > 1) { // the first word's START has no previous slot
                s_lo = MIN(s_lo, ts);
                s_hi = MAX(s_hi, ts);
                e_lo = MIN(e_lo, te);
                e_hi = MAX(e_hi, te);
            }
        }
    }

    nrf_timer_task_trigger(PACE_TIMER, NRF_TIMER_TASK_STOP);
    k_busy_wait(10);
    nrfx_gppi_channels_disable(BIT(ppi_cs) | BIT(ppi_start) | BIT(ppi_end) | BIT(ch_started) |
                               BIT(ch_end));
    nrf_timer_task_trigger(COUNT_TIMER, NRF_TIMER_TASK_STOP);
    cs_high();
    irq_unlock(key);
    nrfx_gppi_channel_free(ch_started);
    nrfx_gppi_channel_free(ch_end);

    LOG_INF("timing slot %3u: STARTED at %u..%u, END at %u..%u ticks after START; "
            "CS high for %d..%d ticks between words", slot_ticks, s_lo, s_hi, e_lo, e_hi,
            (int)(slot_ticks - CS_LEAD_TICKS) - (int)e_hi,
            (int)(slot_ticks - CS_LEAD_TICKS) - (int)e_lo);
    k_msleep(20);
}

void rhs2116_transport_dma_probe(uint32_t slot_ticks) {
    probe_timing(slot_ticks);
    probe_run(slot_ticks, 'A');
    probe_run(slot_ticks, 'B');
    probe_run(slot_ticks, 'C');
}
#endif /* CONFIG_RHS2116_DMA_PROBE */

void rhs2116_transport_get_stats(struct rhs2116_transport_stats *out) {
    *out = stats;
}
