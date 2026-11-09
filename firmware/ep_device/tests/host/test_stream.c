/*
 * Host test for the RHS2116 stream and stimulation scheduling.
 *
 * rhs2116_stream.c (from the rhs2116 module, firmware/modules/rhs2116) is
 * compiled in (to reach its event handler) against Zephyr
 * stubs, with the transport replaced by a word-level model of the SPIM ring
 * ([A | B | guard], with a random rewind latency into the guard every cycle) and
 * the chip by a model of the RHS2116's 2-word response pipeline and triggered
 * registers. stim.c and rhs2116_ctrl.c are linked in unchanged.
 *
 *   make -C tests/host
 */
#include <assert.h>
#include <stdlib.h>

#include "../../../modules/rhs2116/src/rhs2116_stream.c"

#include "recording.h"
#include "rhs2116_priv.h"
#include "stim.h"

static int failures;

#define CHECK(cond, ...)                                                              \
    do {                                                                              \
        if (!(cond)) {                                                                \
            failures++;                                                               \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                             \
            printf(__VA_ARGS__);                                                      \
            printf("\n");                                                             \
        }                                                                             \
    } while (0)

// -- CHIP MODEL --

static uint16_t chip_buf[256];   // written values (triggered registers' buffers)
static uint16_t chip_act[256];   // active values
static uint32_t chip_pipe[2];
static uint32_t chip_conv[64];   // conversions per channel
static uint16_t chip_compliance;
static uint32_t chip_words;

// state of the stim registers after each U flag, one entry per frame while streaming
struct u_snapshot {
    uint32_t word;
    uint16_t on, pol, recovery;
};
static struct u_snapshot u_log[200000];
static uint32_t n_u;

static bool triggered(uint8_t reg) {
    return reg == 10 || reg == 12 || reg == 42 || reg == 44 || reg == 46 || reg == 48 ||
           (reg >= 64 && reg <= 79) || (reg >= 96 && reg <= 111);
}

static uint16_t sample_value(uint8_t ch, uint32_t n) {
    return (uint16_t)(ch * 4096u + n * 7u);
}

static uint32_t chip_compute(uint32_t cmd) {
    uint32_t r;
    uint8_t reg = (cmd >> 16) & 0xFF;

    switch (cmd >> 30) {
    case 0: { // CONVERT
        uint8_t ch = (cmd >> 16) & 0x3F;
        r = (uint32_t)sample_value(ch, chip_conv[ch]++) << 16;
        break;
    }
    case 2: // WRITE
        chip_buf[reg] = (uint16_t)cmd;
        if (!triggered(reg)) {
            chip_act[reg] = (uint16_t)cmd;
        }
        r = 0xFFFF0000u | (uint16_t)cmd;
        break;
    case 3: // READ
        r = reg == 255 ? 0x20 : reg == 40 ? chip_compliance : chip_buf[reg];
        break;
    default:
        r = 0x80000000u;
        break;
    }

    if (cmd & BIT(28)) {
        chip_compliance = 0;
    }
    if (cmd & BIT(29)) {
        for (int i = 0; i < 256; i++) {
            if (triggered(i)) {
                chip_act[i] = chip_buf[i];
            }
        }
        if (n_u < ARRAY_SIZE(u_log)) {
            u_log[n_u++] = (struct u_snapshot){ chip_words, chip_act[42], chip_act[44], chip_act[46] };
        }
    }
    chip_words++;
    return r;
}

static uint32_t chip_xfer(uint32_t cmd) {
    uint32_t out = chip_pipe[0];
    chip_pipe[0] = chip_pipe[1];
    chip_pipe[1] = chip_compute(cmd);
    return out;
}

static void chip_reset(void) {
    memset(chip_buf, 0, sizeof(chip_buf));
    memset(chip_act, 0, sizeof(chip_act));
    memset(chip_pipe, 0, sizeof(chip_pipe));
    memset(chip_conv, 0, sizeof(chip_conv));
    chip_compliance = 0;
    chip_words = 0;
    n_u = 0;
}

// -- TRANSPORT MODEL --

static struct rhs2116_ring hw_ring;
static rhs2116_block_cb_t hw_cb;
static bool hw_running;
static uint32_t hw_pos, hw_count, hw_base, hw_guard_used;
static uint32_t hw_max_guard; // largest rewind latency to simulate (words)

int rhs2116_transport_init(void) { return 0; }
int rhs2116_transport_xfer(uint32_t tx, uint32_t *rx) { (void)tx; (void)rx; return -EBUSY; }
void rhs2116_transport_get_stats(struct rhs2116_transport_stats *s) { memset(s, 0, sizeof(*s)); }
int rhs2116_set_adc_rate(const struct device *dev, uint32_t r) { (void)dev; (void)r; return 0; }

int rhs2116_transport_stream_start(const struct rhs2116_ring *r, uint32_t slot_ticks,
                                   rhs2116_block_cb_t cb) {
    (void)slot_ticks;
    hw_ring = *r;
    hw_cb = cb;
    hw_running = true;
    hw_pos = hw_count = hw_base = hw_guard_used = 0;
    return 0;
}

void rhs2116_transport_stream_stop(void) {
    hw_running = false;
}

static void run_thread(void) {
    struct blk_evt e;
    while (k_msgq_get(&evt_q, &e, K_NO_WAIT) == 0) {
        handle_event(&e);
    }
}

static void hw_word(void) {
    uint32_t cmd = rhs2116_unwire(hw_ring.tx[hw_pos]);
    hw_ring.rx[hw_pos] = rhs2116_wire(chip_xfer(cmd));
    hw_pos++;
    hw_count++;
}

static void hw_run(uint32_t words) {
    uint32_t b = hw_ring.block_words;

    for (uint32_t i = 0; i < words && hw_running; i++) {
        hw_word();
        if (hw_count == hw_base + b) {
            hw_cb(0, hw_guard_used, 0);
            run_thread();
        } else if (hw_count == hw_base + 2 * b) {
            // rewind ISR latency: at least one guard word is clocked first
            uint32_t p = 1 + (uint32_t)rand() % MAX(hw_max_guard, 1);
            if (p >= hw_ring.guard_words) {
                for (uint32_t k = 0; k < hw_ring.guard_words; k++) {
                    hw_word();
                }
                hw_running = false; // the trap compare stops the pace timer
                hw_cb(1, 0, -EOVERFLOW);
                run_thread();
                return;
            }
            for (uint32_t k = 0; k < p; k++) {
                hw_word();
            }
            i += p;
            hw_pos = p;
            hw_guard_used = p;
            hw_base += 2 * b;
            hw_cb(1, 0, 0);
            run_thread();
        }
    }
}

// -- SINK --

static uint32_t next_frame;
static uint32_t frames_seen;
static uint32_t sample_errors;
static uint32_t spi_errors;
static int stopped_err;

static void on_frame(uint32_t frame_idx, const uint16_t *samples, uint8_t n_ch) {
    if (frame_idx != next_frame) {
        sample_errors++;
    }
    next_frame = frame_idx + 1;
    frames_seen++;
    for (uint8_t k = 0; k < n_ch; k++) {
        if (samples[k] != sample_value(layout.chans[k], frame_idx)) {
            if (sample_errors++ < 5) {
                printf("  frame %u slot %u: got 0x%04x want 0x%04x\n", frame_idx, k,
                       samples[k], sample_value(layout.chans[k], frame_idx));
            }
        }
    }
}

static rec_fault_hook_t fault_hook_fn;

static void on_status(enum rhs2116_stream_status status, uint32_t frame, uint32_t value) {
    if (status == RHS2116_STREAM_STATUS_SPI_ERROR) {
        if (spi_errors++ < 5) {
            printf("  spi error at frame %u: 0x%08x\n", frame, value);
        }
    } else if (status == RHS2116_STREAM_STATUS_STOPPED) {
        stopped_err = (int)value;
    }
    if (fault_hook_fn) {
        fault_hook_fn(status, frame, value);
    }
}

static const struct rhs2116_stream_sink test_sink = { on_frame, on_status };

// -- STUBS FOR stim.c --

static struct rhs2116_data dev_data;
static struct device dev = { .data = &dev_data };
static uint32_t n_rec_events;

bool recording_active(void) { return rhs2116_stream_active(); }
void recording_set_stim_active(bool a) { (void)a; }
void recording_set_fault_hook(rec_fault_hook_t hook) { fault_hook_fn = hook; }
void recording_event(enum rec_event_code code, uint32_t frame, uint16_t a, uint16_t b) {
    (void)code; (void)frame; (void)a; (void)b;
    n_rec_events++;
}

// setup writes from stim_start: applied straight to the chip model (the real
// ones go through a misc slot, which needs another thread to wait on)
int rhs2116_write_reg(const struct device *d, uint8_t reg, uint16_t data, bool update, bool clear) {
    (void)d;
    (void)chip_compute(rhs2116_cmd_write(reg, data, update, clear));
    return 0;
}

int rhs2116_read_reg(const struct device *d, uint8_t reg, bool update, bool clear, uint32_t *out) {
    (void)d;
    *out = chip_compute(rhs2116_cmd_read(reg, update, clear));
    return 0;
}

extern int (*const stim_init_sys_init)(void);

// -- HELPERS --

static int start(uint16_t mask, uint32_t rate, uint32_t max_guard) {
    enum rhs2116_stream_reject why;
    chip_reset();
    next_frame = frames_seen = sample_errors = spi_errors = 0;
    stopped_err = 0;
    hw_max_guard = max_guard;
    return rhs2116_stream_start(&dev, mask, rate, &test_sink, NULL, &why);
}

static uint32_t frames_to_words(uint32_t frames) {
    return frames * layout.slots_per_frame;
}

// -- TESTS --

static void test_plan(void) {
    printf("plan limits\n");
    struct rhs2116_stream_layout l;
    enum rhs2116_stream_reject why;

    CHECK(rhs2116_stream_plan(0x0001, 30000, &l, &why) == 0, "1 ch @ 30k should fit");
    CHECK(l.slots_per_frame == 5 && l.slot_ticks == 106, "1 ch layout %u x %u",
          l.slots_per_frame, l.slot_ticks);
    CHECK(rhs2116_stream_plan(0x0003, 30000, &l, &why) == 0, "2 ch @ 30k should fit");
    CHECK(rhs2116_stream_plan(0xFFFF, 30000, &l, &why) < 0 && why == RHS2116_STREAM_RATE_TOO_HIGH,
          "16 ch @ 30k must exceed the SPI budget");
    CHECK(rhs2116_stream_plan(0xFFFF, rhs2116_stream_max_rate_hz(16), &l, &why) == 0,
          "16 ch at its max rate should fit");
    CHECK(rhs2116_stream_plan(0x0000, 1000, &l, &why) < 0 && why == RHS2116_STREAM_BAD_MASK,
          "empty mask");
    CHECK(rhs2116_stream_plan(0x0001, 10, &l, &why) < 0 && why == RHS2116_STREAM_RATE_TOO_LOW,
          "rate too low");
    for (uint8_t n = 1; n <= 16; n++) {
        uint32_t mask = (uint32_t)BIT(n) - 1;
        CHECK(rhs2116_stream_plan((uint16_t)mask, rhs2116_stream_max_rate_hz(n), &l, &why) == 0,
              "%u ch at max rate", n);
        CHECK(l.frames_per_block * l.slots_per_frame <= MAX_BLOCK_WORDS &&
              l.guard_frames * l.slots_per_frame <= MAX_GUARD_WORDS &&
              l.guard_frames <= l.frames_per_block, "%u ch ring sizes", n);
    }
}

static void test_streaming(uint16_t mask, uint32_t rate, uint32_t seconds) {
    printf("stream mask 0x%04x @ %u S/s for %u s\n", mask, rate, seconds);
    CHECK(start(mask, rate, 0) == 0, "start");
    hw_max_guard = guard_words - 1; // any latency short of overflowing the guard

    uint32_t frames = (layout.rate_mhz / 1000) * seconds;
    hw_run(frames_to_words(frames));

    CHECK(stopped_err == 0, "stream stopped itself: %d", stopped_err);
    CHECK(sample_errors == 0, "%u sample errors", sample_errors);
    CHECK(spi_errors == 0, "%u spi errors", spi_errors);
    // the last block or two are still unprocessed when the simulation stops
    uint32_t pending = 2 * layout.frames_per_block + layout.guard_frames;
    CHECK(frames_seen + pending >= frames && frames_seen <= frames, "frames %u of %u",
          frames_seen, frames);
    CHECK(stats.compliance_events == 0, "compliance events");
    rhs2116_stream_stop();
}

static void test_guard_overflow(void) {
    printf("guard overflow stops the stream\n");
    CHECK(start(0x0001, 30000, 0) == 0, "start");
    hw_max_guard = 100000; // always miss the guard
    hw_run(frames_to_words(layout.frames_per_block * 4));
    CHECK(stopped_err == -EOVERFLOW, "expected -EOVERFLOW, got %d", stopped_err);
    CHECK(!rhs2116_stream_active(), "stream should be stopped");
}

static void test_host_access(void) {
    printf("register access through a misc slot\n");
    CHECK(start(0x0005, 20000, 2) == 0, "start");
    hw_run(frames_to_words(layout.frames_per_block * 3));

    // what rhs2116_stream_reg_access does, minus the blocking wait
    req.gen++;
    req.write = false;
    req.cmd = rhs2116_cmd_read(255, false, false);
    req.state = REQ_PENDING;
    hw_run(frames_to_words(layout.frames_per_block * 3));
    CHECK(req.state == REQ_DONE && req.err == 0 && req.result == 0x20,
          "read: state %d err %d result 0x%08x", req.state, req.err, req.result);

    req.gen++;
    req.write = true;
    req.data = 0x1234;
    req.cmd = rhs2116_cmd_write(3, 0x1234, false, false);
    req.state = REQ_PENDING;
    hw_run(frames_to_words(layout.frames_per_block * 3));
    CHECK(req.state == REQ_DONE && req.err == 0 && chip_act[3] == 0x1234,
          "write: state %d err %d reg 0x%04x", req.state, req.err, chip_act[3]);

    CHECK(sample_errors == 0 && spi_errors == 0, "errors %u/%u", sample_errors, spi_errors);
    rhs2116_stream_stop();
}

// collapses the per-frame U log into runs of identical stim state
struct run {
    uint16_t on, pol, recovery;
    uint32_t frames;
};

static uint32_t stim_runs(struct run *runs, uint32_t max) {
    uint32_t n = 0;
    for (uint32_t i = 0; i < n_u; i++) {
        struct u_snapshot *s = &u_log[i];
        if (n && runs[n - 1].on == s->on && runs[n - 1].pol == s->pol &&
            runs[n - 1].recovery == s->recovery) {
            runs[n - 1].frames++;
        } else if (n < max) {
            runs[n++] = (struct run){ s->on, s->pol, s->recovery, 1 };
        }
    }
    return n;
}

static void test_biphasic(void) {
    printf("biphasic pulse train timing\n");
    CHECK(start(0x0001, 30000, 4) == 0, "start");
    hw_run(frames_to_words(10));

    // 30.19 kS/s: 100 us = 3 frames, 33 us = 1, 200 us = 6, 1 ms = 30
    struct stim_biphasic b = {
        .chan_mask = 0x0006, .neg_mag = 20, .pos_mag = 20, .flags = 0,
        .phase1_us = 100, .interphase_us = 33, .phase2_us = 100, .recovery_us = 200,
        .period_us = 1000, .count = 3,
    };
    struct stim_biphasic_frames f;
    enum stim_reject why;
    CHECK(stim_load_biphasic(&b, &f, &why) == 0, "load: reject %d", why);
    CHECK(f.phase1 == 3 && f.interphase == 1 && f.phase2 == 3 && f.recovery == 6 && f.period == 30,
          "frames %u/%u/%u/%u/%u", f.phase1, f.interphase, f.phase2, f.recovery, f.period);

    CHECK(stim_start(&dev, &why) == 0, "start: reject %d", why);
    CHECK(chip_act[32] == 0xAAAA && chip_act[33] == 0x00FF, "stimulators enabled");
    CHECK(chip_buf[65] == 0x8014 && chip_buf[97] == 0x8014, "amplitudes");
    // the program starts at the first frame scheduled after stim_start, up to two
    // blocks ahead of what is being clocked out
    hw_run(frames_to_words(2 * layout.frames_per_block + 200));

    struct run runs[64];
    uint32_t n = stim_runs(runs, ARRAY_SIZE(runs));
    if (getenv("VERBOSE")) {
        printf("  %u U entries, %u runs\n", n_u, n);
        for (uint32_t i = 0; i < n; i++) {
            printf("  run %u: on 0x%x pol 0x%x rec 0x%x x%u\n", i, runs[i].on, runs[i].pol,
                   runs[i].recovery, runs[i].frames);
        }
    }

    // expected per pulse: cathodic 3, off 1, anodic 3, recovery 6, idle to period
    const struct run want[] = {
        { 0x6, 0x0, 0x0, 3 },
        { 0x0, 0x0, 0x0, 1 },
        { 0x6, 0x6, 0x0, 3 },
        { 0x0, 0x0, 0x6, 6 },
        { 0x0, 0x0, 0x0, 30 - 13 },
    };
    uint32_t r = 1; // runs[0] is the idle time before the first pulse
    for (int pulse = 0; pulse < 3; pulse++) {
        for (size_t k = 0; k < ARRAY_SIZE(want); k++, r++) {
            if (pulse == 2 && k == ARRAY_SIZE(want) - 1) {
                break; // idle after the last pulse runs to the end of the log
            }
            CHECK(r < n && runs[r].on == want[k].on && runs[r].pol == want[k].pol &&
                  runs[r].recovery == want[k].recovery && runs[r].frames == want[k].frames,
                  "pulse %d step %zu: got on 0x%x pol 0x%x rec 0x%x x%u", pulse, k,
                  r < n ? runs[r].on : 0, r < n ? runs[r].pol : 0,
                  r < n ? runs[r].recovery : 0, r < n ? runs[r].frames : 0);
        }
    }
    CHECK(!stim_running(), "program should have finished");
    CHECK(chip_act[32] == 0 && chip_act[33] == 0, "stimulators disabled after the train");
    CHECK(sample_errors == 0 && spi_errors == 0, "errors %u/%u", sample_errors, spi_errors);
    rhs2116_stream_stop();
}

static void test_abort(void) {
    printf("abort mid pulse train\n");
    CHECK(start(0x0001, 30000, 4) == 0, "start");
    hw_run(frames_to_words(10));

    struct stim_biphasic b = {
        .chan_mask = 0x0001, .neg_mag = 50, .pos_mag = 50,
        .phase1_us = 200, .interphase_us = 0, .phase2_us = 200, .recovery_us = 300,
        .period_us = 2000, .count = 0,
    };
    struct stim_biphasic_frames f;
    enum stim_reject why;
    CHECK(stim_load_biphasic(&b, &f, &why) == 0, "load: reject %d", why);
    CHECK(stim_start(&dev, &why) == 0, "start");
    hw_run(frames_to_words(500));
    CHECK(stim_running(), "continuous train should still run");

    uint32_t u_before = n_u;
    stim_stop(); // can't block here: the shutdown runs as the stream advances
    hw_run(frames_to_words(layout.frames_per_block * 3 + f.recovery + 10));
    CHECK(!stim_running(), "should be idle after abort");
    CHECK(chip_act[42] == 0 && chip_act[46] == 0 && chip_act[32] == 0 && chip_act[33] == 0,
          "after abort: on 0x%x rec 0x%x en 0x%x/0x%x", chip_act[42], chip_act[46],
          chip_act[32], chip_act[33]);

    // once stimulators go off for the abort, the recovery switch closes for exactly
    // recovery frames and nothing turns on again
    bool saw_recovery = false;
    for (uint32_t i = u_before; i < n_u; i++) {
        if (u_log[i].recovery) {
            saw_recovery = true;
        }
    }
    CHECK(saw_recovery, "recovery switch closed during abort");
    CHECK(u_log[n_u - 1].on == 0, "off at end");
    rhs2116_stream_stop();
}

static void test_compliance_abort(void) {
    printf("compliance fault aborts stimulation\n");
    CHECK(start(0x0001, 30000, 4) == 0, "start");
    hw_run(frames_to_words(10));

    struct stim_biphasic b = {
        .chan_mask = 0x0001, .neg_mag = 50, .pos_mag = 50,
        .phase1_us = 200, .phase2_us = 200, .recovery_us = 100, .period_us = 1000,
    };
    struct stim_biphasic_frames f;
    enum stim_reject why;
    CHECK(stim_load_biphasic(&b, &f, &why) == 0, "load");
    CHECK(stim_start(&dev, &why) == 0, "start");
    hw_run(frames_to_words(300));
    chip_compliance = 0x0001;
    hw_run(frames_to_words(layout.frames_per_block * 4 + f.recovery + 10));
    CHECK(stats.compliance_events >= 1, "compliance seen");
    CHECK(!stim_running() && chip_act[42] == 0 && chip_act[32] == 0, "aborted and disabled");
    rhs2116_stream_stop();
}

static void test_validation(void) {
    printf("stim program validation\n");
    CHECK(start(0x0001, 30000, 4) == 0, "start");
    struct stim_biphasic_frames f;
    enum stim_reject why;

    struct stim_biphasic b = {
        .chan_mask = 0x0001, .neg_mag = 10, .pos_mag = 10,
        .phase1_us = 100, .phase2_us = 200, .period_us = 1000, .count = 1,
    };
    CHECK(stim_load_biphasic(&b, &f, &why) < 0 && why == STIM_UNBALANCED, "unequal widths");
    b.pos_mag = 5;
    CHECK(stim_load_biphasic(&b, &f, &why) == 0, "equal charge with half amplitude: %d", why);
    b.phase1_us = 10;
    CHECK(stim_load_biphasic(&b, &f, &why) < 0 && why == STIM_TOO_SHORT, "sub-frame phase");

    struct stim_program p = {
        .chan_mask = 0x0001, .neg_mag = 10, .pos_mag = 10, .period_frames = 100, .count = 1,
        .n_events = 2,
        .events = { { .frame = 0, .on = 1 }, { .frame = 5, .on = 1, .pol = 1 } },
    };
    CHECK(stim_load_program(&p, &why) < 0 && why == STIM_BAD_EVENTS, "must end off");
    p.events[1] = (struct stim_event){ .frame = 5, .on = 0 };
    p.flags = STIM_FLAG_ALLOW_IMBALANCE;
    CHECK(stim_load_program(&p, &why) == 0, "monophasic allowed with override: %d", why);
    p.events[0].on = 0x0002;
    CHECK(stim_load_program(&p, &why) < 0 && why == STIM_BAD_MASK, "channel outside mask");
    p.events[0] = (struct stim_event){ .frame = 0, .on = 1, .n_extra = 1,
                                       .extra = { { .reg = 32, .val = 0 } } };
    CHECK(stim_load_program(&p, &why) < 0 && why == STIM_BAD_EVENTS, "forbidden register");
    rhs2116_stream_stop();
    CHECK(stim_load_program(&p, &why) < 0 && why == STIM_NOT_RECORDING, "needs recording");
}

static void test_ctrl(void) {
    printf("task-level API (idle)\n");
    chip_reset();
    bool armed;

    CHECK(rhs2116_stim_set_step(&dev, RHS2116_STIM_STEP_10NA) == 0, "set step");
    CHECK(chip_act[34] == rhs2116_enc_stim_step(64, 19, 3) && chip_act[35] == 0x66,
          "step regs 0x%04x/0x%04x", chip_act[34], chip_act[35]);
    CHECK(rhs2116_stim_get_step(&dev) == RHS2116_STIM_STEP_10NA &&
          rhs2116_stim_step_na(RHS2116_STIM_STEP_10UA) == 10000, "step bookkeeping");
    CHECK(rhs2116_stim_set_step(&dev, (enum rhs2116_stim_step)99) == -EINVAL, "bad step");

    CHECK(rhs2116_stim_set_trim(&dev, 0, -10, 5) == 0, "trim");
    CHECK(rhs2116_stim_set_trim(&dev, 16, 0, 0) == -EINVAL, "trim bad channel");
    CHECK(rhs2116_stim_set_amplitude(&dev, 0x0003, 20, 30) == 0, "amplitude");
    CHECK(chip_act[64] == ((0x76 << 8) | 20) && chip_act[96] == ((0x85 << 8) | 30) &&
          chip_act[65] == ((0x80 << 8) | 20) && chip_act[66] == 0,
          "magnitudes 0x%04x 0x%04x 0x%04x", chip_act[64], chip_act[96], chip_act[65]);

    // arming: refuses with a stimulator on, then step changes are refused while armed
    chip_compliance = 0x0004;
    (void)chip_compute(rhs2116_cmd_write(42, 0x0001, true, false));
    CHECK(rhs2116_stim_arm(&dev) == -EBUSY && chip_act[32] == 0, "arm with stim on");
    (void)chip_compute(rhs2116_cmd_write(42, 0, true, false));
    CHECK(rhs2116_stim_arm(&dev) == 0 && chip_act[32] == 0xAAAA && chip_act[33] == 0x00FF &&
          chip_compliance == 0, "arm");
    CHECK(rhs2116_stim_is_armed(&dev, &armed) == 0 && armed, "armed");
    CHECK(rhs2116_stim_set_step(&dev, RHS2116_STIM_STEP_1UA) == -EBUSY, "step while armed");

    struct rhs2116_stim_state st = { .on = 0x0001, .pol = 0x0001, .recovery = 0x0002 };
    CHECK(rhs2116_stim_output(&dev, &st) == 0 && chip_act[42] == 1 && chip_act[44] == 1 &&
          chip_act[46] == 2, "output");
    CHECK(rhs2116_stim_disarm(&dev) == 0 && chip_act[42] == 0 && chip_act[46] == 0 &&
          chip_act[48] == 0 && chip_act[32] == 0 && chip_act[33] == 0, "disarm");
    CHECK(rhs2116_stim_is_armed(&dev, &armed) == 0 && !armed, "disarmed");
    CHECK(rhs2116_stim_output(&dev, &st) == -EPERM && chip_act[42] == 0, "output while disarmed");

    // bandwidth snaps to the nearest table entry on a log scale
    struct rhs2116_bandwidth bw = { 8000, 4000, 1000000 }, got;
    CHECK(rhs2116_set_bandwidth(&dev, &bw, &got) == 0 && got.upper_hz == 7500 &&
          got.lower_mhz == 5000 && got.lower_alt_mhz == 1000000,
          "bandwidth %u/%u/%u", got.upper_hz, got.lower_mhz, got.lower_alt_mhz);
    CHECK(chip_act[4] == 0x16 && chip_act[5] == 0x17 && chip_act[6] == 0xA8 && chip_act[7] == 0x0A,
          "bandwidth regs");
    bw = (struct rhs2116_bandwidth){ 1, 50, 1 };
    CHECK(rhs2116_set_bandwidth(&dev, &bw, &got) == 0 && got.upper_hz == 100 &&
          got.lower_mhz == 100 && got.lower_alt_mhz == 100, "bandwidth clamps to the table");
    bw = (struct rhs2116_bandwidth){ 100, 500000, 1000 };
    CHECK(rhs2116_set_bandwidth(&dev, &bw, NULL) == -EINVAL, "lower above upper");

    CHECK(rhs2116_stim_set_cl_recovery(&dev, 0, RHS2116_CL_ILIM_1NA) == 0 &&
          chip_act[36] == 128 && chip_act[37] == 0x4F00, "cl recovery 0 V / 1 nA");
    CHECK(rhs2116_stim_set_cl_recovery(&dev, -1225, RHS2116_CL_ILIM_1UA) == 0 &&
          chip_act[36] == 0 && chip_act[37] == 9, "cl recovery min");
    CHECK(rhs2116_stim_set_cl_recovery(&dev, 1215, RHS2116_CL_ILIM_1UA) == 0 &&
          chip_act[36] == 255, "cl recovery max");
    CHECK(rhs2116_stim_set_cl_recovery(&dev, 1300, RHS2116_CL_ILIM_1UA) == -EINVAL,
          "cl recovery out of range");
}

int main(void) {
    srand(1);
    req_init();
    stim_init_sys_init();

    test_plan();
    test_streaming(0x0001, 30000, 3);
    test_streaming(0x0003, 30000, 2);
    test_streaming(0x8421, 20000, 2);
    test_streaming(0xFFFF, rhs2116_stream_max_rate_hz(16), 2);
    test_streaming(0x0001, 100, 5);
    test_guard_overflow();
    test_host_access();
    test_validation();
    test_biphasic();
    test_abort();
    test_compliance_abort();
    test_ctrl();

    printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
    return failures ? 1 : 0;
}
