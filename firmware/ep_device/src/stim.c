#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <rhs2116/rhs2116.h>
#include <rhs2116/rhs2116_stream.h>

#include "recording.h"
#include "stim.h"

LOG_MODULE_REGISTER(stim, CONFIG_LOG_DEFAULT_LEVEL);

#define STOP_TIMEOUT K_SECONDS(1)

enum run_state {
    ST_IDLE,
    ST_ARMED,     // starts at the next frame the stream schedules
    ST_RUNNING,
    ST_SHUTDOWN,  // stepping through the safe stop
};

enum shutdown_step {
    SD_OFF,       // stimulators off, recovery switch closed
    SD_RECOVERY,  // waiting out the recovery time
    SD_DISABLE,   // disable the stimulators, then idle
};

static struct k_spinlock lock;
static struct stim_program prog;
static bool loaded;
static uint32_t prog_rate_mhz;

// run state, shared by the command thread and the stream thread (provider)
static enum run_state state;
static uint32_t pulse_start;
static uint16_t pulses;
static uint8_t next_ev;
static uint16_t cur_on;
static uint16_t cur_pol;
static enum shutdown_step sd_step;
static uint32_t sd_until;
static uint8_t abort_reason;

static const struct device *stim_dev;
static K_SEM_DEFINE(idle_sem, 0, 1);

static void idle_safe_off_handler(struct k_work *work);
static K_WORK_DEFINE(idle_safe_off_work, idle_safe_off_handler);

// -- VALIDATION --

static bool extra_reg_allowed(uint8_t reg) {
    return reg == RHS2116_REG_CHARGE_RECOVERY || reg == RHS2116_REG_CL_RECOVERY_EN ||
           reg == RHS2116_REG_FAST_SETTLE || reg == RHS2116_REG_FL_SELECT;
}

static int validate(const struct stim_program *p, enum stim_reject *why) {
    if (p->chan_mask == 0) {
        *why = STIM_BAD_MASK;
        return -EINVAL;
    }
    if (p->n_events == 0 || p->n_events > STIM_MAX_EVENTS) {
        *why = STIM_BAD_EVENTS;
        return -EINVAL;
    }

    // last value written to registers that must be back at rest when a pulse ends
    uint16_t last_recovery = 0, last_cl_recovery = 0, last_settle = 0;

    for (uint8_t i = 0; i < p->n_events; i++) {
        const struct stim_event *e = &p->events[i];

        if (i > 0 && e->frame <= p->events[i - 1].frame) {
            *why = STIM_BAD_EVENTS;
            return -EINVAL;
        }
        if ((e->on | e->pol) & ~p->chan_mask) {
            *why = STIM_BAD_MASK;
            return -EINVAL;
        }
        if (e->n_extra > 2) {
            *why = STIM_BAD_EVENTS;
            return -EINVAL;
        }
        for (uint8_t x = 0; x < e->n_extra; x++) {
            uint8_t reg = e->extra[x].reg;
            if (!extra_reg_allowed(reg)) {
                *why = STIM_BAD_EVENTS;
                return -EINVAL;
            }
            if (reg == RHS2116_REG_CHARGE_RECOVERY) {
                last_recovery = e->extra[x].val;
            } else if (reg == RHS2116_REG_CL_RECOVERY_EN) {
                last_cl_recovery = e->extra[x].val;
            } else if (reg == RHS2116_REG_FAST_SETTLE) {
                last_settle = e->extra[x].val;
            }
        }
    }

    const struct stim_event *last = &p->events[p->n_events - 1];
    if (last->on != 0 || last_recovery != 0 || last_cl_recovery != 0 || last_settle != 0) {
        *why = STIM_BAD_EVENTS;
        return -EINVAL;
    }

    if (p->count != 1 && p->period_frames <= last->frame) {
        *why = STIM_PERIOD_TOO_SHORT;
        return -EINVAL;
    }

    if (!(p->flags & STIM_FLAG_ALLOW_IMBALANCE)) {
        // charge per channel over one pulse, in amplitude-step x frames
        for (uint8_t ch = 0; ch < RHS2116_NUM_CHANNELS; ch++) {
            uint32_t anodic = 0, cathodic = 0;
            for (uint8_t i = 0; i + 1 < p->n_events; i++) {
                const struct stim_event *e = &p->events[i];
                uint32_t dur = p->events[i + 1].frame - e->frame;
                if (e->on & BIT(ch)) {
                    if (e->pol & BIT(ch)) {
                        anodic += p->pos_mag * dur;
                    } else {
                        cathodic += p->neg_mag * dur;
                    }
                }
            }
            if (anodic != cathodic) {
                LOG_WRN("ch %u unbalanced: anodic %u, cathodic %u (step x frames)",
                        ch, anodic, cathodic);
                *why = STIM_UNBALANCED;
                return -EINVAL;
            }
        }
    }

    *why = STIM_OK;
    return 0;
}

static int load(const struct stim_program *p, enum stim_reject *why) {
    if (!recording_active()) {
        *why = STIM_NOT_RECORDING;
        return -ENOTCONN;
    }

    int ret = validate(p, why);
    if (ret < 0) {
        return ret;
    }

    k_spinlock_key_t key = k_spin_lock(&lock);
    if (state != ST_IDLE) {
        k_spin_unlock(&lock, key);
        *why = STIM_BUSY;
        return -EBUSY;
    }
    prog = *p;
    prog_rate_mhz = rhs2116_stream_get_layout()->rate_mhz;
    loaded = true;
    k_spin_unlock(&lock, key);
    return 0;
}

int stim_load_program(const struct stim_program *p, enum stim_reject *why) {
    return load(p, why);
}

static uint32_t us_to_frames(uint32_t us, uint32_t rate_mhz) {
    return (uint32_t)(((uint64_t)us * rate_mhz + 500000000ull) / 1000000000ull);
}

static void add_event(struct stim_program *p, uint32_t frame, uint16_t on, uint16_t pol) {
    p->events[p->n_events++] = (struct stim_event){ .frame = frame, .on = on, .pol = pol };
}

static void add_extra(struct stim_program *p, uint8_t reg, uint16_t val) {
    struct stim_event *e = &p->events[p->n_events - 1];
    e->extra[e->n_extra].reg = reg;
    e->extra[e->n_extra].val = val;
    e->n_extra++;
}

int stim_load_biphasic(const struct stim_biphasic *b, struct stim_biphasic_frames *f,
                       enum stim_reject *why) {
    if (!recording_active()) {
        *why = STIM_NOT_RECORDING;
        return -ENOTCONN;
    }

    uint32_t rate = rhs2116_stream_get_layout()->rate_mhz;
    *f = (struct stim_biphasic_frames){
        .phase1 = us_to_frames(b->phase1_us, rate),
        .interphase = us_to_frames(b->interphase_us, rate),
        .phase2 = us_to_frames(b->phase2_us, rate),
        .recovery = us_to_frames(b->recovery_us, rate),
        .period = us_to_frames(b->period_us, rate),
    };
    if (f->phase1 == 0 || f->phase2 == 0) {
        *why = STIM_TOO_SHORT;
        return -EINVAL;
    }

    uint16_t mask = b->chan_mask;
    uint16_t pol1 = (b->flags & STIM_FLAG_ANODIC_FIRST) ? mask : 0;
    uint16_t pol2 = mask & ~pol1;

    struct stim_program p = {
        .chan_mask = mask,
        .neg_mag = b->neg_mag,
        .pos_mag = b->pos_mag,
        .flags = b->flags,
        .period_frames = f->period,
        .count = b->count,
        .recovery_frames = f->recovery,
    };

    uint32_t t = 0;
    add_event(&p, t, mask, pol1);
    t += f->phase1;
    if (f->interphase) {
        add_event(&p, t, 0, pol1);
        t += f->interphase;
    }
    add_event(&p, t, mask, pol2);
    t += f->phase2;
    add_event(&p, t, 0, 0);
    if (f->recovery) {
        add_extra(&p, RHS2116_REG_CHARGE_RECOVERY, mask);
        t += f->recovery;
        add_event(&p, t, 0, 0);
        add_extra(&p, RHS2116_REG_CHARGE_RECOVERY, 0);
    }

    return load(&p, why);
}

// -- FRAME PROVIDER (stream thread) --

static void emit_event(const struct stim_event *e, uint32_t frame) {
    recording_event(REC_EVT_STIM_EDGE, frame, e->on, e->pol);
    for (uint8_t x = 0; x < e->n_extra; x++) {
        if (e->extra[x].reg == RHS2116_REG_CHARGE_RECOVERY) {
            recording_event(REC_EVT_RECOVERY, frame, e->extra[x].val, 0);
        }
    }
}

// lock held
static void begin_shutdown(uint8_t reason) {
    state = ST_SHUTDOWN;
    sd_step = (reason == 0) ? SD_DISABLE : SD_OFF;
    abort_reason = reason;
}

// lock held; stream thread
static void finish_locked(void) {
    state = ST_IDLE;
    rhs2116_stream_set_stim_provider(NULL);
    recording_set_stim_active(false);
    k_sem_give(&idle_sem);
}

static void shutdown_frame(uint32_t f, struct rhs2116_frame_cmds *c) {
    c->stim_on = 0;
    c->stim_pol = cur_pol;

    switch (sd_step) {
    case SD_OFF:
        cur_on = 0;
        recording_event(REC_EVT_STIM_ABORT, f, abort_reason, 0);
        recording_event(REC_EVT_STIM_EDGE, f, 0, cur_pol);
        if (prog.recovery_frames) {
            c->extra[c->n_extra].reg = RHS2116_REG_CHARGE_RECOVERY;
            c->extra[c->n_extra++].val = prog.chan_mask;
            recording_event(REC_EVT_RECOVERY, f, prog.chan_mask, 0);
            sd_until = f + prog.recovery_frames;
            sd_step = SD_RECOVERY;
        } else {
            sd_step = SD_DISABLE;
        }
        break;
    case SD_RECOVERY:
        if ((int32_t)(f - sd_until) >= 0) {
            c->extra[c->n_extra].reg = RHS2116_REG_CHARGE_RECOVERY;
            c->extra[c->n_extra++].val = 0;
            recording_event(REC_EVT_RECOVERY, f, 0, 0);
            sd_step = SD_DISABLE;
        }
        break;
    case SD_DISABLE:
        c->extra[0].reg = RHS2116_REG_STIM_ENABLE_A;
        c->extra[0].val = 0;
        c->extra[1].reg = RHS2116_REG_STIM_ENABLE_B;
        c->extra[1].val = 0;
        c->n_extra = 2;
        if (abort_reason == 0) {
            recording_event(REC_EVT_STIM_DONE, f, pulses, 0);
        }
        finish_locked();
        break;
    }
}

static void provider(uint32_t f, struct rhs2116_frame_cmds *c) {
    k_spinlock_key_t key = k_spin_lock(&lock);

    if (state == ST_ARMED) {
        state = ST_RUNNING;
        pulse_start = f;
        pulses = 0;
        next_ev = 0;
    }

    if (state == ST_RUNNING) {
        const struct stim_event *e = &prog.events[next_ev];
        if (f - pulse_start == e->frame) {
            cur_on = e->on;
            cur_pol = e->pol;
            for (uint8_t x = 0; x < e->n_extra; x++) {
                c->extra[x].reg = e->extra[x].reg;
                c->extra[x].val = e->extra[x].val;
            }
            c->n_extra = e->n_extra;
            emit_event(e, f);

            if (++next_ev == prog.n_events) {
                pulses++;
                if (prog.count && pulses == prog.count) {
                    begin_shutdown(0);
                } else {
                    pulse_start += prog.period_frames;
                    next_ev = 0;
                }
            }
        }
        c->stim_on = cur_on;
        c->stim_pol = cur_pol;
    } else if (state == ST_SHUTDOWN) {
        shutdown_frame(f, c);
    }

    k_spin_unlock(&lock, key);
}

// stream thread: abort on anything that makes the delivered charge uncertain
static void fault_hook(enum rhs2116_stream_status status, uint32_t frame, uint32_t value) {
    ARG_UNUSED(frame);
    ARG_UNUSED(value);

    k_spinlock_key_t key = k_spin_lock(&lock);

    if (state == ST_IDLE) {
        k_spin_unlock(&lock, key);
        return;
    }

    switch (status) {
    case RHS2116_STREAM_STATUS_COMPLIANCE:
        if (state != ST_SHUTDOWN) {
            begin_shutdown(STIM_ABORT_COMPLIANCE);
        }
        break;
    case RHS2116_STREAM_STATUS_SPI_ERROR:
        if (state != ST_SHUTDOWN) {
            begin_shutdown(STIM_ABORT_SPI_ERROR);
        }
        break;
    case RHS2116_STREAM_STATUS_STOPPED:
        // no stream left to schedule the shutdown: do it with direct SPI writes
        abort_reason = STIM_ABORT_STREAM_FAULT;
        finish_locked();
        k_work_submit(&idle_safe_off_work);
        break;
    }

    k_spin_unlock(&lock, key);
}

static int stim_init(void) {
    recording_set_fault_hook(fault_hook);
    return 0;
}
SYS_INIT(stim_init, APPLICATION, 1);

// idle-mode safe off for when the stream is gone; timing is best effort
static void idle_safe_off_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (stim_dev == NULL || rhs2116_stream_active()) {
        return;
    }

    LOG_WRN("stream stopped during stimulation, switching stimulators off directly");
    if (prog.recovery_frames) {
        uint32_t us = (uint32_t)((uint64_t)prog.recovery_frames * 1000000000ull / prog_rate_mhz);
        const struct rhs2116_stim_state recover = { .recovery = prog.chan_mask };
        (void)rhs2116_stim_output(stim_dev, &recover);
        k_busy_wait(us);
    }
    // stimulators off, recovery switch open, stimulators disabled
    (void)rhs2116_stim_disarm(stim_dev);
}

// -- CONTROL --

int stim_start(const struct device *dev, enum stim_reject *why) {
    if (!recording_active()) {
        *why = STIM_NOT_RECORDING;
        return -ENOTCONN;
    }

    k_spinlock_key_t key = k_spin_lock(&lock);
    if (!loaded) {
        k_spin_unlock(&lock, key);
        *why = STIM_NO_PROGRAM;
        return -EINVAL;
    }
    if (state != ST_IDLE) {
        k_spin_unlock(&lock, key);
        *why = STIM_BUSY;
        return -EBUSY;
    }
    if (prog_rate_mhz != rhs2116_stream_get_layout()->rate_mhz) {
        k_spin_unlock(&lock, key);
        *why = STIM_RATE_CHANGED;
        return -EINVAL;
    }
    k_spin_unlock(&lock, key);

    stim_dev = dev;

    // amplitudes (applied at the end of the frame, while the stimulators are still
    // off), then clear the compliance monitor and arm the stimulators
    int ret = rhs2116_stim_set_amplitude(dev, prog.chan_mask, prog.neg_mag, prog.pos_mag);
    if (ret == 0) {
        ret = rhs2116_stim_arm(dev);
    }
    if (ret < 0) {
        LOG_ERR("stim setup failed: %d", ret);
        (void)rhs2116_stim_disarm(dev);
        *why = STIM_OK;
        return ret;
    }

    key = k_spin_lock(&lock);
    state = ST_ARMED;
    cur_on = 0;
    cur_pol = 0;
    abort_reason = 0;
    k_sem_reset(&idle_sem);
    recording_set_stim_active(true);
    rhs2116_stream_set_stim_provider(provider);
    k_spin_unlock(&lock, key);

    *why = STIM_OK;
    LOG_INF("stimulation started: mask 0x%04x, %u events, period %u frames, count %u",
            prog.chan_mask, prog.n_events, prog.period_frames, prog.count);
    return 0;
}

void stim_stop(void) {
    k_spinlock_key_t key = k_spin_lock(&lock);

    switch (state) {
    case ST_IDLE:
        k_spin_unlock(&lock, key);
        return;
    case ST_ARMED:
        // never reached the stream: just disarm
        state = ST_IDLE;
        rhs2116_stream_set_stim_provider(NULL);
        recording_set_stim_active(false);
        k_spin_unlock(&lock, key);
        (void)rhs2116_stim_disarm(stim_dev);
        return;
    case ST_RUNNING:
        begin_shutdown(STIM_ABORT_STOPPED);
        break;
    case ST_SHUTDOWN:
        break;
    }
    k_spin_unlock(&lock, key);

    if (k_sem_take(&idle_sem, STOP_TIMEOUT) < 0) {
        LOG_ERR("stim shutdown timed out");
    }
}

bool stim_running(void) {
    return state != ST_IDLE;
}
