#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>

#include <rhs2116/rhs2116.h>
#include <rhs2116/rhs2116_stream.h>
#include <rhs2116/rhs2116_transport.h>

#include "rhs2116_cmd.h"
#ifdef CONFIG_RHS2116_EMUL
#include "rhs2116_emul.h"
#endif

LOG_MODULE_REGISTER(rhs2116_stream, CONFIG_RHS2116_LOG_LEVEL);

// a block is ~4 ms of words: the stream thread has that long to refill one block
// while the other is clocked out
#define BLOCK_US 4000
// the guard covers ~0.3 ms of rewind ISR latency
#define GUARD_US 300

// 4 ms at the fastest word rate (16 MHz / 88 = 182 kwords/s) is 728 words
#define MAX_BLOCK_WORDS 768
#define MAX_GUARD_WORDS 128
#define RING_WORDS      (2 * MAX_BLOCK_WORDS + MAX_GUARD_WORDS)

// misc and stim slot offsets after the CONVERT slots of a frame
#define SLOT_MISC0   0
#define SLOT_MISC1   1
#define SLOT_POL     2
#define SLOT_ON      3

#define STREAM_THREAD_STACK 2048
#define STREAM_THREAD_PRIO  K_PRIO_COOP(1)
#define EVT_QUEUE_DEPTH     8
#define REG_ACCESS_TIMEOUT  K_MSEC(100)

enum tag_kind {
    TAG_NONE = 0,   // response not of interest (commands sent before streaming)
    TAG_CONVERT,    // arg: sample index within the frame
    TAG_WRITE,      // arg: register, val: data; echo is checked
    TAG_DUMMY,      // READ(255); must return the chip ID
    TAG_COMPLIANCE, // READ(40) with M
    TAG_HOST,       // queued register access; val: request generation
};

struct slot_tag {
    uint8_t kind;
    uint8_t arg;
    uint16_t val;
};

struct blk_evt {
    uint32_t seq;
    uint16_t guard_used;
    uint8_t block;
    int8_t err;
};

// rings: [A | B | guard]; tags only cover A and B (the guard mirrors A's tags)
static uint32_t tx_ring[RING_WORDS] __aligned(4);
static uint32_t rx_ring[RING_WORDS] __aligned(4);
static struct slot_tag tags[2 * MAX_BLOCK_WORDS];

static struct rhs2116_stream_layout layout;
static uint32_t block_words;
static uint32_t guard_words;
static const struct rhs2116_stream_sink *sink;
static atomic_ptr_t stim_provider;

static K_MUTEX_DEFINE(state_lock);
static atomic_t active;

K_MSGQ_DEFINE(evt_q, sizeof(struct blk_evt), EVT_QUEUE_DEPTH, 4);
static atomic_t isr_seq;      // block events issued by the ISR
static atomic_t evt_overflow;

static struct slot_tag carry[2]; // tags of the previous block's last two words
static uint16_t frame_samples[RHS2116_NUM_CHANNELS];
static uint32_t rx_frame;        // frames received so far
static struct rhs2116_stream_stats stats;

// pre-encoded fixed words
static uint32_t w_dummy;
static uint32_t w_compliance;
static uint32_t w_safe_pol;
static uint32_t w_safe_on;

// the single in-flight register access from another thread
enum req_state { REQ_IDLE, REQ_PENDING, REQ_SENT, REQ_DONE };

static struct {
    struct k_mutex lock;  // one caller at a time
    struct k_spinlock sl; // state shared with the stream thread
    enum req_state state;
    uint16_t gen;
    uint32_t cmd;
    bool write;
    uint16_t data;
    uint32_t result;
    int err;
    struct k_sem done;
} req;

/*
 * Boot-time setup of the host register-access request slot (its caller mutex and
 * completion semaphore). Runs from SYS_INIT so rhs2116_stream_reg_access() is safe
 * to call before any stream has been started.
 */
static int req_init(void) {
    k_mutex_init(&req.lock);
    k_sem_init(&req.done, 0, 1);
    return 0;
}
SYS_INIT(req_init, APPLICATION, 0);

// -- LAYOUT --

/*
 * Highest per-channel sample rate the SPI can sustain for n_ch channels. A frame is
 * n_ch CONVERT slots plus RHS2116_STREAM_OVERHEAD_SLOTS misc/stim slots, and every
 * slot needs at least RHS2116_STREAM_MIN_SLOT_TICKS of the 16 MHz pacing clock, so
 *
 *   rate = TICK_HZ / (MIN_SLOT_TICKS * (n_ch + OVERHEAD_SLOTS))
 */
uint32_t rhs2116_stream_max_rate_hz(uint8_t n_ch) {
    return RHS2116_STREAM_TICK_HZ /
           (RHS2116_STREAM_MIN_SLOT_TICKS * (n_ch + RHS2116_STREAM_OVERHEAD_SLOTS));
}

/*
 * Turns a channel mask and requested per-channel rate into a concrete frame/ring
 * layout. can also be used to validate a request up front
 *
 *   chans[]          the enabled channels in ascending order. This is the order of
 *                    the CONVERT slots in every frame and of the samples handed to
 *                    the sink.
 *   slot_ticks       16 MHz ticks per SPI word, rounded down, so the actual rate
 *                    (rate_mhz, in milli-Hz) is at or slightly above rate_hz.
 *   frames_per_block enough whole frames to cover BLOCK_US (~4 ms), which is how
 *                    long the stream thread has to refill a block; capped so a
 *                    block fits in MAX_BLOCK_WORDS.
 *   guard_frames     enough frames to cover GUARD_US of rewind ISR latency, plus
 *                    one spare; capped to one block and to MAX_GUARD_WORDS.
 *
 * Returns 0 with *out filled and *why = RHS2116_STREAM_OK, or -EINVAL with *why
 * saying which limit was hit (*out is left untouched).
 */
int rhs2116_stream_plan(uint16_t chan_mask, uint32_t rate_hz, struct rhs2116_stream_layout *out,
                        enum rhs2116_stream_reject *why) {
    struct rhs2116_stream_layout l = { .chan_mask = chan_mask };

    for (uint8_t ch = 0; ch < RHS2116_NUM_CHANNELS; ch++) {
        if (chan_mask & BIT(ch)) {
            l.chans[l.n_ch++] = ch;
        }
    }
    if (l.n_ch == 0) {
        *why = RHS2116_STREAM_BAD_MASK;
        return -EINVAL;
    }
    if (rate_hz < RHS2116_STREAM_MIN_RATE_HZ) {
        *why = RHS2116_STREAM_RATE_TOO_LOW;
        return -EINVAL;
    }

    l.slots_per_frame = l.n_ch + RHS2116_STREAM_OVERHEAD_SLOTS;
    l.slot_ticks = RHS2116_STREAM_TICK_HZ / (rate_hz * l.slots_per_frame);
    if (l.slot_ticks < RHS2116_STREAM_MIN_SLOT_TICKS) {
        *why = RHS2116_STREAM_RATE_TOO_HIGH;
        return -EINVAL;
    }

    uint32_t frame_ticks = l.slot_ticks * l.slots_per_frame;
    l.rate_mhz = (uint32_t)((uint64_t)RHS2116_STREAM_TICK_HZ * 1000 / frame_ticks);

    // frames per block/guard from the frame period in ticks (16 ticks per us)
    uint32_t fpb = DIV_ROUND_UP(BLOCK_US * 16u, frame_ticks);
    fpb = CLAMP(fpb, 1, MAX_BLOCK_WORDS / l.slots_per_frame);
    uint32_t guard = DIV_ROUND_UP(GUARD_US * 16u, frame_ticks) + 1;
    guard = CLAMP(guard, 1, MIN(fpb, MAX_GUARD_WORDS / l.slots_per_frame));

    l.frames_per_block = fpb;
    l.guard_frames = guard;

    *out = l;
    *why = RHS2116_STREAM_OK;
    return 0;
}

// -- RING HELPERS --

/*
 * Writes one ring slot: the command into tx_ring (in chip byte order) and a tag
 * recording what it was, so the response that comes back two words later can be
 * routed. Only for indices in blocks A and B; the guard has no tags of its own.
 */
static inline void put(uint32_t idx, uint32_t cmd, enum tag_kind kind, uint8_t arg, uint16_t val) {
    tx_ring[idx] = rhs2116_wire(cmd);
    tags[idx] = (struct slot_tag){ .kind = kind, .arg = arg, .val = val };
}

/*
 * Ring index of the first word (the first CONVERT slot) of frame 'frame' in block
 * 'block' (0 = A, 1 = B). The frame's misc/stim slots start at + layout.n_ch.
 */
static inline uint32_t frame_word(int block, uint32_t frame) {
    return block * block_words + frame * layout.slots_per_frame;
}

/*
 * Fail-safe scrub, run from the block ISR. For n_frames frames starting at ring
 * index first_word, overwrites the four overhead slots with harmless commands: both
 * misc slots become dummy READ(CHIP_ID)s and the stim slots write 0 to STIM_POL and
 * STIM_ON (stimulators off). CONVERT slots are left alone.
 *
 * Only the TX words change. The tags still describe what was actually sent, which
 * process_rx() needs to route those responses. If the stream thread then fails to
 * refill in time, the block goes out again as "record only, no stim" instead of
 * repeating an old stimulation pattern.
 */
static void reset_frames_safe(uint32_t first_word, uint32_t n_frames) {
    for (uint32_t j = 0; j < n_frames; j++) {
        uint32_t w = first_word + j * layout.slots_per_frame + layout.n_ch;
        tx_ring[w + SLOT_MISC0] = w_dummy;
        tx_ring[w + SLOT_MISC1] = w_dummy;
        tx_ring[w + SLOT_POL] = w_safe_pol;
        tx_ring[w + SLOT_ON] = w_safe_on;
    }
}

/*
 * Transport callback, called from the TIMER3 ISR each time a block has been
 * completely clocked out (block 0 = A, 1 = B; they alternate A, B, A, ...).
 *
 * On success it scrubs the block just sent with reset_frames_safe(). After A it also
 * scrubs the guard, since the guard is a copy of A's head and was clocked out at the
 * start of this pass of A.
 *
 * It then posts a blk_evt to the stream thread carrying a sequence number (0, 1, 2,
 * ... counting every block pass since start), how many guard words were used and
 * any transport error. If the queue is full it sets evt_overflow, so the thread
 * faults the stream instead of silently skipping a block.
 *
 * Keep this short: it runs in interrupt context while the next block is already
 * being clocked out.
 */
static void block_isr(int block, uint32_t guard_used, int err) {
    struct blk_evt e = {
        .seq = (uint32_t)atomic_inc(&isr_seq),
        .guard_used = (uint16_t)guard_used,
        .block = (uint8_t)block,
        .err = (int8_t)err,
    };

    if (err == 0) {
        reset_frames_safe(frame_word(block, 0), layout.frames_per_block);
        if (block == 0) {
            reset_frames_safe(2 * block_words, layout.guard_frames);
        }
    }

    if (k_msgq_put(&evt_q, &e, K_NO_WAIT) != 0) {
        atomic_set(&evt_overflow, 1);
    }
}

/*
 * Chooses the command for one misc slot (the m-th of the frame) at ring index idx,
 * in priority order:
 *
 *   1. the stim provider's extra register write c->extra[m], if it supplied one;
 *   2. a pending host register access from rhs2116_stream_reg_access(). Its command
 *      goes out here and the request moves PENDING -> SENT. The tag carries the
 *      request generation so a stale response can't complete a newer request;
 *   3. the block's one compliance-monitor read (READ(40) with M set, which also
 *      clears the latched bits), tracked through *compliance_due;
 *   4. otherwise a dummy READ(CHIP_ID), whose known response doubles as an SPI
 *      link check.
 */
static void fill_misc(uint32_t idx, const struct rhs2116_frame_cmds *c, int m,
                      bool *compliance_due) {
    if (m < c->n_extra) {
        put(idx, rhs2116_cmd_write(c->extra[m].reg, c->extra[m].val, false, false),
            TAG_WRITE, c->extra[m].reg, c->extra[m].val);
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&req.sl);
    if (req.state == REQ_PENDING) {
        put(idx, req.cmd, TAG_HOST, 0, req.gen);
        req.state = REQ_SENT;
        k_spin_unlock(&req.sl, key);
        return;
    }
    k_spin_unlock(&req.sl, key);

    if (*compliance_due) {
        put(idx, rhs2116_unwire(w_compliance), TAG_COMPLIANCE, 0, 0);
        *compliance_due = false;
        return;
    }

    put(idx, rhs2116_unwire(w_dummy), TAG_DUMMY, 0, 0);
}

/*
 * Writes the misc and stim slots of every frame in 'block' for its next pass over
 * the SPI. 'seq' is the block-pass sequence number that pass will have (the same
 * numbering as blk_evt.seq), so frame j of the block is absolute frame
 * seq * frames_per_block + j.
 *
 * For each frame the stim provider, if installed, fills that frame's commands, some
 * milliseconds before they are clocked out. stim_pol and stim_on go in the last two
 * slots (STIM_ON is written with U set, so every triggered register change made in
 * the frame takes effect together at its end). Up to two extra writes, then any
 * queued host access, the compliance read or dummy reads fill the misc slots (see
 * fill_misc()).
 *
 * CONVERT slots are not touched (see build_converts()). When refilling A, A's head
 * is then copied into the guard so the guard keeps mirroring A.
 *
 * Called by rhs2116_stream_start() for the first two passes, and after that by the
 * stream thread for each block event.
 */
static void refill(int block, uint32_t seq) {
    rhs2116_stim_provider_t provider = (rhs2116_stim_provider_t)atomic_ptr_get(&stim_provider);
    uint32_t frame0 = seq * layout.frames_per_block;
    bool compliance_due = true; // one compliance read per block
    const uint8_t s = layout.slots_per_frame;

    for (uint32_t j = 0; j < layout.frames_per_block; j++) {
        struct rhs2116_frame_cmds c = {0};
        if (provider) {
            provider(frame0 + j, &c);
        }
        c.n_extra = MIN(c.n_extra, 2);

        uint32_t w = frame_word(block, j) + layout.n_ch;
        fill_misc(w + SLOT_MISC0, &c, 0, &compliance_due);
        fill_misc(w + SLOT_MISC1, &c, 1, &compliance_due);
        put(w + SLOT_POL, rhs2116_cmd_write(RHS2116_REG_STIM_POL, c.stim_pol, false, false),
            TAG_WRITE, RHS2116_REG_STIM_POL, c.stim_pol);
        put(w + SLOT_ON, rhs2116_cmd_write(RHS2116_REG_STIM_ON, c.stim_on, true, false),
            TAG_WRITE, RHS2116_REG_STIM_ON, c.stim_on);
    }

    if (block == 0) {
        // the guard is A's head, clocked just before the rewind into A
        memcpy(&tx_ring[2 * block_words], &tx_ring[0], layout.guard_frames * s * sizeof(uint32_t));
    }
}

/*
 * Fills the CONVERT slots (the first n_ch words of every frame) of blocks A and B,
 * once at start. They are identical in every frame and the ISR never scrubs them,
 * so they stay valid for the whole stream. The tag's arg is the slot index within
 * the frame (not the channel number); route() stores the sample at that position.
 */
static void build_converts(void) {
    for (int block = 0; block < 2; block++) {
        for (uint32_t j = 0; j < layout.frames_per_block; j++) {
            uint32_t w = frame_word(block, j);
            for (uint8_t k = 0; k < layout.n_ch; k++) {
                put(w + k, rhs2116_cmd_convert(layout.chans[k], 0), TAG_CONVERT, k, 0);
            }
        }
    }
}

// -- RESPONSE ROUTING --

/*
 * Passes a status event to the sink's status callback, if there is one, stamped
 * with the number of frames received so far. Stream thread only.
 */
static void report(enum rhs2116_stream_status status, uint32_t value) {
    if (sink && sink->status) {
        sink->status(status, rx_frame, value);
    }
}

/*
 * The response to a host register access (TAG_HOST) has arrived. If it belongs to
 * the request currently in flight (state SENT with the same generation), this stores
 * the raw response, checks the echo for a write (-EIO on mismatch), marks the request
 * DONE and wakes the waiting caller. A response to a request that already timed out
 * or was cancelled no longer matches and is dropped.
 */
static void complete_host(uint16_t gen, uint32_t word) {
    k_spinlock_key_t key = k_spin_lock(&req.sl);
    if (req.state == REQ_SENT && req.gen == gen) {
        req.result = word;
        req.err = (req.write && word != RHS2116_WRITE_ECHO(req.data)) ? -EIO : 0;
        req.state = REQ_DONE;
        k_sem_give(&req.done);
    }
    k_spin_unlock(&req.sl, key);
}

/*
 * Handles one response word, given the tag of the command that produced it (sent
 * two words earlier):
 *
 *   CONVERT     store the AC sample at its slot position. When the last channel of
 *               the frame arrives, give the whole frame to sink->frame() and
 *               advance rx_frame.
 *   WRITE       the chip must echo the written value, otherwise count an SPI error
 *   DUMMY       READ(CHIP_ID) must return the chip ID, otherwise an SPI error
 *   COMPLIANCE  any of the low 16 bits set means a channel hit its compliance
 *               limit; report it
 *   HOST        complete the waiting register access
 *   NONE        ignored (the first two responses after start answer no command)
 */
static void route(struct slot_tag t, uint32_t word) {
    switch (t.kind) {
    case TAG_CONVERT:
        frame_samples[t.arg] = rhs2116_convert_ac(word);
        if (t.arg == layout.n_ch - 1) {
            if (sink && sink->frame) {
                sink->frame(rx_frame, frame_samples, layout.n_ch);
            }
            rx_frame++;
            stats.frames++;
        }
        break;
    case TAG_WRITE:
        if (word != RHS2116_WRITE_ECHO(t.val)) {
            stats.spi_errors++;
            report(RHS2116_STREAM_STATUS_SPI_ERROR, word);
        }
        break;
    case TAG_DUMMY:
        if (word != RHS2116_CHIP_ID) {
            stats.spi_errors++;
            report(RHS2116_STREAM_STATUS_SPI_ERROR, word);
        }
        break;
    case TAG_COMPLIANCE:
        if (word & 0xFFFF) {
            stats.compliance_events++;
            report(RHS2116_STREAM_STATUS_COMPLIANCE, word & 0xFFFF);
        }
        break;
    case TAG_HOST:
        complete_host(t.val, word);
        break;
    default:
        break;
    }
}

#ifdef CONFIG_RHS2116_EMUL
/*
 * CONFIG_RHS2116_EMUL only. With no chip attached, the response for a slot comes
 * from the software model instead of rx_ring. The ISR has already scrubbed the
 * misc/stim TX words by now, so the command each slot actually carried is rebuilt
 * from its tag (a host access uses req.cmd). Returns the response as the chip
 * would give it, in host byte order.
 */
static uint32_t emul_word(struct slot_tag t) {
    uint32_t cmd;

    switch (t.kind) {
    case TAG_CONVERT:
        cmd = rhs2116_cmd_convert(layout.chans[t.arg], 0);
        break;
    case TAG_WRITE:
        cmd = rhs2116_cmd_write(t.arg, t.val, false, false);
        break;
    case TAG_DUMMY:
        cmd = rhs2116_cmd_read(RHS2116_REG_CHIP_ID, false, false);
        break;
    case TAG_COMPLIANCE:
        cmd = rhs2116_cmd_read(RHS2116_REG_COMPLIANCE, false, true);
        break;
    case TAG_HOST: {
        k_spinlock_key_t key = k_spin_lock(&req.sl);
        cmd = req.cmd;
        k_spin_unlock(&req.sl, key);
        break;
    }
    default:
        return 0;
    }
    return rhs2116_emul_response(cmd);
}
#endif

/*
 * Walks the RX words of the block that just finished and routes each response.
 *
 * The RHS2116 answers a command two words later (sent at N, executed at N+1,
 * result at N+2), so RX word i belongs to the tag at i - 2. For the first two words
 * of a block those tags are the last two of the previous block, kept in carry[];
 * this call then saves this block's last two tags for the next one.
 *
 * For block A, the first guard_used words were actually clocked out from the guard
 * (before the ISR rewound the SPIM into A), so their responses are read from the
 * RX guard instead of RX A. The tags are the same, since the guard mirrors A.
 */
static void process_rx(int block, uint32_t guard_used) {
    uint32_t base = block * block_words;

    for (uint32_t i = 0; i < block_words; i++) {
        // A's first words were clocked from the guard, so their responses are there
        uint32_t raw = (block == 0 && i < guard_used) ? rx_ring[2 * block_words + i]
                                                      : rx_ring[base + i];
        struct slot_tag t = (i >= 2) ? tags[base + i - 2] : carry[i];
#ifdef CONFIG_RHS2116_EMUL
        ARG_UNUSED(raw);
        route(t, emul_word(t));
#else
        route(t, rhs2116_unwire(raw));
#endif
    }

    carry[0] = tags[base + block_words - 2];
    carry[1] = tags[base + block_words - 1];
}

// -- CONTROL --

/*
 * Fails any queued or in-flight host register access with -ECANCELED and wakes its
 * caller. Used when the stream stops, since that command will never get a response.
 */
static void cancel_host_request(void) {
    k_spinlock_key_t key = k_spin_lock(&req.sl);
    if (req.state == REQ_PENDING || req.state == REQ_SENT) {
        req.err = -ECANCELED;
        req.state = REQ_DONE;
        k_sem_give(&req.done);
    }
    k_spin_unlock(&req.sl, key);
}

/*
 * Stops the stream if it is running; a no-op otherwise (active is cleared with a
 * compare-and-swap, so calling it twice is harmless). Halts the transport, saves
 * its final stats into stats.transport, drops any block events still queued and
 * cancels an outstanding host register access.
 *
 * Caller holds state_lock.
 */
static void stop_locked(void) {
    if (!atomic_cas(&active, 1, 0)) {
        return;
    }
    rhs2116_transport_stream_stop();
    rhs2116_transport_get_stats(&stats.transport);
    k_msgq_purge(&evt_q);
    cancel_host_request();
}

/*
 * The stream hit an unrecoverable problem: logs it, stops the stream and tells the
 * sink with RHS2116_STREAM_STATUS_STOPPED (value = the negative errno).
 *
 * Caller holds state_lock.
 */
static void fault_locked(int err) {
    LOG_ERR("stream fault %d at frame %u, stopping", err, rx_frame);
    stop_locked();
    report(RHS2116_STREAM_STATUS_STOPPED, (uint32_t)err);
}

/*
 * Stream thread work for one block event:
 *
 *   1. ignore it if the stream was stopped in the meantime;
 *   2. fault on a transport error (the rewind missed the guard) or if the event
 *      queue overflowed (a block event was lost, so responses can no longer be
 *      matched to tags);
 *   3. route every response of the finished block (process_rx());
 *   4. refill that block for its next pass, seq + 2 (pass seq + 1, the other
 *      block, is going out now);
 *   5. deadline check: if the ISR has already reported pass seq + 1, the other
 *      block finished and this one started going out again before the refill was
 *      done, so some of it was sent scrubbed or half written. That is a missed
 *      deadline and faults with -ETIME.
 *
 * Caller holds state_lock.
 */
static void handle_event(const struct blk_evt *e) {
    if (!atomic_get(&active)) {
        return;
    }
    if (e->err) {
        fault_locked(e->err); // rewind missed the guard
        return;
    }
    if (atomic_get(&evt_overflow)) {
        fault_locked(-EOVERFLOW);
        return;
    }

    process_rx(e->block, e->guard_used);
    refill(e->block, e->seq + 2);
    stats.blocks++;

    // the refill must land before the other block finishes and this one restarts
    if ((uint32_t)atomic_get(&isr_seq) > e->seq + 1) {
        fault_locked(-ETIME);
    }
}

/*
 * Cooperative-priority thread that does all the non-ISR stream work. It waits for
 * block events from block_isr() and handles each one under state_lock, so start,
 * stop and get_stats on other threads never see a half-processed block. The sink
 * and stim provider callbacks run on this thread.
 *
 * k_msgq_purge() (called by start and stop) wakes every thread waiting on the
 * queue, readers included, with -ENOMSG and no message. Such a wake-up must be
 * skipped: 'e' then holds stale stack contents, and handling it faulted every
 * stream start with a garbage error code.
 */
static void stream_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct blk_evt e;

    while (1) {
        if (k_msgq_get(&evt_q, &e, K_FOREVER) != 0) {
            continue;
        }
        k_mutex_lock(&state_lock, K_FOREVER);
        handle_event(&e);
        k_mutex_unlock(&state_lock);
    }
}

K_THREAD_DEFINE(rhs2116_stream_tid, STREAM_THREAD_STACK, stream_thread, NULL, NULL, NULL,
                STREAM_THREAD_PRIO, 0, 0);

/*
 * Starts continuous streaming of the channels in chan_mask at rate_hz per channel:
 *
 *   1. -EBUSY if already streaming; otherwise plan the layout
 *      (rhs2116_stream_plan());
 *   2. set the chip's ADC bias for the resulting SPI word rate;
 *   3. pre-encode the fixed words (dummy read, compliance read, stim-off writes);
 *   4. clear the rings, tags, stats and event queue, then build both blocks: the
 *      CONVERTs, then misc/stim for pass 0 (A, which also fills the guard) and
 *      pass 1 (B);
 *   5. mark the stream active (from here on register accesses go through misc
 *      slots) and hand the ring to the transport, which starts clocking it out.
 *
 * The sink callbacks are invoked on the stream thread. On success returns 0 and,
 * if out is given, copies the layout to it. On failure returns a negative errno and
 * nothing is left running; *why is only meaningful for -EINVAL and -EBUSY.
 */
int rhs2116_stream_start(const struct device *dev, uint16_t chan_mask, uint32_t rate_hz,
                         const struct rhs2116_stream_sink *s,
                         struct rhs2116_stream_layout *out, enum rhs2116_stream_reject *why) {
    k_mutex_lock(&state_lock, K_FOREVER);

    if (atomic_get(&active)) {
        *why = RHS2116_STREAM_BUSY;
        k_mutex_unlock(&state_lock);
        return -EBUSY;
    }

    int ret = rhs2116_stream_plan(chan_mask, rate_hz, &layout, why);
    if (ret < 0) {
        k_mutex_unlock(&state_lock);
        return ret;
    }

    // bias the ADC for the total command rate
    uint32_t word_rate = RHS2116_STREAM_TICK_HZ / layout.slot_ticks;
    ret = rhs2116_set_adc_rate(dev, word_rate);
    if (ret < 0) {
        k_mutex_unlock(&state_lock);
        return ret;
    }

    sink = s;
    block_words = layout.frames_per_block * layout.slots_per_frame;
    guard_words = layout.guard_frames * layout.slots_per_frame;

    w_dummy = rhs2116_wire(rhs2116_cmd_read(RHS2116_REG_CHIP_ID, false, false));
    w_compliance = rhs2116_wire(rhs2116_cmd_read(RHS2116_REG_COMPLIANCE, false, true));
    w_safe_pol = rhs2116_wire(rhs2116_cmd_write(RHS2116_REG_STIM_POL, 0, false, false));
    w_safe_on = rhs2116_wire(rhs2116_cmd_write(RHS2116_REG_STIM_ON, 0, true, false));

    memset(tx_ring, 0, sizeof(tx_ring));
    memset(rx_ring, 0, sizeof(rx_ring));
    memset(tags, 0, sizeof(tags));
    memset(carry, 0, sizeof(carry));
    stats = (struct rhs2116_stream_stats){0};
    rx_frame = 0;
    atomic_clear(&isr_seq);
    atomic_clear(&evt_overflow);
    k_msgq_purge(&evt_q);

    build_converts();
    refill(0, 0);
    refill(1, 1);

    struct rhs2116_ring ring = {
        .tx = tx_ring,
        .rx = rx_ring,
        .block_words = block_words,
        .guard_words = guard_words,
    };

    // set before the first block event can arrive; register access is now queued
    atomic_set(&active, 1);
    ret = rhs2116_transport_stream_start(&ring, layout.slot_ticks, block_isr);
    if (ret < 0) {
        atomic_clear(&active);
        k_mutex_unlock(&state_lock);
        return ret;
    }

    LOG_INF("streaming %u ch (mask 0x%04x) at %u.%03u S/s: %u slots x %u ticks, "
            "%u frames/block, %u guard frames",
            layout.n_ch, layout.chan_mask, layout.rate_mhz / 1000, layout.rate_mhz % 1000,
            layout.slots_per_frame, layout.slot_ticks, layout.frames_per_block,
            layout.guard_frames);

    if (out) {
        *out = layout;
    }
    k_mutex_unlock(&state_lock);
    return 0;
}

/*
 * Stops streaming; a no-op if not streaming. A waiting register access fails with
 * -ECANCELED. Unlike a fault, a requested stop sends no status event to the sink.
 */
void rhs2116_stream_stop(void) {
    k_mutex_lock(&state_lock, K_FOREVER);
    stop_locked();
    k_mutex_unlock(&state_lock);
}

/*
 * True from a successful rhs2116_stream_start() until a stop or fault. Lock-free.
 */
bool rhs2116_stream_active(void) {
    return atomic_get(&active);
}

/*
 * Number of frames received and handed to the sink so far, which is also the index
 * the next received frame will get. Lock-free, so it may be one block stale.
 */
uint32_t rhs2116_stream_current_frame(void) {
    return rx_frame;
}

/*
 * Layout of the current (or most recent) stream. Points at internal state that the
 * next rhs2116_stream_start() overwrites, so copy it if it needs to be kept.
 */
const struct rhs2116_stream_layout *rhs2116_stream_get_layout(void) {
    return &layout;
}

/*
 * Copies out the stream counters. While streaming, the transport stats are read
 * live; after a stop they are the values stop_locked() saved.
 */
void rhs2116_stream_get_stats(struct rhs2116_stream_stats *out) {
    k_mutex_lock(&state_lock, K_FOREVER);
    *out = stats;
    if (atomic_get(&active)) {
        rhs2116_transport_get_stats(&out->transport);
    }
    k_mutex_unlock(&state_lock);
}

/*
 * Installs the per-frame stimulation provider, or clears it with NULL. The swap is
 * atomic, so it can change mid-stream. refill() picks it up for the next block it
 * writes, so it takes effect on frames up to about two blocks (~8 ms) ahead of
 * what is being clocked out now.
 */
void rhs2116_stream_set_stim_provider(rhs2116_stim_provider_t provider) {
    atomic_ptr_set(&stim_provider, (atomic_ptr_val_t)provider);
}

/*
 * Reads or writes a chip register while the stream owns the SPI, by slipping the
 * command into the next free misc slot (see fill_misc()). Blocks the caller until
 * the response comes back, usually within a block or two.
 *
 * Only one access is in flight at a time (req.lock serialises callers). Each request
 * gets a new generation number, so a response that arrives after a timeout can't be
 * mistaken for the next request's.
 *
 * Returns 0 with *result (if given) set to the raw response (the echo for a write),
 * or:
 *   -EIO        a write echo didn't match
 *   -ENOTCONN   not streaming; the caller should use the direct SPI path
 *   -ETIMEDOUT  no response within REG_ACCESS_TIMEOUT
 *   -ECANCELED  the stream stopped while waiting
 *   -EDEADLK    called from the stream thread itself (e.g. from a sink or stim
 *               provider callback), which would wait forever for a response only
 *               that thread can deliver
 */
int rhs2116_stream_reg_access(bool write, uint8_t reg, uint16_t data, bool update, bool clear,
                              uint32_t *result) {
    if (k_current_get() == rhs2116_stream_tid) {
        return -EDEADLK;
    }

    k_mutex_lock(&req.lock, K_FOREVER);

    k_spinlock_key_t key = k_spin_lock(&req.sl);
    req.gen++;
    req.write = write;
    req.data = data;
    req.cmd = write ? rhs2116_cmd_write(reg, data, update, clear)
                    : rhs2116_cmd_read(reg, update, clear);
    req.err = 0;
    k_sem_reset(&req.done);
    req.state = atomic_get(&active) ? REQ_PENDING : REQ_IDLE;
    k_spin_unlock(&req.sl, key);

    int ret;
    if (req.state == REQ_IDLE) {
        ret = -ENOTCONN;
    } else if (k_sem_take(&req.done, REG_ACCESS_TIMEOUT) < 0) {
        key = k_spin_lock(&req.sl);
        req.state = REQ_IDLE; // a late response no longer matches
        req.gen++;
        k_spin_unlock(&req.sl, key);
        ret = -ETIMEDOUT;
    } else {
        key = k_spin_lock(&req.sl);
        ret = req.err;
        if (result) {
            *result = req.result;
        }
        req.state = REQ_IDLE;
        k_spin_unlock(&req.sl, key);
    }

    k_mutex_unlock(&req.lock);
    return ret;
}
