#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "ble_central.h"
#include "ep_cmd.h"
#include "link_test.h"

LOG_MODULE_REGISTER(link_test, CONFIG_LOG_DEFAULT_LEVEL);

/* ep_device link test packets start with a little-endian u32 sequence number
 * that increments by one per packet, so gaps in it are dropped packets. */
#define SEQ_LEN 4

static atomic_t active;

/* State below is written from the BT RX thread and the window timer (ISR) and
 * read from the shell thread, so it is only touched under lock. */
static struct k_spinlock lock;

/* streaming: packets are arriving and windows are being recorded. Cleared on
 * disconnect and set again by the first packet after the test resumes. */
static bool streaming;
static bool got_first;
static int64_t first_ms;
static int64_t end_ms;

static uint32_t received;
static uint32_t dropped;
static uint64_t bytes;
static uint32_t next_seq;

static uint32_t window_bytes;
static uint32_t windows[LINK_TEST_MAX_WINDOWS];
static uint32_t n_windows;
static bool windows_full;

static bool link_down;
static int64_t down_since;
static uint32_t disconnects;
static uint32_t down_ms;
static uint32_t longest_down_ms;

// -- WINDOW TIMER --

static void window_expiry(struct k_timer *timer) {
    ARG_UNUSED(timer);

    k_spinlock_key_t key = k_spin_lock(&lock);
    if (streaming) {
        if (n_windows < LINK_TEST_MAX_WINDOWS) {
            windows[n_windows++] = window_bytes;
        } else {
            windows_full = true;
        }
    }
    window_bytes = 0;
    k_spin_unlock(&lock, key);
}

static K_TIMER_DEFINE(window_timer, window_expiry, NULL);

// call with lock held
static void end_downtime(int64_t now) {
    if (!link_down) {
        return;
    }
    uint32_t dur = (uint32_t)(now - down_since);
    down_ms += dur;
    longest_down_ms = MAX(longest_down_ms, dur);
    link_down = false;
}

// -- BLE HOOKS --

void link_test_on_packet(const uint8_t *data, uint16_t len) {
    if (!atomic_get(&active) || len < SEQ_LEN) {
        return;
    }

    uint32_t seq = sys_get_le32(data);
    int64_t now = k_uptime_get();

    k_spinlock_key_t key = k_spin_lock(&lock);

    if (!streaming) {
        /* First packet of the test, or after a reconnect: the ep_device restarts
         * its sequence numbers, so take this one as the new baseline (packets
         * lost to the disconnect are reported as downtime, not drops) and start
         * the windows from here so setup time isn't counted as a slow window. */
        streaming = true;
        end_downtime(now);
        if (!got_first) {
            got_first = true;
            first_ms = now;
        }
        next_seq = seq;
        window_bytes = 0;
        k_timer_start(&window_timer, K_MSEC(LINK_TEST_WINDOW_MS), K_MSEC(LINK_TEST_WINDOW_MS));
    } else if (seq > next_seq) {
        dropped += seq - next_seq;
    }
    // seq < next_seq would be a duplicate/reorder, which BLE shouldn't produce

    received++;
    bytes += len;
    window_bytes += len;
    next_seq = seq + 1;

    k_spin_unlock(&lock, key);
}

void link_test_on_disconnect(void) {
    if (!atomic_get(&active)) {
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&lock);
    k_timer_stop(&window_timer);
    disconnects++;
    if (!link_down) {
        link_down = true;
        down_since = k_uptime_get();
    }
    // the window in progress is cut short, so it's discarded
    streaming = false;
    window_bytes = 0;
    k_spin_unlock(&lock, key);

    LOG_WRN("link test: ep_device disconnected, will resume on reconnect");
}

/* The ep_device stops its link test when the link drops, so restart it once
 * NUS is usable again. Runs on the system workqueue rather than the BT RX
 * thread, which must not block on a TX buffer. */
static void resume_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (!atomic_get(&active)) {
        return;
    }

    uint8_t cmd = EP_CMD_START_BLE_LINK_TEST;
    int ret = ble_central_send(&cmd, sizeof(cmd));
    if (ret) {
        LOG_ERR("link test: failed to resume: %d", ret);
    } else {
        LOG_INF("link test: resumed after reconnect");
    }
}

static K_WORK_DEFINE(resume_work, resume_work_handler);

void link_test_on_ready(void) {
    if (atomic_get(&active)) {
        k_work_submit(&resume_work);
    }
}

// -- CONTROL --

bool link_test_active(void) {
    return atomic_get(&active);
}

int link_test_start(void) {
    if (!atomic_cas(&active, 0, 1)) {
        return -EALREADY;
    }

    // counting is enabled before the ep_device starts sending so sequence 0 isn't missed
    k_spinlock_key_t key = k_spin_lock(&lock);
    streaming = false;
    got_first = false;
    first_ms = 0;
    end_ms = 0;
    received = 0;
    dropped = 0;
    bytes = 0;
    next_seq = 0;
    window_bytes = 0;
    n_windows = 0;
    windows_full = false;
    link_down = false;
    disconnects = 0;
    down_ms = 0;
    longest_down_ms = 0;
    k_spin_unlock(&lock, key);

    uint8_t cmd = EP_CMD_START_BLE_LINK_TEST;
    int ret = ble_central_send(&cmd, sizeof(cmd));
    if (ret) {
        atomic_clear(&active);
        return ret;
    }
    return 0;
}

int link_test_stop(struct link_test_stats *stats) {
    if (!atomic_cas(&active, 1, 0)) {
        return -EALREADY;
    }

    int64_t now = k_uptime_get();

    k_spinlock_key_t key = k_spin_lock(&lock);
    k_timer_stop(&window_timer);
    streaming = false;
    end_ms = now;
    end_downtime(now);
    k_spin_unlock(&lock, key);

    uint8_t cmd = EP_CMD_STOP_BLE_LINK_TEST;
    int ret = ble_central_send(&cmd, sizeof(cmd));

    link_test_get_stats(stats);
    return ret;
}

// -- STATS --

// windows are copied here so they can be sorted without holding the lock
static uint32_t scratch[LINK_TEST_MAX_WINDOWS];
static K_MUTEX_DEFINE(scratch_mutex);

static int cmp_u32(const void *a, const void *b) {
    uint32_t x = *(const uint32_t *)a;
    uint32_t y = *(const uint32_t *)b;
    return (x > y) - (x < y);
}

static uint32_t window_to_bps(uint64_t window_bytes_sum, uint32_t n) {
    return (uint32_t)(window_bytes_sum * 1000 / ((uint64_t)n * LINK_TEST_WINDOW_MS));
}

// lowest average over any span of 'span' consecutive windows, or 0 if too few
static uint32_t min_rolling_bps(const uint32_t *w, uint32_t n, uint32_t span) {
    if (n < span) {
        return 0;
    }

    uint64_t sum = 0;
    for (uint32_t i = 0; i < span; i++) {
        sum += w[i];
    }

    uint64_t min_sum = sum;
    for (uint32_t i = span; i < n; i++) {
        sum += w[i];
        sum -= w[i - span];
        min_sum = MIN(min_sum, sum);
    }
    return window_to_bps(min_sum, span);
}

// nearest-rank percentile of an ascending array
static uint32_t percentile_bps(const uint32_t *sorted, uint32_t n, uint32_t pct) {
    return window_to_bps(sorted[(uint64_t)(n - 1) * pct / 100], 1);
}

void link_test_get_stats(struct link_test_stats *stats) {
    memset(stats, 0, sizeof(*stats));

    k_mutex_lock(&scratch_mutex, K_FOREVER);

    int64_t now = k_uptime_get();
    bool running = atomic_get(&active);

    k_spinlock_key_t key = k_spin_lock(&lock);
    stats->running = running;
    if (got_first) {
        stats->elapsed_ms = (uint32_t)((running ? now : end_ms) - first_ms);
    }
    stats->received = received;
    stats->dropped = dropped;
    stats->bytes = bytes;
    stats->windows_full = windows_full;
    stats->disconnects = disconnects;
    stats->down_ms = down_ms;
    stats->longest_down_ms = longest_down_ms;
    if (link_down) {
        // still disconnected: include the outage so far
        uint32_t cur = (uint32_t)(now - down_since);
        stats->down_ms += cur;
        stats->longest_down_ms = MAX(stats->longest_down_ms, cur);
    }
    uint32_t n = n_windows;
    memcpy(scratch, windows, n * sizeof(scratch[0]));
    k_spin_unlock(&lock, key);

    stats->windows = n;
    if (n > 0) {
        uint64_t sum = 0;
        for (uint32_t i = 0; i < n; i++) {
            sum += scratch[i];
        }
        stats->mean_bps = window_to_bps(sum, n);

        // rolling minima need the windows in time order, so before sorting
        stats->min_5s_bps = min_rolling_bps(scratch, n, 5000 / LINK_TEST_WINDOW_MS);
        stats->min_10s_bps = min_rolling_bps(scratch, n, 10000 / LINK_TEST_WINDOW_MS);

        qsort(scratch, n, sizeof(scratch[0]), cmp_u32);
        stats->min_bps = percentile_bps(scratch, n, 0);
        stats->p5_bps = percentile_bps(scratch, n, 5);
        stats->p10_bps = percentile_bps(scratch, n, 10);
        stats->p50_bps = percentile_bps(scratch, n, 50);
        stats->max_bps = percentile_bps(scratch, n, 100);
    }

    k_mutex_unlock(&scratch_mutex);
}
