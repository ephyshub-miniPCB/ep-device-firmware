#ifndef LINK_TEST_H
#define LINK_TEST_H

#include <stdbool.h>
#include <stdint.h>

/* Throughput is sampled in fixed windows while the ep_device is streaming, so
 * the dips the link goes through (e.g. as an implanted animal moves around) show
 * up in the low percentiles instead of being averaged away. */
#define LINK_TEST_WINDOW_MS   1000
/* One hour of 1 s windows; later windows still count towards the totals. */
#define LINK_TEST_MAX_WINDOWS 3600

/* All rates are payload bytes per second, measured per window. Rates are 0
 * when there aren't enough windows to compute them. */
struct link_test_stats {
    bool running;
    uint32_t elapsed_ms;      /* first packet until now (or until stop) */

    uint32_t received;        /* packets */
    uint32_t dropped;         /* sequence-number gaps while connected */
    uint64_t bytes;

    uint32_t windows;         /* complete windows recorded */
    bool windows_full;        /* hit LINK_TEST_MAX_WINDOWS */
    uint32_t mean_bps;
    uint32_t min_bps;
    uint32_t p5_bps;
    uint32_t p10_bps;
    uint32_t p50_bps;
    uint32_t max_bps;
    /* lowest rolling average over 5 / 10 consecutive windows: the worst dip an
     * ep_device buffer holding that many seconds of data would have to ride out */
    uint32_t min_5s_bps;
    uint32_t min_10s_bps;

    uint32_t disconnects;
    uint32_t down_ms;         /* total time from disconnect to streaming again */
    uint32_t longest_down_ms;
};

/* Resets the stats and tells the ep_device to start streaming link test
 * packets. Returns -EALREADY if running, or the BLE send error. */
int link_test_start(void);

/* Stops the test, tells the ep_device to stop streaming and fills *stats.
 * Returns -EALREADY if no test was running; a failed stop command is returned
 * as its error, but *stats is still filled in. */
int link_test_stop(struct link_test_stats *stats);

/* Stats of the running test, or of the last one if none is running. */
void link_test_get_stats(struct link_test_stats *stats);

bool link_test_active(void);

/* Hooks for the BLE central (called from the BT stack's context). */
void link_test_on_packet(const uint8_t *data, uint16_t len);
void link_test_on_ready(void);
void link_test_on_disconnect(void);

#endif /* LINK_TEST_H */
