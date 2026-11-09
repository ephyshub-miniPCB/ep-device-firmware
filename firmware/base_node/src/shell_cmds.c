#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>

#include "ble_central.h"
#include "ep_cmd.h"
#include "ep_info.h"
#include "ep_request.h"
#include "link_test.h"
#include "usb_stream.h"

/* Shell commands for driving the base node (and through it, the ep_device)
 * from a PC over the console. Picking the ep_device:
 *   scan [seconds]
 *   connect <index | address [random|public]>
 *   disconnect
 * Everything else is grouped by verb:
 *   start <mode>
 *   stop <mode>
 *   status <mode>
 *   list <thing>    (read-only information about the ep_device)
 * Add new modes as subcommands of the verbs below. */

// -- EP_DEVICE CONNECTION --

#define SCAN_DEFAULT_S      3
#define SCAN_MAX_S          30
#define SCAN_MAX_RESULTS    16
// covers the create connection timeout (3 s), MTU exchange and discovery
#define CONNECT_TIMEOUT_MS  8000
#define DISCONNECT_TIMEOUT_MS 2000
#define POLL_MS             50

// results of the last scan, so "connect <index>" can refer to them
static struct ble_central_device scan_results[SCAN_MAX_RESULTS];
static size_t scan_n;

static void print_device(const struct shell *sh, const char *prefix,
                         const struct ble_central_device *dev) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&dev->addr, addr, sizeof(addr));
    shell_print(sh, "%s%s %s", prefix, dev->name[0] ? dev->name : "(no name)", addr);
}

static int cmd_scan(const struct shell *sh, size_t argc, char **argv) {
    uint32_t seconds = SCAN_DEFAULT_S;
    if (argc > 1) {
        char *end;
        seconds = strtoul(argv[1], &end, 0);
        if (*end != '\0' || seconds == 0 || seconds > SCAN_MAX_S) {
            shell_error(sh, "scan time must be 1-%u seconds", SCAN_MAX_S);
            return -EINVAL;
        }
    }

    shell_print(sh, "scanning for %u s...", seconds);
    int n = ble_central_scan(K_SECONDS(seconds), scan_results, ARRAY_SIZE(scan_results));
    if (n < 0) {
        scan_n = 0;
        shell_error(sh, "scan failed: %d%s", n,
                    n == -EAGAIN || n == -EBUSY ? " (busy connecting, try again)" : "");
        return n;
    }

    scan_n = n;
    shell_print(sh, "found %d ep_device%s", n, n == 1 ? "" : "s");
    for (int i = 0; i < n; i++) {
        char addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&scan_results[i].addr, addr, sizeof(addr));
        shell_print(sh, "  %2d  %-16s %s  %d dBm", i,
                    scan_results[i].name[0] ? scan_results[i].name : "(no name)", addr,
                    scan_results[i].rssi);
    }
    if (n) {
        shell_print(sh, "connect with \"connect <index>\"");
    }
    return 0;
}

static int cmd_connect(const struct shell *sh, size_t argc, char **argv) {
    struct ble_central_device dev = { 0 };
    struct ble_central_status st;

    ble_central_get_status(&st);
    if (st.state != BLE_CENTRAL_IDLE) {
        print_device(sh, st.state == BLE_CENTRAL_CONNECTED ? "already connected to " :
                     "already connecting to ", &st.target);
        shell_error(sh, "disconnect first");
        return -EALREADY;
    }

    char *end;
    unsigned long idx = strtoul(argv[1], &end, 10);
    if (*argv[1] != '\0' && *end == '\0') {
        if (idx >= scan_n) {
            shell_error(sh, "no device %lu in the last scan (run \"scan\" first)", idx);
            return -EINVAL;
        }
        dev = scan_results[idx];
    } else {
        const char *type = argc > 2 ? argv[2] : "random";
        if (bt_addr_le_from_str(argv[1], type, &dev.addr)) {
            shell_error(sh, "bad address: %s %s (expected e.g. C0:11:22:33:44:55 random)",
                        argv[1], type);
            return -EINVAL;
        }
        // keep the name if the address was in the last scan
        for (size_t i = 0; i < scan_n; i++) {
            if (bt_addr_le_eq(&scan_results[i].addr, &dev.addr)) {
                dev = scan_results[i];
            }
        }
    }

    int ret = ble_central_connect(&dev);
    if (ret) {
        shell_error(sh, "failed to start connecting: %d", ret);
        return ret;
    }
    print_device(sh, "connecting to ", &dev);

    for (int waited = 0; waited < CONNECT_TIMEOUT_MS; waited += POLL_MS) {
        k_msleep(POLL_MS);
        ble_central_get_status(&st);
        if (st.state == BLE_CENTRAL_CONNECTED) {
            print_device(sh, "connected to ", &dev);
            return 0;
        }
        if (st.state == BLE_CENTRAL_IDLE) {
            shell_error(sh, "connection failed (reason 0x%02x)%s", st.last_err,
                        st.last_err == BT_HCI_ERR_UNKNOWN_CONN_ID ?
                        ": no answer, is the ep_device on and advertising?" : "");
            return -ENOTCONN;
        }
    }

    (void)ble_central_disconnect();
    shell_error(sh, "connection timed out");
    return -ETIMEDOUT;
}

static int cmd_disconnect(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int ret = ble_central_disconnect();
    if (ret == -ENOTCONN) {
        shell_warn(sh, "not connected");
        return 0;
    }
    if (ret) {
        shell_error(sh, "disconnect failed: %d", ret);
        return ret;
    }

    struct ble_central_status st;
    for (int waited = 0; waited < DISCONNECT_TIMEOUT_MS; waited += POLL_MS) {
        ble_central_get_status(&st);
        if (st.state == BLE_CENTRAL_IDLE) {
            shell_print(sh, "disconnected");
            return 0;
        }
        k_msleep(POLL_MS);
    }
    shell_error(sh, "still connected after %u ms", DISCONNECT_TIMEOUT_MS);
    return -ETIMEDOUT;
}

static int cmd_status_connection(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct ble_central_status st;
    ble_central_get_status(&st);

    switch (st.state) {
    case BLE_CENTRAL_CONNECTED:
        print_device(sh, "connected to ", &st.target);
        break;
    case BLE_CENTRAL_CONNECTING:
        print_device(sh, "connecting to ", &st.target);
        break;
    case BLE_CENTRAL_RECONNECTING:
        print_device(sh, "reconnecting to ", &st.target);
        shell_warn(sh, "  link lost (reason 0x%02x), retrying", st.last_err);
        break;
    default:
        shell_print(sh, "not connected");
        break;
    }
    return 0;
}

// -- LINK TEST --

// prints a payload rate as kB/s and kbit/s (integer maths, no float printf needed)
static void print_rate(const struct shell *sh, const char *label, uint32_t bps) {
    shell_print(sh, "  %-14s %6u.%u kB/s  (%u kbit/s)", label,
                bps / 1000, (bps % 1000) / 100, (uint32_t)((uint64_t)bps * 8 / 1000));
}

static void print_link_test_stats(const struct shell *sh, const struct link_test_stats *s) {
    uint32_t expected = s->received + s->dropped;
    uint32_t loss_x100 = expected ? (uint32_t)((uint64_t)s->dropped * 10000 / expected) : 0;
    uint32_t pps_x100 = s->elapsed_ms ?
        (uint32_t)((uint64_t)s->received * 100000 / s->elapsed_ms) : 0;

    shell_print(sh, "link test %s, %u.%u s since first packet",
                s->running ? "running" : "stopped",
                s->elapsed_ms / 1000, (s->elapsed_ms % 1000) / 100);
    shell_print(sh, "  packets:       %u received, %u dropped (%u.%02u %% loss)",
                s->received, s->dropped, loss_x100 / 100, loss_x100 % 100);
    shell_print(sh, "  packet rate:   %u.%02u packets/s", pps_x100 / 100, pps_x100 % 100);

    if (s->windows == 0) {
        shell_print(sh, "  throughput:    no complete %u ms windows yet", LINK_TEST_WINDOW_MS);
    } else {
        shell_print(sh, "  throughput over %u x %u ms windows%s:", s->windows,
                    LINK_TEST_WINDOW_MS, s->windows_full ? " (window buffer full)" : "");
        print_rate(sh, "mean", s->mean_bps);
        print_rate(sh, "max", s->max_bps);
        print_rate(sh, "median", s->p50_bps);
        print_rate(sh, "10th pct", s->p10_bps);
        print_rate(sh, "5th pct", s->p5_bps);
        print_rate(sh, "min", s->min_bps);
        if (s->min_5s_bps) {
            print_rate(sh, "worst 5 s avg", s->min_5s_bps);
        }
        if (s->min_10s_bps) {
            print_rate(sh, "worst 10 s avg", s->min_10s_bps);
        }
    }

    shell_print(sh, "  disconnects:   %u (%u ms down in total, longest %u ms)",
                s->disconnects, s->down_ms, s->longest_down_ms);
}

static int cmd_start_link_test(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int ret = link_test_start();
    if (ret == -EALREADY) {
        shell_warn(sh, "link test already running");
        return ret;
    }
    if (ret) {
        shell_error(sh, "failed to start link test: %d%s", ret,
                    ret == -ENOTCONN ? " (ep_device not connected)" : "");
        return ret;
    }

    shell_print(sh, "link test started");
    return 0;
}

static int cmd_stop_link_test(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct link_test_stats stats;
    int ret = link_test_stop(&stats);
    if (ret == -EALREADY) {
        shell_warn(sh, "link test not running");
        return ret;
    }
    if (ret) {
        shell_warn(sh, "failed to send stop command to ep_device: %d", ret);
    }

    print_link_test_stats(sh, &stats);
    return 0;
}

static int cmd_status_link_test(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct link_test_stats stats;
    link_test_get_stats(&stats);
    print_link_test_stats(sh, &stats);
    return 0;
}

// -- USB DATA STREAM --

static int cmd_status_stream(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct usb_stream_stats s;
    usb_stream_get_stats(&s);

    shell_print(sh, "data stream port %s", s.port_open ? "open" : "closed (no DTR)");
    shell_print(sh, "  frames:        %u sent, %u dropped (buffer full), %u discarded (port closed)",
                s.sent, s.dropped, s.discarded);
    shell_print(sh, "  buffer:        %u / %u bytes", s.buf_used, s.buf_size);
    return 0;
}

// -- EP_DEVICE INFO --

#define EP_INFO_TIMEOUT K_MSEC(1000)

static void print_rhs2116_info(const struct shell *sh, const struct ep_info *info) {
    switch (info->rhs2116_state) {
    case EP_INFO_RHS2116_EMULATED:
        shell_print(sh, "  EMULATED (no chip fitted, data is synthetic)");
        //__fallthrough;
    case EP_INFO_RHS2116_OK:
        shell_print(sh, "  chip id:       0x%02x", info->rhs2116_chip_id);
        shell_print(sh, "  die revision:  %u", info->rhs2116_die_revision);
        shell_print(sh, "  channels:      %u", info->rhs2116_num_channels);
        break;
    case EP_INFO_RHS2116_ERROR:
        shell_print(sh, "  unknown (query failed: %d)", info->rhs2116_error);
        break;
    case EP_INFO_RHS2116_UNKNOWN:
        shell_print(sh, "  unknown (not queried yet)");
        break;
    default:
        shell_print(sh, "  unknown (state %u)", info->rhs2116_state);
        break;
    }
}

static int cmd_list_device(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct ep_info info;
    int ret = ep_info_query(&info, EP_INFO_TIMEOUT);
    if (ret) {
        shell_error(sh, "failed to get ep_device info: %d%s", ret,
                    ret == -ENOTCONN ? " (ep_device not connected)" :
                    ret == -ETIMEDOUT ? " (no reply)" : "");
        return ret;
    }

    shell_print(sh, "ep_device \"%s\"", info.name);
    shell_print(sh, "  firmware:      %u.%u.%u (build %s)",
                info.fw_major, info.fw_minor, info.fw_patch, info.build);
    shell_print(sh, "  zephyr:        %s", info.zephyr);
    shell_print(sh, "  hardware:      rev %u", info.hw_revision);
    shell_print(sh, "rhs2116");
    print_rhs2116_info(sh, &info);
    return 0;
}

// -- RECORDING --

#define EP_REQ_TIMEOUT      K_MSEC(1000)
// stopping waits for any stimulation to wind down on the ep_device first
#define EP_STOP_TIMEOUT     K_MSEC(2500)

static const char *rec_reject_str(uint8_t r) {
    switch (r) {
    case EP_REC_OK:            return "ok";
    case EP_REC_BAD_MASK:      return "no channels in mask";
    case EP_REC_RATE_TOO_LOW:  return "rate below minimum";
    case EP_REC_RATE_TOO_HIGH: return "rate exceeds SPI bandwidth for this channel count";
    case EP_REC_BUSY:          return "already recording";
    case EP_REC_BAD_BITS:      return "bits must be 8, 10, 12 or 16";
    default:                   return "unknown";
    }
}

static const char *stim_reject_str(uint8_t r) {
    switch (r) {
    case EP_STIM_OK:               return "ok";
    case EP_STIM_NOT_RECORDING:    return "start recording first (stim is timed by the stream)";
    case EP_STIM_BUSY:             return "stimulation already running";
    case EP_STIM_BAD_MASK:         return "bad channel mask";
    case EP_STIM_TOO_SHORT:        return "a phase rounds to zero frames at this sample rate";
    case EP_STIM_PERIOD_TOO_SHORT: return "period shorter than the pulse";
    case EP_STIM_UNBALANCED:       return "anodic and cathodic charge differ";
    case EP_STIM_BAD_EVENTS:       return "invalid event list";
    case EP_STIM_NO_PROGRAM:       return "no stim program loaded";
    case EP_STIM_RATE_CHANGED:     return "sample rate changed since the program was loaded";
    default:                       return "unknown";
    }
}

static int request_error(const struct shell *sh, const char *what, int ret) {
    shell_error(sh, "%s: %d%s", what, ret,
                ret == -ENOTCONN ? " (ep_device not connected)" :
                ret == -ETIMEDOUT ? " (no reply)" : "");
    return ret;
}

static bool parse_u32(const struct shell *sh, const char *arg, const char *name, uint32_t *out) {
    char *end;
    unsigned long v = strtoul(arg, &end, 0);
    if (*arg == '\0' || *end != '\0') {
        shell_error(sh, "bad %s: %s", name, arg);
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

/* Compares what the recording needs with the last link test. This is a soft
 * limit: the recording runs either way, but more than the link sustains means
 * dropped packets (seq gaps on the stream port). */
static void check_ble_budget(const struct shell *sh, uint32_t required, uint32_t ep_link_bps) {
    struct link_test_stats lt;
    link_test_get_stats(&lt);

    if (lt.windows == 0) {
        if (ep_link_bps) {
            shell_warn(sh, "no link test results on the base node; the ep_device measured "
                       "%u B/s last time", ep_link_bps);
            if (required > ep_link_bps) {
                shell_warn(sh, "WARNING: needs %u B/s, more than that", required);
            }
        } else {
            shell_warn(sh, "BLE capacity untested: run \"start link_test\" to check that "
                       "%u B/s fits", required);
        }
        return;
    }

    if (required > lt.mean_bps) {
        shell_warn(sh, "WARNING: needs %u B/s but the last link test averaged %u B/s; "
                   "expect continuous packet loss", required, lt.mean_bps);
    } else if (required > lt.p10_bps) {
        shell_warn(sh, "WARNING: needs %u B/s, above the link test's 10th percentile "
                   "(%u B/s); expect loss during link dips", required, lt.p10_bps);
    } else {
        shell_print(sh, "  BLE budget:    %u B/s of %u B/s (10th percentile of last link test)",
                    required, lt.p10_bps);
    }
}

static int cmd_start_recording(const struct shell *sh, size_t argc, char **argv) {
    uint32_t mask, rate, bits;

    if (!parse_u32(sh, argv[1], "channel mask", &mask) ||
        !parse_u32(sh, argv[2], "rate", &rate) ||
        !parse_u32(sh, argv[3], "bits", &bits)) {
        return -EINVAL;
    }
    if (mask > 0xFFFF || bits > 0xFF) {
        shell_error(sh, "channel mask is 16 bits, bits is 8/10/12/16");
        return -EINVAL;
    }
    if (link_test_active()) {
        shell_error(sh, "stop the link test first (it shares the data characteristic)");
        return -EBUSY;
    }

    uint8_t payload[7];
    sys_put_le16((uint16_t)mask, &payload[0]);
    sys_put_le32(rate, &payload[2]);
    payload[6] = (uint8_t)bits;

    uint8_t r[24];
    int ret = ep_request(EP_CMD_START_RECORDING, payload, sizeof(payload), r, sizeof(r),
                         EP_REQ_TIMEOUT);
    if (ret < 0) {
        return request_error(sh, "start recording", ret);
    }
    if (ret < 22) {
        shell_error(sh, "short reply (%d bytes)", ret);
        return -EIO;
    }

    int8_t status = (int8_t)r[0];
    uint8_t reject = r[1];
    uint32_t rate_mhz = sys_get_le32(&r[2]);
    uint16_t slot_ticks = sys_get_le16(&r[6]);
    uint8_t slots = r[8];
    uint8_t fpp = r[9];
    uint32_t required = sys_get_le32(&r[10]);
    uint32_t ep_link = sys_get_le32(&r[14]);
    uint32_t max_rate = sys_get_le32(&r[18]);

    if (status) {
        shell_error(sh, "rejected: %s (%d)", rec_reject_str(reject), status);
        if (reject == EP_REC_RATE_TOO_HIGH) {
            shell_error(sh, "  max %u S/s per channel for %u channels", max_rate,
                        __builtin_popcount(mask));
        }
        return status;
    }

    shell_print(sh, "recording 0x%04x at %u.%03u S/s/ch, %u bits", mask,
                rate_mhz / 1000, rate_mhz % 1000, bits);
    shell_print(sh, "  frame:         %u slots x %u ticks (%u.%02u us/slot)", slots, slot_ticks,
                slot_ticks / 16, (slot_ticks % 16) * 100 / 16);
    shell_print(sh, "  packets:       %u frames each", fpp);
    shell_print(sh, "  SPI limit:     %u S/s/ch at this channel count", max_rate);
    check_ble_budget(sh, required, ep_link);
    return 0;
}

static int cmd_stop_recording(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uint8_t r[4];
    int ret = ep_request(EP_CMD_STOP_RECORDING, NULL, 0, r, sizeof(r), EP_STOP_TIMEOUT);
    if (ret < 0) {
        return request_error(sh, "stop recording", ret);
    }
    shell_print(sh, "recording stopped");
    return 0;
}

static int cmd_status_recording(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uint8_t r[40];
    int ret = ep_request(EP_CMD_GET_RECORDING_STATS, NULL, 0, r, sizeof(r), EP_REQ_TIMEOUT);
    if (ret < 0) {
        return request_error(sh, "recording status", ret);
    }
    if (ret < 39) {
        shell_error(sh, "short reply (%d bytes)", ret);
        return -EIO;
    }

    shell_print(sh, "recording %s, stimulation %s", r[1] ? "running" : "stopped",
                r[2] ? "running" : "stopped");
    shell_print(sh, "  frames:        %u (%u blocks)", sys_get_le32(&r[3]), sys_get_le32(&r[7]));
    shell_print(sh, "  spi errors:    %u", sys_get_le32(&r[11]));
    shell_print(sh, "  compliance:    %u events", sys_get_le32(&r[15]));
    shell_print(sh, "  ring rewinds:  %u (%u late, max %u guard words used)",
                sys_get_le32(&r[19]), sys_get_le32(&r[23]), sys_get_le32(&r[27]));
    shell_print(sh, "  ble packets:   %u sent, %u dropped on the ep_device",
                sys_get_le32(&r[31]), sys_get_le32(&r[35]));
    return 0;
}

// -- STIMULATION --

static int cmd_stim_biphasic(const struct shell *sh, size_t argc, char **argv) {
    static const char *const names[] = {
        "channel mask", "cathodic amplitude", "anodic amplitude", "phase 1 us",
        "interphase us", "phase 2 us", "recovery us", "period us", "count",
    };
    uint32_t v[ARRAY_SIZE(names)];

    for (size_t i = 0; i < ARRAY_SIZE(names); i++) {
        if (!parse_u32(sh, argv[i + 1], names[i], &v[i])) {
            return -EINVAL;
        }
    }

    uint8_t flags = 0;
    for (size_t i = ARRAY_SIZE(names) + 1; i < argc; i++) {
        if (strcmp(argv[i], "anodic") == 0) {
            flags |= EP_STIM_FLAG_ANODIC_FIRST;
        } else if (strcmp(argv[i], "unbalanced") == 0) {
            flags |= EP_STIM_FLAG_ALLOW_IMBALANCE;
        } else {
            shell_error(sh, "unknown option %s", argv[i]);
            return -EINVAL;
        }
    }
    if (v[0] > 0xFFFF || v[1] > 0xFF || v[2] > 0xFF || v[3] > 0xFFFF || v[4] > 0xFFFF ||
        v[5] > 0xFFFF || v[6] > 0xFFFF || v[8] > 0xFFFF) {
        shell_error(sh, "value out of range (amplitudes 0-255 steps, times 0-65535 us)");
        return -EINVAL;
    }

    uint8_t p[19];
    sys_put_le16((uint16_t)v[0], &p[0]);
    p[2] = (uint8_t)v[1];
    p[3] = (uint8_t)v[2];
    p[4] = flags;
    sys_put_le16((uint16_t)v[3], &p[5]);
    sys_put_le16((uint16_t)v[4], &p[7]);
    sys_put_le16((uint16_t)v[5], &p[9]);
    sys_put_le16((uint16_t)v[6], &p[11]);
    sys_put_le32(v[7], &p[13]);
    sys_put_le16((uint16_t)v[8], &p[17]);

    uint8_t r[16];
    int ret = ep_request(EP_CMD_STIM_LOAD_BIPHASIC, p, sizeof(p), r, sizeof(r), EP_REQ_TIMEOUT);
    if (ret < 0) {
        return request_error(sh, "load stim", ret);
    }
    if (ret < 14) {
        shell_error(sh, "short reply (%d bytes)", ret);
        return -EIO;
    }
    if ((int8_t)r[0]) {
        shell_error(sh, "rejected: %s (%d)", stim_reject_str(r[1]), (int8_t)r[0]);
        if (r[1] == EP_STIM_UNBALANCED) {
            shell_error(sh, "  (phases are rounded to whole frames; add \"unbalanced\" to override)");
        }
        return (int8_t)r[0];
    }

    shell_print(sh, "stim program loaded, in frames: phase1 %u, interphase %u, phase2 %u, "
                "recovery %u, period %u", sys_get_le16(&r[2]), sys_get_le16(&r[4]),
                sys_get_le16(&r[6]), sys_get_le16(&r[8]), sys_get_le32(&r[10]));
    shell_print(sh, "run it with \"start stim\"");
    return 0;
}

static int cmd_start_stim(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uint8_t r[4];
    int ret = ep_request(EP_CMD_START_STIMULATION, NULL, 0, r, sizeof(r), EP_REQ_TIMEOUT);
    if (ret < 0) {
        return request_error(sh, "start stim", ret);
    }
    if (ret < 2) {
        shell_error(sh, "short reply (%d bytes)", ret);
        return -EIO;
    }
    if ((int8_t)r[0]) {
        shell_error(sh, "rejected: %s (%d)", stim_reject_str(r[1]), (int8_t)r[0]);
        return (int8_t)r[0];
    }
    shell_print(sh, "stimulation started");
    return 0;
}

static int cmd_stop_stim(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uint8_t r[4];
    int ret = ep_request(EP_CMD_STOP_STIMULATION, NULL, 0, r, sizeof(r), EP_STOP_TIMEOUT);
    if (ret < 0) {
        return request_error(sh, "stop stim", ret);
    }
    shell_print(sh, "stimulation stopped");
    return 0;
}

// -- COMMAND TREE --

SHELL_STATIC_SUBCMD_SET_CREATE(sub_start,
    SHELL_CMD(link_test, NULL, "Stream dummy packets from the ep_device and measure throughput",
              cmd_start_link_test),
    SHELL_CMD_ARG(recording, NULL,
                  "Record from the RHS2116: <channel mask> <rate S/s per channel> <bits 8|10|12|16>\n"
                  "e.g. start recording 0x0001 30000 16",
                  cmd_start_recording, 4, 0),
    SHELL_CMD(stim, NULL, "Run the loaded stim program (recording must be running)",
              cmd_start_stim),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_stop,
    SHELL_CMD(link_test, NULL, "Stop the link test and print its results",
              cmd_stop_link_test),
    SHELL_CMD(recording, NULL, "Stop recording (and any stimulation)", cmd_stop_recording),
    SHELL_CMD(stim, NULL, "Stop stimulation (safe shutdown with charge recovery)",
              cmd_stop_stim),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_status,
    SHELL_CMD(connection, NULL, "Show which ep_device the base node is connected to",
              cmd_status_connection),
    SHELL_CMD(link_test, NULL, "Print results of the running (or last) link test",
              cmd_status_link_test),
    SHELL_CMD(stream, NULL, "Print USB data stream port counters", cmd_status_stream),
    SHELL_CMD(recording, NULL, "Print ep_device recording/stim counters", cmd_status_recording),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_list,
    SHELL_CMD(device, NULL, "Show ep_device firmware/hardware and RHS2116 chip info",
              cmd_list_device),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_stim,
    SHELL_CMD_ARG(biphasic, NULL,
                  "Load a biphasic pulse train: <channel mask> <cathodic amp> <anodic amp> "
                  "<phase1 us> <interphase us> <phase2 us> <recovery us> <period us> "
                  "<count, 0 = until stopped> [anodic] [unbalanced]\n"
                  "amplitudes are in stim steps (1 uA by default); cathodic phase first "
                  "unless \"anodic\"",
                  cmd_stim_biphasic, 10, 2),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_ARG_REGISTER(scan, NULL, "Scan for ep_devices: [seconds, default 3]", cmd_scan, 1, 1);
SHELL_CMD_ARG_REGISTER(connect, NULL,
                       "Connect to an ep_device: <index from the last scan> or "
                       "<address> [random|public]\n"
                       "a lost link is retried until \"disconnect\"",
                       cmd_connect, 2, 1);
SHELL_CMD_REGISTER(disconnect, NULL, "Disconnect from the ep_device and stop retrying",
                   cmd_disconnect);
SHELL_CMD_REGISTER(stim, &sub_stim, "Configure stimulation", NULL);
SHELL_CMD_REGISTER(start, &sub_start, "Start a mode", NULL);
SHELL_CMD_REGISTER(stop, &sub_stop, "Stop a mode", NULL);
SHELL_CMD_REGISTER(status, &sub_status, "Show the state of a mode", NULL);
SHELL_CMD_REGISTER(list, &sub_list, "Show information about the ep_device", NULL);
