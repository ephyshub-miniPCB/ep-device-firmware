#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ble_handler.h"
#include "cmd_handler.h"
#include "device_info.h"
#include "recording.h"
#include "stim.h"

LOG_MODULE_REGISTER(cmd_handler, CONFIG_LOG_DEFAULT_LEVEL);

#define CMD_QUEUE_DEPTH      4
#define CMD_THREAD_STACK     2048
#define CMD_THREAD_PRIORITY  7

struct cmd_packet {
    uint16_t len;
    uint8_t data[CMD_MAX_LEN];
};

K_MSGQ_DEFINE(cmd_msgq, sizeof(struct cmd_packet), CMD_QUEUE_DEPTH, 4);

/* Handlers run on the command thread (not the BLE stack), so they may block,
 * e.g. on SPI transfers to the RHS2116. Return 0 on success or a negative errno. */
typedef int (*cmd_fn_t)(const uint8_t *payload, uint16_t len);

struct cmd_entry {
    uint8_t opcode;
    const char *name;
    uint16_t min_len; /* payload bytes, excluding the opcode */
    uint16_t max_len;
    cmd_fn_t fn;
};

static const struct device *const rhs2116_dev = DEVICE_DT_GET(DT_NODELABEL(ep_dev));

// -- COMMAND HANDLERS --

// sends [opcode][status][body...] over NUS
static int reply(uint8_t opcode, int status, const uint8_t *body, uint16_t body_len) {
    uint8_t buf[2 + 48];

    __ASSERT_NO_MSG(body_len <= sizeof(buf) - 2);
    buf[0] = opcode;
    buf[1] = (uint8_t)(int8_t)status;
    if (body_len) {
        memcpy(&buf[2], body, body_len);
    }
    (void)ble_handler_send(buf, body_len + 2);
    return status;
}

static int handle_ping(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(payload);
    ARG_UNUSED(len);
    LOG_INF("ping");
    return 0;
}

static int handle_get_info(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(payload);
    ARG_UNUSED(len);

    uint8_t reply[CMD_MAX_LEN];
    reply[0] = CMD_GET_INFO;

    int ret = device_info_encode(&reply[1], sizeof(reply) - 1);
    if (ret < 0) {
        return ret;
    }
    return ble_handler_send(reply, ret + 1);
}

static int handle_start_recording(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(len);

    struct rec_params params = {
        .chan_mask = sys_get_le16(&payload[0]),
        .rate_hz = sys_get_le32(&payload[2]),
        .bits = payload[6],
    };
    struct rec_info info;

    int ret = recording_start(rhs2116_dev, &params, &info);

    uint8_t body[22];
    body[0] = info.reject;
    sys_put_le32(info.rate_mhz, &body[1]);
    sys_put_le16((uint16_t)info.slot_ticks, &body[5]);
    body[7] = info.slots_per_frame;
    body[8] = info.frames_per_packet;
    sys_put_le32(info.required_bps, &body[9]);
    sys_put_le32(info.link_bps, &body[13]);
    sys_put_le32(info.max_rate_hz, &body[17]);
    return reply(CMD_START_RECORDING, ret, body, 21);
}

static int handle_stop_recording(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(payload);
    ARG_UNUSED(len);

    // stimulation is timed by the stream, so it has to wind down first
    stim_stop();
    recording_stop();
    return reply(CMD_STOP_RECORDING, 0, NULL, 0);
}

static int handle_start_stimulation(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(payload);
    ARG_UNUSED(len);

    enum stim_reject why;
    int ret = stim_start(rhs2116_dev, &why);
    uint8_t body[1] = { why };
    return reply(CMD_START_STIMULATION, ret, body, sizeof(body));
}

static int handle_stop_stimulation(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(payload);
    ARG_UNUSED(len);

    stim_stop();
    return reply(CMD_STOP_STIMULATION, 0, NULL, 0);
}

static int handle_stim_load_biphasic(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(len);

    struct stim_biphasic b = {
        .chan_mask = sys_get_le16(&payload[0]),
        .neg_mag = payload[2],
        .pos_mag = payload[3],
        .flags = payload[4],
        .phase1_us = sys_get_le16(&payload[5]),
        .interphase_us = sys_get_le16(&payload[7]),
        .phase2_us = sys_get_le16(&payload[9]),
        .recovery_us = sys_get_le16(&payload[11]),
        .period_us = sys_get_le32(&payload[13]),
        .count = sys_get_le16(&payload[17]),
    };
    struct stim_biphasic_frames f = {0};
    enum stim_reject why;

    int ret = stim_load_biphasic(&b, &f, &why);

    uint8_t body[13];
    body[0] = why;
    sys_put_le16(f.phase1, &body[1]);
    sys_put_le16(f.interphase, &body[3]);
    sys_put_le16(f.phase2, &body[5]);
    sys_put_le16(f.recovery, &body[7]);
    sys_put_le32(f.period, &body[9]);
    return reply(CMD_STIM_LOAD_BIPHASIC, ret, body, sizeof(body));
}

#define STIM_EVENTS_HDR_LEN 14

static int handle_stim_load_events(const uint8_t *payload, uint16_t len) {
    struct stim_program p = {
        .chan_mask = sys_get_le16(&payload[0]),
        .neg_mag = payload[2],
        .pos_mag = payload[3],
        .flags = payload[4],
        .period_frames = sys_get_le32(&payload[5]),
        .count = sys_get_le16(&payload[9]),
        .recovery_frames = sys_get_le16(&payload[11]),
        .n_events = payload[13],
    };
    enum stim_reject why = STIM_BAD_EVENTS;
    int ret = -EINVAL;

    if (p.n_events <= STIM_MAX_EVENTS &&
        len == STIM_EVENTS_HDR_LEN + p.n_events * CMD_STIM_EVENT_LEN) {
        const uint8_t *e = &payload[STIM_EVENTS_HDR_LEN];
        for (uint8_t i = 0; i < p.n_events; i++, e += CMD_STIM_EVENT_LEN) {
            p.events[i] = (struct stim_event){
                .frame = sys_get_le32(&e[0]),
                .on = sys_get_le16(&e[4]),
                .pol = sys_get_le16(&e[6]),
                .n_extra = e[8],
                .extra = {
                    { .reg = e[9], .val = sys_get_le16(&e[10]) },
                    { .reg = e[12], .val = sys_get_le16(&e[13]) },
                },
            };
        }
        ret = stim_load_program(&p, &why);
    }

    uint8_t body[1] = { why };
    return reply(CMD_STIM_LOAD_EVENTS, ret, body, sizeof(body));
}

static int handle_get_recording_stats(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(payload);
    ARG_UNUSED(len);

    struct rec_stats st;
    recording_get_stats(&st);

    uint8_t body[38];
    body[0] = recording_active();
    body[1] = stim_running();
    sys_put_le32(st.stream.frames, &body[2]);
    sys_put_le32(st.stream.blocks, &body[6]);
    sys_put_le32(st.stream.spi_errors, &body[10]);
    sys_put_le32(st.stream.compliance_events, &body[14]);
    sys_put_le32(st.stream.transport.rewinds, &body[18]);
    sys_put_le32(st.stream.transport.late_rewinds, &body[22]);
    sys_put_le32(st.stream.transport.max_guard_used, &body[26]);
    sys_put_le32(st.packets_sent, &body[30]);
    sys_put_le32(st.packets_dropped, &body[34]);
    return reply(CMD_GET_RECORDING_STATS, 0, body, sizeof(body));
}

// payload: u16 channel mask, bit n == RHS2116 channel n
static int handle_set_electrode_mask(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(len);
    uint16_t mask = sys_get_le16(payload);

    LOG_INF("electrode mask 0x%04x", mask);
    // TODO: store the mask and apply it to the recording/stim channel selection
    return 0;
}

static int handle_start_ble_link_test(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(payload);
    ARG_UNUSED(len);
    return ble_handler_link_test_start();
}

static int handle_stop_ble_link_test(const uint8_t *payload, uint16_t len) {
    ARG_UNUSED(payload);
    ARG_UNUSED(len);
    ble_handler_link_test_stop();
    return 0;
}

// -- DISPATCH --

/* Add new commands here: an opcode in cmd_handler.h, a handler above, and an
 * entry below giving the accepted payload length range. */
static const struct cmd_entry cmd_table[] = {
    { CMD_PING,               "PING",               0, 0, handle_ping },
    { CMD_GET_INFO,           "GET_INFO",           0, 0, handle_get_info },
    { CMD_START_RECORDING,    "START_RECORDING",    7, 7, handle_start_recording },
    { CMD_STOP_RECORDING,     "STOP_RECORDING",     0, 0, handle_stop_recording },
    { CMD_START_STIMULATION,  "START_STIMULATION",  0, 0, handle_start_stimulation },
    { CMD_STOP_STIMULATION,   "STOP_STIMULATION",   0, 0, handle_stop_stimulation },
    { CMD_STIM_LOAD_BIPHASIC, "STIM_LOAD_BIPHASIC", 19, 19, handle_stim_load_biphasic },
    { CMD_STIM_LOAD_EVENTS,   "STIM_LOAD_EVENTS",   STIM_EVENTS_HDR_LEN,
      STIM_EVENTS_HDR_LEN + STIM_MAX_EVENTS * CMD_STIM_EVENT_LEN, handle_stim_load_events },
    { CMD_GET_RECORDING_STATS, "GET_RECORDING_STATS", 0, 0, handle_get_recording_stats },
    { CMD_SET_ELECTRODE_MASK, "SET_ELECTRODE_MASK", 2, 2, handle_set_electrode_mask },
    { CMD_START_BLE_LINK_TEST, "START_BLE_LINK_TEST", 0, 0, handle_start_ble_link_test },
    { CMD_STOP_BLE_LINK_TEST,  "STOP_BLE_LINK_TEST",  0, 0, handle_stop_ble_link_test },
};

static const struct cmd_entry *find_cmd(uint8_t opcode) {
    for (size_t i = 0; i < ARRAY_SIZE(cmd_table); i++) {
        if (cmd_table[i].opcode == opcode) {
            return &cmd_table[i];
        }
    }
    return NULL;
}

static void dispatch(const struct cmd_packet *pkt) {
    uint8_t opcode = pkt->data[0];
    const uint8_t *payload = &pkt->data[1];
    uint16_t payload_len = pkt->len - 1;

    const struct cmd_entry *cmd = find_cmd(opcode);
    if (cmd == NULL) {
        LOG_WRN("unknown opcode 0x%02x (%u byte payload)", opcode, payload_len);
        return;
    }

    if (payload_len < cmd->min_len || payload_len > cmd->max_len) {
        LOG_WRN("%s: bad payload length %u (expected %u..%u)",
                cmd->name, payload_len, cmd->min_len, cmd->max_len);
        return;
    }

    int ret = cmd->fn(payload, payload_len);
    if (ret < 0) {
        LOG_ERR("%s failed: %d", cmd->name, ret);
    } else {
        LOG_DBG("%s ok", cmd->name);
    }
}

static void cmd_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct cmd_packet pkt;

    while (1) {
        k_msgq_get(&cmd_msgq, &pkt, K_FOREVER);
        dispatch(&pkt);
    }
}

K_THREAD_DEFINE(cmd_tid, CMD_THREAD_STACK, cmd_thread, NULL, NULL, NULL,
                CMD_THREAD_PRIORITY, 0, 0);

int cmd_handler_submit(const uint8_t *data, uint16_t len) {
    if (len == 0 || len > CMD_MAX_LEN) {
        LOG_WRN("dropping command of length %u", len);
        return -EINVAL;
    }

    struct cmd_packet pkt = { .len = len };
    memcpy(pkt.data, data, len);

    // never block here: this is called from the BLE RX thread
    int ret = k_msgq_put(&cmd_msgq, &pkt, K_NO_WAIT);
    if (ret) {
        LOG_WRN("command queue full, dropping opcode 0x%02x", data[0]);
        return -ENOMEM;
    }
    return 0;
}

void cmd_handler_stop_all(void) {
    uint8_t stop = CMD_STOP_RECORDING;
    (void)cmd_handler_submit(&stop, 1);
}
