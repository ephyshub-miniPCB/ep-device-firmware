#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

#include <stdint.h>

/*
 * Command packet format (base node -> ep_device, over NUS RX):
 *
 *   byte 0      opcode   (enum cmd_opcode)
 *   byte 1..n   payload  (opcode specific, multi-byte fields little-endian)
 *
 * One NUS write carries exactly one command. The payload length is whatever
 * follows the opcode, and is checked against the opcode's table entry in
 * cmd_handler.c before the handler runs.
 *
 * Commands that return data reply over NUS TX with the same layout:
 * [opcode of the command][payload...].
 */

/* Largest command accepted: one ATT MTU (247) minus the 3-byte ATT header. */
#define CMD_MAX_LEN 244

enum cmd_opcode {
    CMD_PING               = 0x01,
    CMD_GET_INFO           = 0x02, /* replies with device_info.h's payload */

    /* recording / stimulation control, payloads and replies below */
    CMD_START_RECORDING    = 0x10,
    CMD_STOP_RECORDING     = 0x11,
    CMD_START_STIMULATION  = 0x12,
    CMD_STOP_STIMULATION   = 0x13,
    CMD_STIM_LOAD_BIPHASIC = 0x14,
    CMD_STIM_LOAD_EVENTS   = 0x15,
    CMD_GET_RECORDING_STATS = 0x16,

    /* electrode setup / parameters */
    CMD_SET_ELECTRODE_MASK = 0x20,

    /* diagnostics */
    CMD_START_BLE_LINK_TEST = 0x30,
    CMD_STOP_BLE_LINK_TEST  = 0x31,
};

/*
 * Recording / stimulation payloads (all little-endian). Every one of these
 * commands replies [opcode][status s8 (0 or -errno)]... as listed.
 *
 * START_RECORDING   payload: chan_mask u16, rate_hz u32, bits u8
 *                   reply:   status, reject u8 (enum rec_reject), rate_mhz u32,
 *                            slot_ticks u16, slots_per_frame u8, frames_per_packet u8,
 *                            required_bps u32, link_bps u32, max_rate_hz u32
 * STOP_RECORDING    reply:   status (also stops any stimulation first)
 * START_STIMULATION reply:   status, reject u8 (enum stim_reject)
 * STOP_STIMULATION  reply:   status
 * STIM_LOAD_BIPHASIC payload: chan_mask u16, neg_mag u8, pos_mag u8, flags u8,
 *                            phase1_us u16, interphase_us u16, phase2_us u16,
 *                            recovery_us u16, period_us u32, count u16
 *                   reply:   status, reject u8, phase1 u16, interphase u16, phase2 u16,
 *                            recovery u16, period u32 (all in frames)
 * STIM_LOAD_EVENTS  payload: chan_mask u16, neg_mag u8, pos_mag u8, flags u8,
 *                            period_frames u32, count u16, recovery_frames u16, n u8,
 *                            then n events of: frame u32, on u16, pol u16, n_extra u8,
 *                            reg u8, val u16, reg u8, val u16
 *                   reply:   status, reject u8
 * GET_RECORDING_STATS reply: status, recording u8, stim u8, frames u32, blocks u32,
 *                            spi_errors u32, compliance_events u32, rewinds u32,
 *                            late_rewinds u32, max_guard_used u32, packets_sent u32,
 *                            packets_dropped u32
 */
#define CMD_STIM_EVENT_LEN 15

/* Queues a received command for processing on the command thread. Safe to call
 * from the BLE RX callback: it only copies the data and never blocks.
 * Returns -EINVAL for empty/oversized packets, -ENOMEM if the queue is full. */
int cmd_handler_submit(const uint8_t *data, uint16_t len);

/* Queues a STOP_RECORDING (which also stops stimulation), e.g. on disconnect. */
void cmd_handler_stop_all(void);

#endif /* CMD_HANDLER_H */
