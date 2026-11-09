#ifndef EP_CMD_H
#define EP_CMD_H

/* Command opcodes understood by the ep_device. Mirrors enum cmd_opcode in
 * ep_device/src/cmd_handler.h; keep the two in sync. A command is sent as one
 * NUS write: [opcode][payload...]. */
enum ep_cmd_opcode {
    EP_CMD_PING               = 0x01,
    EP_CMD_GET_INFO           = 0x02, /* replies [opcode][info], see ep_info.h */

    EP_CMD_START_RECORDING    = 0x10,
    EP_CMD_STOP_RECORDING     = 0x11,
    EP_CMD_START_STIMULATION  = 0x12,
    EP_CMD_STOP_STIMULATION   = 0x13,
    EP_CMD_STIM_LOAD_BIPHASIC = 0x14,
    EP_CMD_STIM_LOAD_EVENTS   = 0x15,
    EP_CMD_GET_RECORDING_STATS = 0x16,

    EP_CMD_SET_ELECTRODE_MASK = 0x20,

    EP_CMD_START_BLE_LINK_TEST = 0x30,
    EP_CMD_STOP_BLE_LINK_TEST  = 0x31,
};

/* Recording / stimulation payloads and replies are documented in
 * ep_device/src/cmd_handler.h; reject codes mirror ep_device's enum rec_reject
 * (recording.h) and enum stim_reject (stim.h). */
enum ep_rec_reject {
    EP_REC_OK = 0,
    EP_REC_BAD_MASK,
    EP_REC_RATE_TOO_LOW,
    EP_REC_RATE_TOO_HIGH,
    EP_REC_BUSY,
    EP_REC_BAD_BITS,
};

enum ep_stim_reject {
    EP_STIM_OK = 0,
    EP_STIM_NOT_RECORDING,
    EP_STIM_BUSY,
    EP_STIM_BAD_MASK,
    EP_STIM_TOO_SHORT,
    EP_STIM_PERIOD_TOO_SHORT,
    EP_STIM_UNBALANCED,
    EP_STIM_BAD_EVENTS,
    EP_STIM_NO_PROGRAM,
    EP_STIM_RATE_CHANGED,
};

#define EP_STIM_FLAG_ANODIC_FIRST    0x01
#define EP_STIM_FLAG_ALLOW_IMBALANCE 0x02

#endif /* EP_CMD_H */
