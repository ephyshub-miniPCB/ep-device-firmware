"""Decoders for the base node's USB stream port and the ep_device recording packets.

USB frames (base_node/src/usb_stream.h), little-endian:
    sync 0xA5 0x5A | type u8 | seq u16 | len u16 | payload[len]

USB_FRAME_DATA payloads are ep_device data characteristic notifications
(ep_device/src/recording.h):
    type u8 | flags u8 | seq u16 | ...
    SAMPLES (0x01): first_frame u32 | n_frames u8 | bits u8 | chan_mask u16 |
                    n_frames x n_ch samples bit-packed LSB first
    EVENT   (0x02): code u8 | frame u32 | a u16 | b u16
"""
from dataclasses import dataclass

import numpy as np

USB_SYNC = b"\xa5\x5a"
USB_HEADER_LEN = 7
USB_FRAME_DATA = 0x01

REC_PKT_SAMPLES = 0x01
REC_PKT_EVENT = 0x02
REC_SAMPLES_HDR_LEN = 12
REC_EVENT_LEN = 13

REC_FLAG_STIM_ACTIVE = 0x01
REC_FLAG_DROPPED = 0x02

EVENT_NAMES = {
    0x01: "stim_edge",       # a: on mask, b: polarity mask
    0x02: "recovery",        # a: charge recovery switch mask
    0x03: "stim_done",       # a: pulses delivered
    0x04: "stim_abort",      # a: reason (1 stopped, 2 compliance, 3 spi error, 4 stream fault)
    0x05: "compliance",      # a: compliance monitor bits
    0x06: "spi_error",       # (b << 16) | a: bad response word
    0x07: "stream_stopped",  # a: -errno (as u16)
}

# RHS2116 AC amplifier: 0.195 uV per LSB of the 16-bit offset-binary code
UV_PER_LSB = 0.195


@dataclass
class SamplesPacket:
    seq: int
    flags: int
    first_frame: int
    bits: int
    chan_mask: int
    channels: list[int]
    codes: np.ndarray  # (n_frames, n_ch) uint16, rescaled to the full 16-bit code

    def microvolts(self) -> np.ndarray:
        return (self.codes.astype(np.float32) - 32768.0) * UV_PER_LSB


@dataclass
class EventPacket:
    seq: int
    flags: int
    code: int
    frame: int
    a: int
    b: int

    @property
    def name(self) -> str:
        return EVENT_NAMES.get(self.code, f"event_0x{self.code:02x}")


def unpack_samples(data: bytes, n_samples: int, bits: int) -> np.ndarray:
    """Unpacks n_samples LSB-first bit-packed values of 'bits' bits each."""
    raw = np.frombuffer(data, dtype=np.uint8)
    bitstream = np.unpackbits(raw, bitorder="little")[: n_samples * bits]
    groups = bitstream.reshape(n_samples, bits).astype(np.uint32)
    return (groups << np.arange(bits, dtype=np.uint32)).sum(axis=1).astype(np.uint16)


def decode_packet(payload: bytes):
    """Decodes one ep_device data notification; returns a packet or None."""
    if len(payload) < 4:
        return None
    ptype, flags = payload[0], payload[1]
    seq = int.from_bytes(payload[2:4], "little")

    if ptype == REC_PKT_SAMPLES and len(payload) >= REC_SAMPLES_HDR_LEN:
        first_frame = int.from_bytes(payload[4:8], "little")
        n_frames, bits = payload[8], payload[9]
        chan_mask = int.from_bytes(payload[10:12], "little")
        channels = [ch for ch in range(16) if chan_mask & (1 << ch)]
        n = n_frames * len(channels)
        if bits == 0 or not channels or len(payload) < REC_SAMPLES_HDR_LEN + (n * bits + 7) // 8:
            return None
        values = unpack_samples(payload[REC_SAMPLES_HDR_LEN:], n, bits)
        codes = (values << (16 - bits)).reshape(n_frames, len(channels))
        return SamplesPacket(seq, flags, first_frame, bits, chan_mask, channels, codes)

    if ptype == REC_PKT_EVENT and len(payload) >= REC_EVENT_LEN:
        return EventPacket(
            seq,
            flags,
            payload[4],
            int.from_bytes(payload[5:9], "little"),
            int.from_bytes(payload[9:11], "little"),
            int.from_bytes(payload[11:13], "little"),
        )
    return None


class UsbFrameParser:
    """Incremental parser for the USB stream port; resyncs on the sync bytes."""

    def __init__(self):
        self._buf = bytearray()
        self.usb_gaps = 0  # frames the base node dropped
        self._next_seq = None

    def feed(self, data: bytes):
        """Yields (type, payload) for every complete frame in data."""
        self._buf += data
        while True:
            start = self._buf.find(USB_SYNC)
            if start < 0:
                del self._buf[:-1]
                return
            if start:
                del self._buf[:start]
            if len(self._buf) < USB_HEADER_LEN:
                return
            ftype = self._buf[2]
            seq = int.from_bytes(self._buf[3:5], "little")
            length = int.from_bytes(self._buf[5:7], "little")
            end = USB_HEADER_LEN + length
            if len(self._buf) < end + 2:
                return
            # a real frame is followed by the next sync; otherwise this was noise
            if self._buf[end:end + 2] != USB_SYNC:
                del self._buf[:1]
                continue
            if self._next_seq is not None and seq != self._next_seq:
                self.usb_gaps += (seq - self._next_seq) & 0xFFFF
            self._next_seq = (seq + 1) & 0xFFFF
            payload = bytes(self._buf[USB_HEADER_LEN:end])
            del self._buf[:end]
            yield ftype, payload
