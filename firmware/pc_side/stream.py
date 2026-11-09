"""Receives a recording from the base node's stream port (the second CDC ACM port)
and optionally writes it to CSV.

StreamReader parses frames on its own thread into a SampleRing that the GUI
polls for plotting; CsvWriter drains a queue on another thread so disk writes
never hold up the serial port.
"""
import queue
import threading
import time

import numpy as np
import serial
from PyQt5 import QtCore

from rec_packets import (REC_FLAG_DROPPED, UV_PER_LSB, USB_FRAME_DATA, EventPacket,
                         SamplesPacket, UsbFrameParser, decode_packet)

RING_SECONDS = 12


class SampleRing:
    """Last few seconds of samples (uV) with their frame indices, plus events."""

    def __init__(self, n_ch: int, rate_hz: float):
        self.n_ch = n_ch
        self.capacity = max(int(rate_hz * RING_SECONDS), 1024)
        self._lock = threading.Lock()
        self._frames = np.zeros(self.capacity, dtype=np.int64)
        self._uv = np.zeros((self.capacity, n_ch), dtype=np.float32)
        self._head = 0
        self._count = 0
        self._events: list[tuple[int, str]] = []

    def push(self, first_frame: int, uv: np.ndarray) -> None:
        n = len(uv)
        if n > self.capacity:
            first_frame += n - self.capacity
            uv = uv[-self.capacity:]
            n = self.capacity
        idx = (self._head + np.arange(n)) % self.capacity
        with self._lock:
            self._frames[idx] = first_frame + np.arange(n)
            self._uv[idx] = uv
            self._head = (self._head + n) % self.capacity
            self._count = min(self._count + n, self.capacity)

    def add_event(self, frame: int, name: str) -> None:
        with self._lock:
            self._events.append((frame, name))

    def latest(self, n: int) -> tuple[np.ndarray, np.ndarray, list[tuple[int, str]]]:
        """Up to the last n frames as (frames, uV[n, n_ch]) in order, and the
        events at or after the first of them."""
        with self._lock:
            n = min(n, self._count)
            idx = (self._head - n + np.arange(n)) % self.capacity
            frames, uv = self._frames[idx], self._uv[idx]
            start = frames[0] if n else 0
            self._events = [e for e in self._events if e[0] >= start - self.capacity]
            events = [e for e in self._events if e[0] >= start]
        return frames, uv, events


class StreamStats:
    def __init__(self):
        self.frames = 0          # frames received
        self.missing_frames = 0  # gaps in the frame index
        self.ble_gaps = 0        # packets lost over BLE or on the ep_device
        self.device_dropped = False


class StreamReader(QtCore.QThread):
    """Reads the stream port until stop() is called."""

    error = QtCore.pyqtSignal(str)
    stim_event = QtCore.pyqtSignal(str, int)  # event name, a

    def __init__(self, port: str, ring: SampleRing, csv_queue: queue.Queue | None):
        super().__init__()
        self._port = port
        self.ring = ring
        self.csv_queue = csv_queue
        self._running = True
        self.stats = StreamStats()
        self.parser = UsbFrameParser()
        self.channels: list[int] | None = None
        self.bits: int | None = None

    def stop(self) -> None:
        self._running = False
        self.wait(2000)

    def run(self) -> None:
        try:
            # pyserial asserts DTR on open, which the base node needs before it
            # queues any frames
            ser = serial.Serial(self._port, timeout=0.05)
        except (serial.SerialException, OSError) as e:
            self.error.emit(f"could not open stream port {self._port}: {e}")
            return

        next_seq = None
        next_frame = None
        try:
            while self._running:
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                for ftype, payload in self.parser.feed(data):
                    if ftype != USB_FRAME_DATA:
                        continue
                    pkt = decode_packet(payload)
                    if pkt is None:
                        continue
                    if next_seq is not None and pkt.seq != next_seq:
                        self.stats.ble_gaps += (pkt.seq - next_seq) & 0xFFFF
                    next_seq = (pkt.seq + 1) & 0xFFFF

                    if isinstance(pkt, EventPacket):
                        self._on_event(pkt)
                    elif isinstance(pkt, SamplesPacket):
                        if next_frame is not None and pkt.first_frame > next_frame:
                            self.stats.missing_frames += pkt.first_frame - next_frame
                        next_frame = pkt.first_frame + len(pkt.codes)
                        self._on_samples(pkt)
        except (serial.SerialException, OSError) as e:
            self.error.emit(f"stream port error: {e}")
        finally:
            ser.close()

    def _on_samples(self, pkt: SamplesPacket) -> None:
        self.channels, self.bits = pkt.channels, pkt.bits
        if pkt.flags & REC_FLAG_DROPPED:
            self.stats.device_dropped = True
        uv = pkt.microvolts()
        if uv.shape[1] != self.ring.n_ch:
            return  # not the recording we were set up for
        self.stats.frames += len(uv)
        self.ring.push(pkt.first_frame, uv)
        if self.csv_queue is not None:
            self.csv_queue.put(("samples", pkt.first_frame, uv))

    def _on_event(self, pkt: EventPacket) -> None:
        if pkt.name == "stim_edge" and pkt.a:
            self.ring.add_event(pkt.frame, pkt.name)
        if pkt.name in ("stim_done", "stim_abort", "stream_stopped"):
            self.stim_event.emit(pkt.name, pkt.a)
        if self.csv_queue is not None:
            self.csv_queue.put(("event", pkt.frame, pkt.name, pkt.a, pkt.b))


class CsvWriter(QtCore.QThread):
    """Writes queued samples to <path> and events to <path stem>_events.csv.

    Each packet's block of frames goes out in one np.savetxt call; writing row by
    row from Python would struggle at the higher channel counts and rates."""

    error = QtCore.pyqtSignal(str)

    def __init__(self, path: str, header: list[str], channels: list[int], q: queue.Queue):
        super().__init__()
        self.path = path
        self.events_path = (path[:-4] if path.lower().endswith(".csv") else path) + "_events.csv"
        self._header = header
        self._channels = channels
        self._queue = q
        self._fmt = ["%d"] + ["%.2f"] * len(channels)

    def finish(self) -> None:
        """Writes whatever is still queued, then closes the files."""
        self._queue.put(None)
        self.wait()

    def backlog(self) -> int:
        return self._queue.qsize()

    def run(self) -> None:
        try:
            with open(self.path, "w", newline="") as f, \
                 open(self.events_path, "w", newline="") as fe:
                for line in self._header:
                    f.write(f"# {line}\n")
                f.write("frame," + ",".join(f"ch{ch}_uV" for ch in self._channels) + "\n")
                fe.write("frame,event,a,b\n")

                last_flush = time.monotonic()
                while True:
                    item = self._queue.get()
                    if item is None:
                        break
                    if item[0] == "samples":
                        _, first, uv = item
                        frames = np.arange(first, first + len(uv), dtype=np.float64)
                        np.savetxt(f, np.column_stack((frames, uv)), fmt=self._fmt,
                                   delimiter=",")
                    else:
                        _, frame, name, a, b = item
                        fe.write(f"{frame},{name},{a},{b}\n")
                    if time.monotonic() - last_flush > 1.0:
                        f.flush()
                        fe.flush()
                        last_flush = time.monotonic()
        except OSError as e:
            self.error.emit(f"could not write {self.path}: {e}")


def csv_header(mask: int, rate_mhz: int, bits: int) -> list[str]:
    return [
        "ep_device recording",
        f"started {time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"channel mask 0x{mask:04x}",
        f"sample rate {rate_mhz / 1000:.3f} S/s per channel",
        f"bits {bits}",
        f"uV per LSB {UV_PER_LSB} (16-bit code); time of a row = frame / sample rate",
    ]
