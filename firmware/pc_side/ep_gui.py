#!/usr/bin/env python3
"""Demo GUI for the ep_device, driven through the base node.

Commands go to the base node's shell (first USB serial port), exactly as if they
were typed; the recording arrives on its stream port (second USB serial port).

Usage:
    python3 ep_gui.py
"""
import argparse
import queue
import re
import sys
import time

import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets
from serial.tools import list_ports

import limits
from limits import Biphasic, LinkResult
from shell_link import WARN, ShellWorker
from stim_diagram import StimDiagram
from stream import CsvWriter, SampleRing, StreamReader, csv_header

SEV_COLOURS = {"error": "#d32f2f", "warn": "#b36b00", "cmd": "#1565c0"}
PLOT_REFRESH_MS = 33
STOP_TIMEOUT = 4.0  # stop recording/stim waits up to 2.5 s for stim to wind down
STIM_ABORT_REASONS = {1: "stopped", 2: "compliance limit hit", 3: "SPI error", 4: "stream fault"}


def html_line(sev: str, text: str) -> str:
    text = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    colour = SEV_COLOURS.get(sev)
    return f'<span style="color:{colour}">{text}</span>' if colour else text


def natural_key(s: str):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", s)]


class ChannelRow(QtWidgets.QWidget):
    """A row of 16 electrode checkboxes that builds a channel mask."""

    changed = QtCore.pyqtSignal(int)

    def __init__(self, initial_mask: int = 0x0001):
        super().__init__()
        grid = QtWidgets.QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(2)
        self.boxes = []
        for ch in range(limits.NUM_CHANNELS):
            label = QtWidgets.QLabel(str(ch))
            label.setAlignment(QtCore.Qt.AlignHCenter)
            box = QtWidgets.QCheckBox()
            box.setToolTip(f"Electrode {ch}")
            box.setChecked(bool(initial_mask & (1 << ch)))
            box.toggled.connect(lambda _: self.changed.emit(self.mask()))
            grid.addWidget(label, 0, ch, QtCore.Qt.AlignHCenter)
            grid.addWidget(box, 1, ch, QtCore.Qt.AlignHCenter)
            self.boxes.append(box)
        buttons = QtWidgets.QHBoxLayout()
        for text, mask in (("All", 0xFFFF), ("None", 0)):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(lambda _, m=mask: self.set_mask(m))
            buttons.addWidget(b)
        self.mask_label = QtWidgets.QLabel()
        buttons.addWidget(self.mask_label)
        buttons.addStretch()
        grid.addLayout(buttons, 2, 0, 1, limits.NUM_CHANNELS)
        self.changed.connect(lambda m: self.mask_label.setText(f"mask 0x{m:04x}"))
        self.changed.emit(self.mask())

    def mask(self) -> int:
        return sum(1 << ch for ch, b in enumerate(self.boxes) if b.isChecked())

    def set_mask(self, mask: int) -> None:
        for ch, b in enumerate(self.boxes):
            b.blockSignals(True)
            b.setChecked(bool(mask & (1 << ch)))
            b.blockSignals(False)
        self.changed.emit(self.mask())


class ReplyBox(QtWidgets.QTextEdit):
    """Shows the shell's reply to the last command, coloured by severity."""

    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        self.setMinimumHeight(90)
        self.setPlaceholderText("Replies from the base node appear here.")

    def show_reply(self, cmd: str, lines: list) -> None:
        self.clear()
        self.add_reply(cmd, lines)

    def add_reply(self, cmd: str, lines: list) -> None:
        for line in [html_line("cmd", "> " + cmd)] + [html_line(sev, t) for sev, t in lines]:
            self.append(line)


def message_html(errors=(), warnings=(), notes=()) -> str:
    return "<br>".join([html_line("error", "✖ " + e) for e in errors] +
                       [html_line("warn", "⚠ " + w) for w in warnings] +
                       [html_line("info", n) for n in notes])


# -- DEVICE TAB --

class DeviceTab(QtWidgets.QWidget):
    FIELDS = [("name", "Name"), ("firmware", "Firmware"), ("build", "Build"),
              ("zephyr", "Zephyr"), ("hardware", "Hardware"), ("rhs_state", "RHS2116"),
              ("chip id", "Chip ID"), ("die revision", "Die revision"), ("channels", "Channels")]

    def __init__(self):
        super().__init__()
        outer = QtWidgets.QVBoxLayout(self)

        pick = QtWidgets.QGroupBox("ep_devices")
        pv = QtWidgets.QVBoxLayout(pick)
        row = QtWidgets.QHBoxLayout()
        self.scan = QtWidgets.QPushButton("Scan")
        self.scan_s = QtWidgets.QSpinBox()
        self.scan_s.setRange(1, 30)
        self.scan_s.setValue(3)
        self.scan_s.setSuffix(" s")
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.conn_status = QtWidgets.QLabel()
        for w in (self.scan, self.scan_s, self.connect_button, self.disconnect_button):
            row.addWidget(w)
        row.addWidget(self.conn_status, 1)
        pv.addLayout(row)
        self.devices = QtWidgets.QTableWidget(0, 3)
        self.devices.setHorizontalHeaderLabels(["Name", "Address", "Signal"])
        self.devices.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.devices.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.devices.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.devices.verticalHeader().hide()
        header = self.devices.horizontalHeader()
        header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        header.setStretchLastSection(True)
        self.devices.setMinimumHeight(120)
        self.devices.setMaximumHeight(170)
        pv.addWidget(self.devices)
        outer.addWidget(pick)

        info = QtWidgets.QGroupBox("Connected ep_device")
        form = QtWidgets.QFormLayout(info)
        self.values = {}
        for key, label in self.FIELDS:
            v = QtWidgets.QLabel("–")
            v.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            form.addRow(label + ":", v)
            self.values[key] = v
        self.emulated = QtWidgets.QLabel(
            "EMULATED: no RHS2116 fitted, the recorded data is synthetic")
        self.emulated.setStyleSheet("background:#fff3cd; color:#664d03; padding:6px; "
                                    "border-radius:4px; font-weight:bold")
        self.emulated.hide()
        form.addRow(self.emulated)
        self.refresh = QtWidgets.QPushButton("Refresh")
        form.addRow(self.refresh)
        outer.addWidget(info)

        link = QtWidgets.QGroupBox("BLE link test")
        lv = QtWidgets.QVBoxLayout(link)
        lv.addWidget(QtWidgets.QLabel(
            "Streams dummy packets from the ep_device to measure how much data the BLE link "
            "carries. Run it for as long as you like, then stop it; the recording tab uses "
            "the result to warn when a recording won't fit."))
        row = QtWidgets.QHBoxLayout()
        self.link_button = QtWidgets.QPushButton("Start link test")
        self.link_elapsed = QtWidgets.QLabel()
        row.addWidget(self.link_button)
        row.addWidget(self.link_elapsed)
        row.addStretch()
        lv.addLayout(row)
        self.link_result = QtWidgets.QLabel("No link test results yet.")
        lv.addWidget(self.link_result)
        outer.addWidget(link)

        outer.addWidget(QtWidgets.QLabel("Reply:"))
        self.reply = ReplyBox()
        outer.addWidget(self.reply, 1)

    def show_scan(self, lines: list) -> None:
        """Fills the table from the output of 'scan'; each row keeps (address, type)."""
        self.devices.setRowCount(0)
        for _, text in lines:
            m = re.match(r"\s*\d+\s+(.+?)\s+([0-9A-Fa-f:]{17}) \((\w+)\)\s+(-?\d+) dBm", text)
            if not m:
                continue
            r = self.devices.rowCount()
            self.devices.insertRow(r)
            name, addr, kind, rssi = m.groups()
            for col, value in enumerate((name, f"{addr} ({kind})", f"{rssi} dBm")):
                item = QtWidgets.QTableWidgetItem(value)
                item.setData(QtCore.Qt.UserRole, (addr, kind))
                self.devices.setItem(r, col, item)
        if self.devices.rowCount():
            self.devices.selectRow(0)

    def selected_device(self) -> tuple[str, str] | None:
        rows = self.devices.selectionModel().selectedRows()
        if not rows:
            return None
        return self.devices.item(rows[0].row(), 0).data(QtCore.Qt.UserRole)

    def clear_device(self) -> None:
        for v in self.values.values():
            v.setText("–")
        self.emulated.hide()

    def show_device(self, lines: list) -> None:
        self.clear_device()
        section = "ep"
        for sev, text in lines:
            m = re.match(r'ep_device "(.*)"', text)
            if m:
                self.values["name"].setText(m.group(1))
                continue
            if text.strip() == "rhs2116":
                section = "rhs"
                self.values["rhs_state"].setText("OK")
                continue
            if section == "rhs" and text.strip().startswith("EMULATED"):
                self.values["rhs_state"].setText("emulated")
                self.emulated.show()
                continue
            if section == "rhs" and text.strip().startswith("unknown"):
                self.values["rhs_state"].setText(text.strip())
                continue
            m = re.match(r"\s*([a-z ]+):\s+(.*)$", text)
            if not m:
                continue
            key, value = m.group(1).strip(), m.group(2).strip()
            if key == "firmware":
                fm = re.match(r"(\S+) \(build (.*)\)", value)
                if fm:
                    value = fm.group(1)
                    self.values["build"].setText(fm.group(2))
            if key in self.values:
                self.values[key].setText(value)


# -- RECORDING TAB --

class RecordingTab(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        QtWidgets.QHBoxLayout(self).addWidget(split)

        # controls
        left = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(left)
        ch_box = QtWidgets.QGroupBox("Electrodes to record")
        QtWidgets.QVBoxLayout(ch_box).addWidget(chans := ChannelRow(0x000F))
        self.channels = chans
        lv.addWidget(ch_box)

        params = QtWidgets.QGroupBox("Parameters")
        form = QtWidgets.QFormLayout(params)
        self.rate = QtWidgets.QLineEdit("10000")
        self.rate.setValidator(QtGui.QIntValidator(1, 10_000_000))
        rate_row = QtWidgets.QHBoxLayout()
        rate_row.addWidget(self.rate)
        rate_row.addWidget(QtWidgets.QLabel("S/s per channel"))
        form.addRow("Sample rate:", rate_row)
        self.bits = QtWidgets.QComboBox()
        for b in limits.VALID_BITS:
            self.bits.addItem(f"{b} bits", b)
        self.bits.setCurrentIndex(len(limits.VALID_BITS) - 1)
        form.addRow("Resolution:", self.bits)
        self.info = QtWidgets.QLabel()
        form.addRow(self.info)
        lv.addWidget(params)

        save = QtWidgets.QGroupBox("Save to CSV")
        save.setCheckable(True)
        save.setChecked(False)
        sv = QtWidgets.QHBoxLayout(save)
        self.save_path = QtWidgets.QLineEdit()
        self.save_path.setPlaceholderText("default: rec_<date>_<time>.csv in the current folder")
        browse = QtWidgets.QPushButton("Browse…")
        browse.clicked.connect(self._browse)
        sv.addWidget(self.save_path)
        sv.addWidget(browse)
        self.save = save
        lv.addWidget(save)

        self.message = QtWidgets.QLabel()
        self.message.setWordWrap(True)
        self.message.setTextFormat(QtCore.Qt.RichText)
        lv.addWidget(self.message)
        self.button = QtWidgets.QPushButton("Start recording")
        self.button.setMinimumHeight(36)
        lv.addWidget(self.button)
        lv.addWidget(QtWidgets.QLabel("Reply:"))
        self.reply = ReplyBox()
        lv.addWidget(self.reply, 1)
        split.addWidget(left)

        # live plot
        right = QtWidgets.QWidget()
        rv = QtWidgets.QVBoxLayout(right)
        bar = QtWidgets.QHBoxLayout()
        self.window_s = QtWidgets.QComboBox()
        for s in (1, 2, 5, 10):
            self.window_s.addItem(f"{s} s", s)
        self.window_s.setCurrentIndex(1)
        self.spacing = QtWidgets.QComboBox()
        for uv in (50, 100, 200, 500, 1000, 2000, 5000):
            self.spacing.addItem(f"{uv} µV", uv)
        self.spacing.setCurrentIndex(3)
        self.remove_dc = QtWidgets.QCheckBox("Remove DC offset")
        self.remove_dc.setChecked(True)
        bar.addWidget(QtWidgets.QLabel("Window:"))
        bar.addWidget(self.window_s)
        bar.addWidget(QtWidgets.QLabel("Trace spacing:"))
        bar.addWidget(self.spacing)
        bar.addWidget(self.remove_dc)
        bar.addStretch()
        rv.addLayout(bar)
        self.plot = pg.PlotWidget()
        self.plot.setLabel("bottom", "time", units="s")
        self.plot.showGrid(x=True, y=False, alpha=0.3)
        self.plot.getPlotItem().setDownsampling(auto=True, mode="peak")
        self.plot.getPlotItem().setClipToView(True)
        self.plot.setMouseEnabled(x=False, y=True)
        rv.addWidget(self.plot, 1)
        self.stats = QtWidgets.QLabel("Not recording.")
        rv.addWidget(self.stats)
        split.addWidget(right)
        split.setStretchFactor(1, 1)
        split.setSizes([480, 800])

        self.curves: list = []
        self.stim_lines = pg.PlotDataItem(pen=pg.mkPen("#e53935", width=1, style=QtCore.Qt.DashLine),
                                          connect="pairs")
        self.plot.addItem(self.stim_lines)

    def _browse(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save recording", self.default_path(),
                                                        "CSV files (*.csv)")
        if path:
            self.save_path.setText(path if path.lower().endswith(".csv") else path + ".csv")

    @staticmethod
    def default_path() -> str:
        return time.strftime("rec_%Y%m%d_%H%M%S.csv")

    def rate_value(self) -> int | None:
        return int(self.rate.text()) if self.rate.text().isdigit() else None

    def set_locked(self, locked: bool) -> None:
        for w in (self.channels, self.rate, self.bits, self.save):
            w.setEnabled(not locked)

    def setup_traces(self, channels: list[int]) -> None:
        for c in self.curves:
            self.plot.removeItem(c)
        self.curves = []
        colours = [pg.intColor(i, hues=max(len(channels), 8), maxValue=200) for i in range(len(channels))]
        for i, _ in enumerate(channels):
            curve = self.plot.plot(pen=pg.mkPen(colours[i], width=1))
            self.curves.append(curve)
        self._channels = channels
        self._apply_ticks()

    def _apply_ticks(self) -> None:
        spacing = self.spacing.currentData()
        chans = getattr(self, "_channels", [])
        self.plot.getAxis("left").setTicks([[(-i * spacing, f"CH{ch}") for i, ch in enumerate(chans)]])
        if chans:
            self.plot.setYRange(-(len(chans) - 0.5) * spacing, 0.5 * spacing, padding=0)

    def redraw(self, ring: SampleRing, rate_hz: float) -> None:
        window = self.window_s.currentData()
        frames, uv, events = ring.latest(int(window * rate_hz))
        if not len(frames) or not self.curves:
            return
        spacing = self.spacing.currentData()
        t = frames / rate_hz
        if self.remove_dc.isChecked():
            uv = uv - uv.mean(axis=0)
        for i, curve in enumerate(self.curves):
            curve.setData(t, uv[:, i] - i * spacing)
        end = t[-1]
        self.plot.setXRange(max(end - window, 0), max(end, window), padding=0)

        y0, y1 = -(len(self.curves) - 0.5) * spacing, 0.5 * spacing
        xs, ys = [], []
        for frame, _ in events:
            xs += [frame / rate_hz] * 2
            ys += [y0, y1]
        self.stim_lines.setData(xs, ys)


# -- STIMULATION TAB --

class StimTab(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        QtWidgets.QHBoxLayout(self).addWidget(split)

        left = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(left)
        ch_box = QtWidgets.QGroupBox("Electrodes to stimulate")
        QtWidgets.QVBoxLayout(ch_box).addWidget(chans := ChannelRow(0x0001))
        self.channels = chans
        lv.addWidget(ch_box)

        d = Biphasic()
        params = QtWidgets.QGroupBox("Biphasic pulse")
        form = QtWidgets.QFormLayout(params)

        def spin(value: int, maximum: int, suffix: str, tip: str) -> QtWidgets.QSpinBox:
            s = QtWidgets.QSpinBox()
            s.setRange(0, maximum)
            s.setValue(value)
            s.setSuffix(suffix)
            s.setToolTip(tip)
            s.setAccelerated(True)
            return s

        self.cathodic = spin(d.cathodic, 255, " µA", "Cathodic (negative) amplitude, in stim steps (1 µA by default)")
        self.anodic = spin(d.anodic, 255, " µA", "Anodic (positive) amplitude, in stim steps (1 µA by default)")
        self.phase1 = spin(d.phase1_us, 0xFFFF, " µs", "Length of the first phase")
        self.interphase = spin(d.interphase_us, 0xFFFF, " µs", "Gap between the two phases")
        self.phase2 = spin(d.phase2_us, 0xFFFF, " µs", "Length of the second phase")
        self.recovery = spin(d.recovery_us, 0xFFFF, " µs",
                             "Charge-recovery switch closed after phase 2 (0 = none)")
        self.period = spin(d.period_us, 2_000_000_000, " µs", "Time from one pulse start to the next")
        self.count = spin(d.count, 0xFFFF, "", "Number of pulses; 0 repeats until stopped")
        self.count.setSpecialValueText("until stopped")
        self.anodic_first = QtWidgets.QCheckBox("Anodic phase first")
        self.unbalanced = QtWidgets.QCheckBox("Allow unbalanced charge")
        self.unbalanced.setToolTip("Lets the anodic and cathodic charge differ (not recommended)")
        for label, w in (("Cathodic amplitude:", self.cathodic), ("Anodic amplitude:", self.anodic),
                         ("Phase 1:", self.phase1), ("Interphase:", self.interphase),
                         ("Phase 2:", self.phase2), ("Recovery:", self.recovery),
                         ("Period:", self.period), ("Pulses:", self.count)):
            form.addRow(label, w)
        form.addRow(self.anodic_first)
        form.addRow(self.unbalanced)
        lv.addWidget(params)

        self.message = QtWidgets.QLabel()
        self.message.setWordWrap(True)
        self.message.setTextFormat(QtCore.Qt.RichText)
        lv.addWidget(self.message)
        self.button = QtWidgets.QPushButton("Start stimulation")
        self.button.setMinimumHeight(36)
        lv.addWidget(self.button)
        lv.addWidget(QtWidgets.QLabel("Reply:"))
        self.reply = ReplyBox()
        lv.addWidget(self.reply, 1)
        split.addWidget(left)

        self.diagram = StimDiagram()
        split.addWidget(self.diagram)
        split.setStretchFactor(1, 1)
        split.setSizes([480, 800])

        self.inputs = [self.channels, self.cathodic, self.anodic, self.phase1, self.interphase,
                       self.phase2, self.recovery, self.period, self.count, self.anodic_first,
                       self.unbalanced]

    def params(self) -> Biphasic:
        return Biphasic(
            mask=self.channels.mask(), cathodic=self.cathodic.value(), anodic=self.anodic.value(),
            phase1_us=self.phase1.value(), interphase_us=self.interphase.value(),
            phase2_us=self.phase2.value(), recovery_us=self.recovery.value(),
            period_us=self.period.value(), count=self.count.value(),
            anodic_first=self.anodic_first.isChecked(), allow_unbalanced=self.unbalanced.isChecked())

    def set_locked(self, locked: bool) -> None:
        for w in self.inputs:
            w.setEnabled(not locked)


# -- MAIN WINDOW --

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ep_device")
        self.resize(1320, 860)

        self.shell: ShellWorker | None = None
        self.connected = False
        self.busy: set[str] = set()     # tags of commands in flight
        self.link: LinkResult | None = None
        self.link_running = False
        self.link_started = 0.0
        self.recording = False
        self.rec_external = False       # running when we connected; we have no stream for it
        self.rec_rate_mhz: int | None = None
        self.rec_mask = 0
        self.rec_bits = 16
        self.reader: StreamReader | None = None
        self.writer: CsvWriter | None = None
        self.csv_queue: queue.Queue | None = None
        self.stim_running = False
        self.ep_state = "idle"          # idle, connecting, connected, reconnecting

        central = QtWidgets.QWidget()
        cv = QtWidgets.QVBoxLayout(central)

        # connection bar
        bar = QtWidgets.QHBoxLayout()
        self.shell_port = QtWidgets.QComboBox()
        self.stream_port = QtWidgets.QComboBox()
        for c in (self.shell_port, self.stream_port):
            c.setMinimumWidth(170)
        rescan = QtWidgets.QPushButton("Rescan")
        rescan.clicked.connect(self.scan_ports)
        self.base_button = QtWidgets.QPushButton("Connect to base node")
        self.base_button.clicked.connect(self.toggle_connection)
        self.status = QtWidgets.QLabel("Not connected.")
        bar.addWidget(QtWidgets.QLabel("Shell port:"))
        bar.addWidget(self.shell_port)
        bar.addWidget(QtWidgets.QLabel("Stream port:"))
        bar.addWidget(self.stream_port)
        bar.addWidget(rescan)
        bar.addWidget(self.base_button)
        bar.addWidget(self.status, 1)
        cv.addLayout(bar)

        vsplit = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.tabs = QtWidgets.QTabWidget()
        self.device_tab = DeviceTab()
        self.rec_tab = RecordingTab()
        self.stim_tab = StimTab()
        self.tabs.addTab(self.device_tab, "Device")
        self.tabs.addTab(self.rec_tab, "Recording")
        self.tabs.addTab(self.stim_tab, "Stimulation")
        vsplit.addWidget(self.tabs)
        log_box = QtWidgets.QGroupBox("Shell log")
        lb = QtWidgets.QVBoxLayout(log_box)
        self.log = QtWidgets.QTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(3000)
        self.log.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        lb.addWidget(self.log)
        vsplit.addWidget(log_box)
        vsplit.setSizes([700, 140])
        cv.addWidget(vsplit, 1)
        self.setCentralWidget(central)

        # wiring
        d = self.device_tab
        d.scan.clicked.connect(self.scan_devices)
        d.connect_button.clicked.connect(self.connect_device)
        d.disconnect_button.clicked.connect(lambda: self.send("disconnect", "ep_disconnect", 4.0))
        d.devices.itemSelectionChanged.connect(self.update_all)
        d.devices.itemDoubleClicked.connect(lambda _: self.connect_device())
        self.device_tab.refresh.clicked.connect(lambda: self.send("list device", "device"))
        self.device_tab.link_button.clicked.connect(self.toggle_link_test)
        r = self.rec_tab
        r.channels.changed.connect(self.update_recording_checks)
        r.rate.textChanged.connect(self.update_recording_checks)
        r.bits.currentIndexChanged.connect(self.update_recording_checks)
        r.spacing.currentIndexChanged.connect(r._apply_ticks)
        r.button.clicked.connect(self.toggle_recording)
        s = self.stim_tab
        for w in s.inputs:
            for sig in ("changed", "valueChanged", "toggled"):
                if hasattr(w, sig):
                    getattr(w, sig).connect(self.update_stim_checks)
        s.button.clicked.connect(self.toggle_stim)

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.refresh_plot)
        self.stats_timer = QtCore.QTimer(self)
        self.stats_timer.timeout.connect(self.refresh_stats)
        self.stats_timer.start(500)
        self._last_stats = (time.monotonic(), 0)
        # notices a lost or recovered BLE link while nothing else is going on
        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self.poll_connection)
        self.poll_timer.start(2000)

        self.handlers = {
            "scan": self.on_scan,
            "ep_connect": self.on_ep_connect,
            "ep_disconnect": self.on_ep_disconnect,
            "conn_status": self.on_conn_status,
            "conn_poll": self.on_conn_status,
            "device": self.on_device,
            "link_status": self.on_link_status,
            "link_start": self.on_link_start,
            "link_stop": self.on_link_stop,
            "rec_status": self.on_rec_status,
            "rec_start": self.on_rec_start,
            "rec_stop": self.on_rec_stop,
            "stim_load": self.on_stim_load,
            "stim_start": self.on_stim_start,
            "stim_stop": self.on_stim_stop,
        }

        self.scan_ports()
        self.update_recording_checks()
        self.update_stim_checks()

    # -- connection --

    def scan_ports(self) -> None:
        ports = sorted((p.device for p in list_ports.comports()), key=natural_key)
        for combo, default in ((self.shell_port, 0), (self.stream_port, 1)):
            current = combo.currentText()
            combo.clear()
            combo.addItems(ports)
            if current in ports:
                combo.setCurrentText(current)
            elif len(ports) > default:
                combo.setCurrentIndex(default)

    def toggle_connection(self) -> None:
        if self.shell:
            self.disconnect_shell()
            return
        port = self.shell_port.currentText()
        if not port:
            self.status.setText("No serial ports found; plug in the base node and press Rescan.")
            return
        self.shell = ShellWorker()
        self.shell.opened.connect(self.on_opened)
        self.shell.finished.connect(self.on_finished)
        self.shell.transcript.connect(lambda sev, text: self.log.append(html_line(sev, text)))
        self.status.setText(f"Connecting to {port}…")
        self.base_button.setEnabled(False)
        self.shell.start(port)

    def on_opened(self, ok: bool, message: str) -> None:
        self.base_button.setEnabled(True)
        if not ok:
            self.disconnect_shell()
            self.status.setText(html_line("error", f"Could not connect: {message}"))
            return
        self.connected = True
        self.status.setText(f"Connected to the base node on {message}.")
        self.base_button.setText("Disconnect from base node")
        self.send("status connection", "conn_status")
        self.update_all()

    def disconnect_shell(self) -> None:
        self.teardown_stream()
        if self.shell:
            self.shell.stop()
            self.shell = None
        self.connected = False
        self.busy.clear()
        self.recording = self.rec_external = self.stim_running = self.link_running = False
        self.rec_rate_mhz = None
        self.ep_state = "idle"
        self.device_tab.conn_status.setText("")
        self.base_button.setText("Connect to base node")
        self.status.setText("Not connected.")
        self.update_all()

    def send(self, cmd: str, tag: str, timeout: float = 2.0, quiet: bool = False) -> None:
        if not self.shell or not self.connected:
            return
        self.busy.add(tag)
        self.shell.submit(cmd, tag, timeout, quiet)
        self.update_all()

    def on_finished(self, tag: str, cmd: str, lines: list, ok: bool) -> None:
        self.busy.discard(tag)
        handler = self.handlers.get(tag)
        if handler:
            handler(cmd, lines, ok)
        self.update_all()

    def closeEvent(self, event) -> None:
        self.disconnect_shell()
        super().closeEvent(event)

    # -- choosing the ep_device --

    def scan_devices(self) -> None:
        seconds = self.device_tab.scan_s.value()
        self.device_tab.conn_status.setText(f"Scanning for {seconds} s…")
        self.send(f"scan {seconds}", "scan", seconds + 3.0)

    def on_scan(self, cmd, lines, ok) -> None:
        self.device_tab.reply.show_reply(cmd, lines)
        self.device_tab.show_scan(lines)
        n = self.device_tab.devices.rowCount()
        self.set_conn_label()
        if ok and n == 0:
            self.device_tab.conn_status.setText(html_line("warn", "No ep_devices found."))

    def connect_device(self) -> None:
        dev = self.device_tab.selected_device()
        if not dev or {"ep_connect", "ep_disconnect"} & self.busy:
            return
        if self.ep_state != "idle":
            # the base node only talks to one ep_device at a time
            self.send("disconnect", "ep_disconnect", 4.0)
        self.device_tab.conn_status.setText(f"Connecting to {dev[0]}…")
        self.send(f"connect {dev[0]} {dev[1]}", "ep_connect", 10.0)

    def on_ep_connect(self, cmd, lines, ok) -> None:
        self.device_tab.reply.show_reply(cmd, lines)
        if ok:
            self.set_ep_state("connected", lines)
        else:
            self.set_conn_label()

    def on_ep_disconnect(self, cmd, lines, ok) -> None:
        self.device_tab.reply.show_reply(cmd, lines)
        self.set_ep_state("idle", lines)
        # a link test result belongs to the device it was measured on
        self.link = None
        self.device_tab.link_result.setText("No link test results yet.")
        self.update_recording_checks()

    def poll_connection(self) -> None:
        if self.connected and not self.busy:
            self.send("status connection", "conn_poll", quiet=True)

    def on_conn_status(self, cmd, lines, ok) -> None:
        text = next((t for _, t in lines if t.strip()), "")
        for prefix, state in (("connected to", "connected"), ("connecting to", "connecting"),
                              ("reconnecting to", "reconnecting"), ("not connected", "idle")):
            if text.startswith(prefix):
                self.set_ep_state(state, lines)
                return

    def set_ep_state(self, state: str, lines: list) -> None:
        was = self.ep_state
        self.ep_state = state
        self._conn_text = next((t for _, t in lines if t.startswith(("connected to",
                                "connecting to", "reconnecting to"))), "")
        if state == "connected" and was != "connected":
            self.send("list device", "device")
            self.send("status link_test", "link_status")
            self.send("status recording", "rec_status")
        elif state != "connected" and was == "connected":
            self.ep_link_down(lost=state == "reconnecting")
        self.set_conn_label()
        self.update_stim_checks()

    def set_conn_label(self) -> None:
        text = getattr(self, "_conn_text", "")
        label = {
            "idle": "Not connected to an ep_device. Scan, then pick one.",
            "connected": text[:1].upper() + text[1:] + ".",
            "connecting": text[:1].upper() + text[1:] + "…",
            "reconnecting": html_line("warn", f"Link lost: {text}…"),
        }[self.ep_state]
        self.device_tab.conn_status.setText(label)

    def ep_link_down(self, lost: bool) -> None:
        """The ep_device stops recording and stimulation when the link goes."""
        if self.recording or self.reader:
            self.teardown_stream()
            if lost:
                self.rec_tab.message.setText(message_html(
                    ["The link to the ep_device was lost, which stopped the recording."]))
        self.recording = self.rec_external = self.stim_running = self.link_running = False
        self.rec_rate_mhz = None
        self.device_tab.clear_device()
        self.update_recording_checks()

    # -- device / link test --

    def on_device(self, cmd, lines, ok) -> None:
        self.device_tab.reply.show_reply(cmd, lines)
        if ok:
            self.device_tab.show_device(lines)

    def toggle_link_test(self) -> None:
        if self.link_running:
            self.send("stop link_test", "link_stop")
        else:
            self.send("start link_test", "link_start")

    def on_link_start(self, cmd, lines, ok) -> None:
        self.device_tab.reply.show_reply(cmd, lines)
        if ok:
            self.link_running = True
            self.link_started = time.monotonic()

    def on_link_stop(self, cmd, lines, ok) -> None:
        self.device_tab.reply.show_reply(cmd, lines)
        self.link_running = False
        self._parse_link(lines)

    def on_link_status(self, cmd, lines, ok) -> None:
        if not ok:
            return
        if any(re.match(r"link test running", t) for _, t in lines):
            self.link_running = True
            self.link_started = time.monotonic()
        self._parse_link(lines)

    def _parse_link(self, lines) -> None:
        rates = {}
        for _, text in lines:
            m = re.match(r"\s*(mean|10th pct)\s+(\d+)\.(\d) kB/s", text)
            if m:
                rates[m.group(1)] = int(m.group(2)) * 1000 + int(m.group(3)) * 100
        if "mean" in rates and "10th pct" in rates:
            self.link = LinkResult(rates["mean"], rates["10th pct"])
            self.device_tab.link_result.setText(
                f"Last result: mean {self.link.mean_bps / 1000:.1f} kB/s, "
                f"10th percentile {self.link.p10_bps / 1000:.1f} kB/s.")
        self.update_recording_checks()

    # -- recording --

    def update_recording_checks(self) -> None:
        r = self.rec_tab
        if self.recording:
            return
        mask, bits = r.channels.mask(), r.bits.currentData()
        c = limits.check_recording(mask, r.rate_value(), bits, self.link)
        n = len(limits.mask_channels(mask))
        info = f"{n} channel{'s' if n != 1 else ''}: max {c.max_rate_hz} S/s per channel (SPI limit)"
        if c.rate_mhz:
            info += (f"<br>actual rate {c.rate_mhz / 1000:.3f} S/s, needs "
                     f"{c.required_bps / 1000:.1f} kB/s over BLE")
        r.info.setText(info)
        r.message.setText(message_html(c.errors, c.warnings, c.notes))
        self._rec_check = c
        self.update_all()

    def toggle_recording(self) -> None:
        if self.recording:
            self.send("stop recording", "rec_stop", STOP_TIMEOUT)
            return

        r = self.rec_tab
        mask, rate, bits = r.channels.mask(), r.rate_value(), r.bits.currentData()
        channels = limits.mask_channels(mask)
        ring = SampleRing(len(channels), rate)
        self.csv_queue = queue.Queue() if r.save.isChecked() else None
        # open the stream port first so the first packets aren't discarded
        self.reader = StreamReader(self.stream_port.currentText(), ring, self.csv_queue)
        self.reader.error.connect(lambda e: r.message.setText(message_html([e])))
        self.reader.stim_event.connect(self.on_stream_event)
        self.reader.start()
        self.rec_mask, self.rec_bits = mask, bits
        r.setup_traces(channels)
        self.send(f"start recording 0x{mask:04x} {rate} {bits}", "rec_start")

    def on_rec_start(self, cmd, lines, ok) -> None:
        r = self.rec_tab
        r.reply.show_reply(cmd, lines)
        m = next((re.search(r"at (\d+)\.(\d+) S/s/ch", t) for _, t in lines
                  if re.search(r"at (\d+)\.(\d+) S/s/ch", t)), None)
        if not ok or not m:
            self.teardown_stream()
            r.message.setText(message_html(["Recording did not start; see the reply below."]))
            return
        self.recording = True
        self.rec_rate_mhz = int(m.group(1)) * 1000 + int(m.group(2))
        warnings = [t for sev, t in lines if sev == WARN]
        notes = []
        if self.csv_queue is not None:
            path = r.save_path.text().strip() or r.default_path()
            self.writer = CsvWriter(path, csv_header(self.rec_mask, self.rec_rate_mhz, self.rec_bits),
                                    limits.mask_channels(self.rec_mask), self.csv_queue)
            self.writer.error.connect(self.on_writer_error)
            self.writer.start()
            notes.append(f"Saving to {path}")
        r.message.setText(message_html([], warnings, notes))
        self._last_stats = (time.monotonic(), 0)
        self.plot_timer.start(PLOT_REFRESH_MS)
        self.update_stim_checks()

    def on_writer_error(self, message: str) -> None:
        if self.reader:
            self.reader.csv_queue = None
        self.rec_tab.message.setText(message_html([message]))

    def on_rec_stop(self, cmd, lines, ok) -> None:
        self.rec_tab.reply.show_reply(cmd, lines)
        self.recording = self.rec_external = False
        self.stim_running = False  # the firmware stops stimulation with the recording
        self.rec_rate_mhz = None
        # give the last packets in flight a moment to arrive
        QtCore.QTimer.singleShot(300, self.teardown_stream)
        self.update_stim_checks()
        self.update_recording_checks()

    def on_rec_status(self, cmd, lines, ok) -> None:
        if not ok:
            return
        m = next((re.match(r"recording (\w+), stimulation (\w+)", t) for _, t in lines
                  if t.startswith("recording ")), None)
        if m and m.group(1) == "running":
            self.recording = self.rec_external = True
            self.stim_running = m.group(2) == "running"
            self.rec_tab.message.setText(message_html(
                warnings=["A recording was already running when the GUI connected. "
                          "Stop it and start it again from here to see the data."]))

    def teardown_stream(self) -> None:
        self.plot_timer.stop()
        if self.reader:
            self.reader.stop()
            stats = self.reader.stats
            self.reader = None
        else:
            stats = None
        if self.writer:
            self.writer.finish()
            if stats:
                self.rec_tab.message.setText(message_html(
                    notes=[f"Saved {stats.frames} frames to {self.writer.path} "
                           f"(events in {self.writer.events_path})."]))
            self.writer = None
        self.csv_queue = None

    def refresh_plot(self) -> None:
        if self.reader and self.rec_rate_mhz:
            self.rec_tab.redraw(self.reader.ring, self.rec_rate_mhz / 1000)

    def refresh_stats(self) -> None:
        if self.link_running:
            el = time.monotonic() - self.link_started
            self.device_tab.link_elapsed.setText(f"running for {int(el) // 60}:{int(el) % 60:02d}")
        else:
            self.device_tab.link_elapsed.setText("")

        if not self.reader:
            if not self.recording:
                self.rec_tab.stats.setText("Not recording.")
            return
        s = self.reader.stats
        now = time.monotonic()
        t0, f0 = self._last_stats
        rate = (s.frames - f0) / (now - t0) if now > t0 else 0
        self._last_stats = (now, s.frames)
        text = (f"{rate:,.0f} S/s per channel received · {s.frames:,} frames · "
                f"{s.missing_frames:,} missing frames · {s.ble_gaps} BLE packets lost · "
                f"{self.reader.parser.usb_gaps} USB frames lost")
        if s.device_dropped:
            text += " · ep_device dropped packets"
        if self.writer and self.rec_rate_mhz:
            # roughly one queued item per packet
            backlog_s = self.writer.backlog() * 10 / (self.rec_rate_mhz / 1000)
            if backlog_s > 5:
                text += " · " + html_line("warn", "file writer falling behind")
        self.rec_tab.stats.setText(text)

    # -- stimulation --

    def update_stim_checks(self) -> None:
        s = self.stim_tab
        if self.stim_running:
            self.update_all()
            return
        b = s.params()
        rate = self.rec_rate_mhz if self.recording and not self.rec_external else None
        errors, frames = limits.check_biphasic(b, rate)
        if self.rec_external:
            errors = ["Restart the recording from the Recording tab first."] + \
                     [e for e in errors if not e.startswith("Start recording first")]
        s.message.setText(message_html(errors))
        s.diagram.set_params(b, frames, rate)
        self._stim_errors = errors
        self.update_all()

    def toggle_stim(self) -> None:
        if self.stim_running:
            self.send("stop stim", "stim_stop", STOP_TIMEOUT)
        else:
            self.send(self.stim_tab.params().shell_command(), "stim_load")

    def on_stim_load(self, cmd, lines, ok) -> None:
        self.stim_tab.reply.show_reply(cmd, lines)
        if ok:
            self.send("start stim", "stim_start")

    def on_stim_start(self, cmd, lines, ok) -> None:
        self.stim_tab.reply.add_reply(cmd, lines)
        if ok:
            self.stim_running = True
            self.stim_tab.message.setText(message_html(notes=["Stimulating."]))

    def on_stim_stop(self, cmd, lines, ok) -> None:
        self.stim_tab.reply.show_reply(cmd, lines)
        self.stim_running = False
        self.update_stim_checks()

    def on_stream_event(self, name: str, a: int) -> None:
        if name == "stim_done":
            self.stim_running = False
            self.update_stim_checks()
            self.stim_tab.message.setText(message_html(notes=[f"Stimulation finished: {a} pulses."]))
        elif name == "stim_abort":
            self.stim_running = False
            self.update_stim_checks()
            reason = STIM_ABORT_REASONS.get(a, f"reason {a}")
            if a != 1:
                self.stim_tab.message.setText(message_html([f"Stimulation aborted: {reason}."]))
        elif name == "stream_stopped":
            self.rec_tab.message.setText(message_html(
                ["The ep_device stopped streaming. Press Stop recording."]))

    # -- widget state --

    def update_all(self) -> None:
        d, r, s = self.device_tab, self.rec_tab, self.stim_tab
        choosing = {"scan", "ep_connect", "ep_disconnect"} & self.busy
        d.scan.setEnabled(self.connected and not choosing)
        d.connect_button.setEnabled(self.connected and not choosing and
                                    d.selected_device() is not None)
        d.disconnect_button.setEnabled(self.connected and not choosing and self.ep_state != "idle")

        # everything else needs a connected ep_device
        idle = self.connected and self.ep_state == "connected"
        d.refresh.setEnabled(idle and "device" not in self.busy)
        d.link_button.setText("Stop link test" if self.link_running else "Start link test")
        d.link_button.setEnabled(idle and not self.recording and
                                 not {"link_start", "link_stop"} & self.busy)

        rec_busy = bool({"rec_start", "rec_stop"} & self.busy)
        if self.recording:
            r.button.setText("Stop recording")
            r.button.setEnabled(idle and not rec_busy)
        else:
            r.button.setText("Starting…" if "rec_start" in self.busy else "Start recording")
            ok = not getattr(self, "_rec_check", None) or not self._rec_check.errors
            r.button.setEnabled(idle and ok and not rec_busy and not self.link_running)
        r.set_locked(self.recording or rec_busy)

        stim_busy = bool({"stim_load", "stim_start", "stim_stop"} & self.busy)
        if self.stim_running:
            s.button.setText("Stop stimulation")
            s.button.setEnabled(idle and not stim_busy)
        else:
            s.button.setText("Starting…" if stim_busy else "Start stimulation")
            s.button.setEnabled(idle and self.recording and not self.rec_external and
                                not stim_busy and not getattr(self, "_stim_errors", ["?"]))
        s.set_locked(self.stim_running or stim_busy)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--screenshot", metavar="PREFIX",
                    help="render each tab to PREFIX<n>.png and exit (for testing)")
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=False)
    w = MainWindow()
    w.show()
    if args.screenshot:
        app.processEvents()
        for i in range(w.tabs.count()):
            w.tabs.setCurrentIndex(i)
            app.processEvents()
            w.grab().save(f"{args.screenshot}{i}.png")
        return 0
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
