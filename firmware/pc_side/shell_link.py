"""Runs base node shell commands over its console port (the first CDC ACM port).

The shell colours shell_warn() output yellow and shell_error() red
(CONFIG_SHELL_VT100_COLORS), so each reply line is tagged with a severity taken
from its colour before the escape codes are stripped.
"""
import queue
import re
import time

import serial
from PyQt5 import QtCore

PROMPT = "uart:~$ "
DEFAULT_TIMEOUT = 2.0

INFO, WARN, ERROR = "info", "warn", "error"

_ANSI = re.compile(r"\x1b\[([0-9;?]*)([A-Za-z])")


def strip_ansi(text: str) -> str:
    return _ANSI.sub("", text)


class _Colour:
    """Tracks the VT100 foreground colour across lines and classifies text."""

    def __init__(self):
        self.severity = INFO

    def classify(self, line: str) -> str:
        """Severity of the first visible text in line (colour carries over
        from earlier lines, as the shell resets it after the newline)."""
        found = None
        pos = 0
        for m in _ANSI.finditer(line):
            if found is None and line[pos:m.start()].strip():
                found = self.severity
            if m.group(2) == "m":
                self._apply_sgr(m.group(1))
            pos = m.end()
        if found is None:
            found = self.severity
        return found

    def _apply_sgr(self, params: str) -> None:
        for p in (params or "0").split(";"):
            if p in ("", "0", "39"):
                self.severity = INFO
            elif p == "31":
                self.severity = ERROR
            elif p == "33":
                self.severity = WARN
            elif p.startswith("3") and len(p) == 2:
                self.severity = INFO


class ShellLink:
    """Blocking shell client. Use from one thread only."""

    def __init__(self, port: str):
        # pyserial asserts DTR on open, which the shell waits for
        # (CONFIG_SHELL_BACKEND_SERIAL_CHECK_DTR)
        self._ser = serial.Serial(port, 115200, timeout=0.05)
        self._colour = _Colour()
        self._pending = ""

    def close(self) -> None:
        self._ser.close()

    def read_async(self) -> list[tuple[str, str]]:
        """Complete lines that arrived outside a command (log messages)."""
        data = self._ser.read(self._ser.in_waiting or 1)
        if not data:
            return []
        self._pending += data.decode(errors="replace")
        lines = []
        while "\n" in self._pending:
            raw, self._pending = self._pending.split("\n", 1)
            text = strip_ansi(raw).strip("\r")
            sev = self._colour.classify(raw)
            if text.strip() and text.strip() != PROMPT.strip():
                lines.append((sev, text))
        return lines

    def run(self, cmd: str, timeout: float = DEFAULT_TIMEOUT) -> tuple[list[tuple[str, str]], bool]:
        """Sends cmd and collects its output up to the next prompt.
        Returns ([(severity, text)], completed)."""
        early = self.read_async()
        self._pending = ""
        self._ser.write((cmd + "\n").encode())

        buf = ""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            data = self._ser.read(self._ser.in_waiting or 1)
            if not data:
                continue
            buf += data.decode(errors="replace")
            # done once a newline has been seen (end of the echo) and the
            # last, unterminated line is the prompt again
            nl = buf.rfind("\n")
            if nl >= 0 and strip_ansi(buf[nl + 1:]).endswith(PROMPT):
                lines = early + self._parse(cmd, buf[:nl])
                # the colour reset after the last line comes in front of the prompt
                self._colour.classify(buf[nl + 1:])
                return lines, True
        lines = early + self._parse(cmd, buf)
        lines.append((ERROR, f"no reply to '{cmd}' within {timeout:.1f} s"))
        return lines, False

    def _parse(self, cmd: str, text: str) -> list[tuple[str, str]]:
        out = []
        echo_seen = False
        for raw in text.split("\n"):
            sev = self._colour.classify(raw)
            line = strip_ansi(raw).strip("\r")
            if not echo_seen and line.rstrip().endswith(cmd):
                echo_seen = True
                continue
            if line.strip() and line.strip() != PROMPT.strip():
                out.append((sev, line.rstrip()))
        return out


class ShellWorker(QtCore.QObject):
    """Owns a ShellLink on its own QThread and runs submitted commands in order."""

    finished = QtCore.pyqtSignal(str, str, list, bool)  # tag, cmd, [(sev, text)], ok
    transcript = QtCore.pyqtSignal(str, str)           # severity, text
    opened = QtCore.pyqtSignal(bool, str)              # success, message

    def __init__(self):
        super().__init__()
        self._queue: queue.Queue = queue.Queue()
        self._thread = QtCore.QThread()
        self.moveToThread(self._thread)
        self._thread.started.connect(self._loop)
        self._running = False

    def start(self, port: str) -> None:
        self._port = port
        self._running = True
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        self._thread.quit()
        self._thread.wait(3000)

    def submit(self, cmd: str, tag: str = "", timeout: float = DEFAULT_TIMEOUT,
               quiet: bool = False) -> None:
        """quiet keeps the command and its reply out of the transcript (for polling)."""
        self._queue.put((cmd, tag, timeout, quiet))

    def _loop(self) -> None:
        try:
            link = ShellLink(self._port)
            lines, ok = link.run("", timeout=1.0)  # wait for a prompt
        except (serial.SerialException, OSError) as e:
            self.opened.emit(False, str(e))
            return
        if not ok:
            link.close()
            self.opened.emit(False, f"no shell prompt on {self._port}")
            return
        self.opened.emit(True, self._port)

        try:
            while self._running:
                try:
                    cmd, tag, timeout, quiet = self._queue.get(timeout=0.05)
                except queue.Empty:
                    for sev, text in link.read_async():
                        self.transcript.emit(sev, text)
                    continue
                lines, done = link.run(cmd, timeout)
                if not quiet:
                    self.transcript.emit("cmd", "> " + cmd)
                    for sev, text in lines:
                        self.transcript.emit(sev, text)
                ok = done and not any(sev == ERROR for sev, _ in lines)
                self.finished.emit(tag, cmd, lines, ok)
        except (serial.SerialException, OSError) as e:
            self.transcript.emit(ERROR, f"shell port error: {e}")
            self.opened.emit(False, str(e))
        finally:
            link.close()
