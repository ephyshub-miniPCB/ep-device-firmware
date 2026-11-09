"""Schematic of a biphasic stimulation pulse train, labelled with the parameters
of 'stim biphasic'. Not to scale: short segments are widened so their labels fit,
and the gap between pulses is shortened (marked with a break)."""
from PyQt5 import QtCore, QtGui, QtWidgets

from limits import Biphasic, BiphasicFrames, mask_channels

CATHODIC_COLOUR = QtGui.QColor(66, 133, 244)
ANODIC_COLOUR = QtGui.QColor(219, 68, 55)
INTERPHASE_COLOUR = QtGui.QColor(150, 150, 150)
RECOVERY_COLOUR = QtGui.QColor(15, 157, 88)


class StimDiagram(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(520, 360)
        self._b = Biphasic()
        self._frames: BiphasicFrames | None = None
        self._rate_mhz: int | None = None

    def set_params(self, b: Biphasic, frames: BiphasicFrames | None, rate_mhz: int | None) -> None:
        self._b, self._frames, self._rate_mhz = b, frames, rate_mhz
        self.update()

    # -- timing --

    def _durations(self) -> dict[str, float]:
        """Segment lengths in us: rounded to whole frames once the rate is known."""
        b, f = self._b, self._frames
        req = {"phase1": b.phase1_us, "interphase": b.interphase_us, "phase2": b.phase2_us,
               "recovery": b.recovery_us, "period": b.period_us}
        if f is None or not self._rate_mhz:
            return {k: float(v) for k, v in req.items()}
        us_per_frame = 1e9 / self._rate_mhz
        return {k: getattr(f, k) * us_per_frame for k in req}

    def _value_text(self, key: str, us: float) -> str:
        text = f"{us:.0f} µs" if us >= 10 else f"{us:.1f} µs"
        if self._frames is not None:
            n = getattr(self._frames, key)
            text += f"\n{n} frame{'s' if n != 1 else ''}"
        return text

    # -- drawing --

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        pal = self.palette()
        fg = pal.color(QtGui.QPalette.WindowText)
        p.fillRect(self.rect(), pal.color(QtGui.QPalette.Base))

        b = self._b
        d = self._durations()
        first_cath = not b.anodic_first
        amp1, amp2 = (b.cathodic, b.anodic) if first_cath else (b.anodic, b.cathodic)
        col1, col2 = ((CATHODIC_COLOUR, ANODIC_COLOUR) if first_cath
                      else (ANODIC_COLOUR, CATHODIC_COLOUR))
        name1 = "cathodic" if first_cath else "anodic"
        name2 = "anodic" if first_cath else "cathodic"

        # (key, label, us, signed amplitude, colour)
        pulse = [("phase1", f"phase 1\n({name1})", d["phase1"], -amp1 if first_cath else amp1, col1)]
        if d["interphase"] > 0:
            pulse.append(("interphase", "interphase", d["interphase"], 0, INTERPHASE_COLOUR))
        pulse.append(("phase2", f"phase 2\n({name2})", d["phase2"], amp2 if first_cath else -amp2, col2))
        if d["recovery"] > 0:
            pulse.append(("recovery", "recovery", d["recovery"], 0, RECOVERY_COLOUR))

        pulse_us = sum(s[2] for s in pulse)
        repeats = b.count != 1
        rest_us = max(d["period"] - pulse_us, 0)

        # display widths: every pulse segment gets at least 16% of the pulse
        # so its label fits; the rest of the period is drawn compressed
        unit = max(pulse_us, 1)
        widths = [max(s[2], 0.16 * unit) for s in pulse]
        pulse_w = sum(widths)
        lead_w = 0.35 * pulse_w
        rest_w = min(max(rest_us, 0.2 * unit) / unit * pulse_w, 1.3 * pulse_w) if repeats else 0.6 * pulse_w
        rest_broken = repeats and rest_us / unit * pulse_w > rest_w
        second_w = sum(widths[:3 if d["interphase"] > 0 else 2]) if repeats else 0
        tail_w = 0.25 * pulse_w
        total_w = lead_w + pulse_w + rest_w + second_w + tail_w

        left, right, top, bottom = 110, 20, 52, 92
        plot = QtCore.QRectF(left, top, self.width() - left - right, self.height() - top - bottom)
        sx = plot.width() / total_w
        zero_y = plot.center().y()
        max_amp = max(b.cathodic, b.anodic, 1)
        half_h = plot.height() / 2 * 0.62

        def ypx(amp: float) -> float:
            return zero_y - amp / max_amp * half_h

        small = QtGui.QFont(self.font())
        small.setPointSizeF(self.font().pointSizeF() * 0.9)
        bold = QtGui.QFont(self.font())
        bold.setBold(True)

        # title
        chans = mask_channels(b.mask)
        ch_text = ", ".join(f"CH{c}" for c in chans) if chans else "no channels"
        reps = ("repeats until stopped" if b.count == 0 else
                "single pulse" if b.count == 1 else f"{b.count} pulses")
        p.setPen(fg)
        p.setFont(bold)
        p.drawText(QtCore.QRectF(10, 6, self.width() - 20, 20), QtCore.Qt.AlignLeft,
                   f"Biphasic pulse on {ch_text}  ·  {reps}")
        p.setFont(small)
        note = ("Times rounded to whole sample frames at the current recording rate."
                if self._frames is not None else
                "Times as requested; they are rounded to whole sample frames once recording.")
        p.drawText(QtCore.QRectF(10, 26, self.width() - 20, 18), QtCore.Qt.AlignLeft, note)

        # segments: (x0, x1, amplitude, key, label, us, colour)
        segs = []
        x = plot.left() + lead_w * sx
        for (key, label, us, amp, col), w in zip(pulse, widths):
            segs.append((x, x + w * sx, amp, key, label, us, col))
            x += w * sx
        pulse_end_x = x
        rest_end_x = x + rest_w * sx
        second = []
        x = rest_end_x
        if repeats:
            for (key, label, us, amp, col), w in zip(pulse[:3 if d["interphase"] > 0 else 2], widths):
                second.append((x, x + w * sx, amp, col))
                x += w * sx

        # shaded backgrounds and labels; neighbouring labels are staggered so
        # narrow segments don't overlap
        for i, (x0, x1, amp, key, label, us, col) in enumerate(segs):
            stagger = 34 * (i % 2)
            shade = QtGui.QColor(col)
            shade.setAlpha(40)
            p.fillRect(QtCore.QRectF(x0, plot.top(), x1 - x0, plot.height()), shade)
            if key == "recovery":
                hatch = QtGui.QColor(col)
                hatch.setAlpha(90)
                p.fillRect(QtCore.QRectF(x0, zero_y - 14, x1 - x0, 28),
                           QtGui.QBrush(hatch, QtCore.Qt.BDiagPattern))
            p.setPen(col.darker(130) if pal.color(QtGui.QPalette.Base).lightness() > 128 else col.lighter(130))
            p.setFont(small)
            p.drawText(QtCore.QRectF(x0 - 30, plot.top() + 2 + stagger, x1 - x0 + 60, 34),
                       QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop, label)
            p.drawText(QtCore.QRectF(x0 - 30, plot.bottom() - 34 - stagger, x1 - x0 + 60, 34),
                       QtCore.Qt.AlignHCenter | QtCore.Qt.AlignBottom, self._value_text(key, us))
        for x0, x1, amp, col in second:
            shade = QtGui.QColor(col)
            shade.setAlpha(20)
            p.fillRect(QtCore.QRectF(x0, plot.top(), x1 - x0, plot.height()), shade)

        # zero line
        p.setPen(QtGui.QPen(fg, 1, QtCore.Qt.DashLine))
        p.drawLine(QtCore.QPointF(plot.left(), zero_y), QtCore.QPointF(plot.right(), zero_y))
        p.setFont(small)
        p.setPen(fg)
        p.drawText(QtCore.QRectF(0, zero_y - 9, left - 8, 18),
                   QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, "0 µA")

        # amplitude guides
        for amp_signed, amp, name, col in ((-b.cathodic, b.cathodic, "cathodic", CATHODIC_COLOUR),
                                           (b.anodic, b.anodic, "anodic", ANODIC_COLOUR)):
            y = ypx(amp_signed)
            p.setPen(QtGui.QPen(col, 1, QtCore.Qt.DotLine))
            p.drawLine(QtCore.QPointF(plot.left(), y), QtCore.QPointF(plot.right(), y))
            p.setPen(col)
            sign = "−" if amp_signed < 0 else "+"
            p.drawText(QtCore.QRectF(0, y - 16, left - 8, 32),
                       QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
                       f"{name}\n{sign}{amp} µA")

        # waveform
        path = QtGui.QPainterPath(QtCore.QPointF(plot.left(), zero_y))
        for x0, x1, amp, *_ in segs:
            path.lineTo(x0, zero_y)
            path.lineTo(x0, ypx(amp))
            path.lineTo(x1, ypx(amp))
            path.lineTo(x1, zero_y)
        for x0, x1, amp, _ in second:
            path.lineTo(x0, zero_y)
            path.lineTo(x0, ypx(amp))
            path.lineTo(x1, ypx(amp))
            path.lineTo(x1, zero_y)
        path.lineTo(plot.right(), zero_y)
        p.setPen(QtGui.QPen(fg, 2.2))
        p.setBrush(QtCore.Qt.NoBrush)
        p.drawPath(path)

        # break in the compressed gap between pulses
        if rest_broken:
            mx = (pulse_end_x + rest_end_x) / 2
            p.setPen(QtGui.QPen(pal.color(QtGui.QPalette.Base), 7))
            p.drawLine(QtCore.QPointF(mx - 3, zero_y), QtCore.QPointF(mx + 3, zero_y))
            p.setPen(QtGui.QPen(fg, 1.5))
            for dx in (-4, 4):
                p.drawLine(QtCore.QPointF(mx + dx - 4, zero_y + 8), QtCore.QPointF(mx + dx + 4, zero_y - 8))
        if repeats or b.count == 1:
            p.setFont(small)
            p.setPen(fg)
            p.drawText(QtCore.QRectF(pulse_end_x, zero_y + 10, rest_end_x - pulse_end_x, 36),
                       QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop,
                       "stimulators off" if repeats else "done")

        # period dimension line
        if repeats and segs:
            y = plot.bottom() + 52
            x0, x1 = segs[0][0], rest_end_x
            self._dimension(p, x0, x1, y, fg)
            freq = 1e6 / d["period"] if d["period"] > 0 else 0
            p.setFont(self.font())
            p.setPen(fg)
            p.drawText(QtCore.QRectF(x0, y + 4, max(x1 - x0, 200), 22), QtCore.Qt.AlignHCenter,
                       f"period {self._value_text('period', d['period']).replace(chr(10), ', ')}"
                       f"  ({freq:.1f} Hz)")
        # pulse length dimension line
        if segs:
            y = plot.bottom() + 24
            self._dimension(p, segs[0][0], segs[-1][1], y, fg)
            p.setFont(small)
            p.setPen(fg)
            p.drawText(QtCore.QRectF(segs[0][0], y - 18, segs[-1][1] - segs[0][0], 16),
                       QtCore.Qt.AlignHCenter, f"pulse {pulse_us:.0f} µs")
        p.end()

    @staticmethod
    def _dimension(p: QtGui.QPainter, x0: float, x1: float, y: float, colour: QtGui.QColor) -> None:
        p.setPen(QtGui.QPen(colour, 1))
        p.drawLine(QtCore.QPointF(x0, y), QtCore.QPointF(x1, y))
        for x, s in ((x0, 1), (x1, -1)):
            p.drawLine(QtCore.QPointF(x, y), QtCore.QPointF(x + 6 * s, y - 4))
            p.drawLine(QtCore.QPointF(x, y), QtCore.QPointF(x + 6 * s, y + 4))
            p.drawLine(QtCore.QPointF(x, y - 6), QtCore.QPointF(x, y + 6))
