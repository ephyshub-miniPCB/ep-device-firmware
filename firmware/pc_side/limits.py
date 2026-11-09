"""Host-side copies of the ep_device's recording and stimulation checks.

The GUI uses these to warn about bad parameters before anything is sent. The
firmware still has the final say; keep these in sync with the sources named on
each function.
"""
from dataclasses import dataclass

NUM_CHANNELS = 16
VALID_BITS = (8, 10, 12, 16)

# modules/rhs2116/include/rhs2116/rhs2116_stream.h
TICK_HZ = 16_000_000
MIN_SLOT_TICKS = 88
OVERHEAD_SLOTS = 4
MIN_RATE_HZ = 100

# ep_device/src/recording.h, CONFIG_BT_L2CAP_TX_MTU (247) minus the ATT header
SAMPLES_HDR_LEN = 12
BLE_PAYLOAD_MAX = 244


def mask_channels(mask: int) -> list[int]:
    return [ch for ch in range(NUM_CHANNELS) if mask & (1 << ch)]


def max_rate_hz(n_ch: int) -> int:
    """rhs2116_stream.c: rhs2116_stream_max_rate_hz()"""
    return TICK_HZ // (MIN_SLOT_TICKS * (max(n_ch, 1) + OVERHEAD_SLOTS))


def actual_rate_mhz(n_ch: int, rate_hz: int) -> int | None:
    """rhs2116_stream.c: rhs2116_stream_plan(). Per-channel rate in milli-Hz the
    chip will really run at, or None if the request is out of range."""
    if n_ch == 0 or rate_hz < MIN_RATE_HZ:
        return None
    slots = n_ch + OVERHEAD_SLOTS
    slot_ticks = TICK_HZ // (rate_hz * slots)
    if slot_ticks < MIN_SLOT_TICKS:
        return None
    return TICK_HZ * 1000 // (slot_ticks * slots)


def required_bps(n_ch: int, rate_mhz: int, bits: int, payload_max: int = BLE_PAYLOAD_MAX) -> int:
    """recording.c: recording_start(). BLE payload bytes/s the recording needs."""
    fpp = (payload_max - SAMPLES_HDR_LEN) * 8 // (n_ch * bits)
    fpp = min(max(fpp, 1), 255)
    pkt_len = SAMPLES_HDR_LEN + (fpp * n_ch * bits + 7) // 8
    return -(-rate_mhz * pkt_len // (1000 * fpp))


@dataclass
class LinkResult:
    """Last BLE link test, in payload bytes/s."""
    mean_bps: int
    p10_bps: int


@dataclass
class RecordingCheck:
    errors: list[str]    # the firmware would reject these
    warnings: list[str]  # it would run, but probably lose data
    notes: list[str]
    rate_mhz: int | None = None
    max_rate_hz: int = 0
    required_bps: int = 0


def check_recording(mask: int, rate_hz: int | None, bits: int,
                    link: LinkResult | None) -> RecordingCheck:
    n_ch = len(mask_channels(mask))
    c = RecordingCheck([], [], [], max_rate_hz=max_rate_hz(n_ch))

    if n_ch == 0:
        c.errors.append("Select at least one channel.")
    if rate_hz is None:
        c.errors.append("Enter a sample rate.")
    elif rate_hz < MIN_RATE_HZ:
        c.errors.append(f"Rate must be at least {MIN_RATE_HZ} S/s.")
    elif n_ch and rate_hz > c.max_rate_hz:
        c.errors.append(f"Rate exceeds the SPI limit of {c.max_rate_hz} S/s per channel "
                        f"for {n_ch} channel{'s' if n_ch > 1 else ''}.")
    if bits not in VALID_BITS:
        c.errors.append("Bits must be 8, 10, 12 or 16.")
    if c.errors:
        return c

    c.rate_mhz = actual_rate_mhz(n_ch, rate_hz)
    c.required_bps = required_bps(n_ch, c.rate_mhz, bits)

    # shell_cmds.c: check_ble_budget()
    if link is None:
        c.notes.append(f"BLE capacity untested: run a link test to check that "
                       f"{c.required_bps} B/s fits.")
    elif c.required_bps > link.mean_bps:
        c.warnings.append(f"Needs {c.required_bps} B/s but the last link test averaged "
                          f"{link.mean_bps} B/s; expect continuous packet loss.")
    elif c.required_bps > link.p10_bps:
        c.warnings.append(f"Needs {c.required_bps} B/s, above the link test's 10th "
                          f"percentile ({link.p10_bps} B/s); expect loss during link dips.")
    else:
        c.notes.append(f"BLE budget: {c.required_bps} B/s of {link.p10_bps} B/s "
                       f"(10th percentile of last link test).")
    return c


# -- STIMULATION --

@dataclass
class Biphasic:
    """Parameters of 'stim biphasic' (shell_cmds.c: cmd_stim_biphasic)."""
    mask: int = 0x0001
    cathodic: int = 10      # stim steps (1 uA by default)
    anodic: int = 10
    phase1_us: int = 200
    interphase_us: int = 100
    phase2_us: int = 200
    recovery_us: int = 500
    period_us: int = 10_000
    count: int = 0          # 0 = until stopped
    anodic_first: bool = False
    allow_unbalanced: bool = False

    def shell_command(self) -> str:
        cmd = (f"stim biphasic 0x{self.mask:04x} {self.cathodic} {self.anodic} "
               f"{self.phase1_us} {self.interphase_us} {self.phase2_us} {self.recovery_us} "
               f"{self.period_us} {self.count}")
        if self.anodic_first:
            cmd += " anodic"
        if self.allow_unbalanced:
            cmd += " unbalanced"
        return cmd


@dataclass
class BiphasicFrames:
    phase1: int
    interphase: int
    phase2: int
    recovery: int
    period: int


def us_to_frames(us: int, rate_mhz: int) -> int:
    """stim.c: us_to_frames(), rounds to the nearest frame."""
    return (us * rate_mhz + 500_000_000) // 1_000_000_000


def check_biphasic(b: Biphasic, rate_mhz: int | None) -> tuple[list[str], BiphasicFrames | None]:
    """Returns (errors, frame counts). Frame checks (stim.c: stim_load_biphasic()
    and validate()) need the recording's actual rate, so they are skipped without one."""
    errors = []
    if b.mask == 0:
        errors.append("Select at least one channel.")
    if not (0 <= b.cathodic <= 255 and 0 <= b.anodic <= 255):
        errors.append("Amplitudes are 0-255 steps.")
    if any(not 0 <= v <= 0xFFFF for v in (b.phase1_us, b.interphase_us, b.phase2_us,
                                          b.recovery_us, b.count)):
        errors.append("Times are 0-65535 us and count is 0-65535.")
    if rate_mhz is None:
        errors.append("Start recording first: stimulation is timed by the sample stream.")
        return errors, None

    f = BiphasicFrames(*(us_to_frames(us, rate_mhz) for us in (
        b.phase1_us, b.interphase_us, b.phase2_us, b.recovery_us, b.period_us)))
    if f.phase1 == 0 or f.phase2 == 0:
        errors.append("A phase rounds to zero frames at this sample rate; lengthen it or "
                      "record faster.")
    pulse = f.phase1 + f.interphase + f.phase2 + f.recovery
    if b.count != 1 and f.period <= pulse:
        errors.append("Period must be longer than the pulse (phases + interphase + recovery).")
    if not b.allow_unbalanced:
        first, second = (b.anodic, b.cathodic) if b.anodic_first else (b.cathodic, b.anodic)
        if first * f.phase1 != second * f.phase2:
            errors.append(f"Unbalanced charge: {first} x {f.phase1} frames vs "
                          f"{second} x {f.phase2} frames (phases are rounded to whole frames). "
                          "Tick 'Allow unbalanced' to override.")
    return errors, f
