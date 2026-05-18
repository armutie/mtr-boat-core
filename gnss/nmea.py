from __future__ import annotations

import time
from dataclasses import asdict, dataclass


@dataclass
class GnssFix:
    timestamp: float
    source: str
    fix: str = "none"
    lat: float | None = None
    lon: float | None = None
    altitude_m: float | None = None
    speed_mps: float | None = None
    heading_deg: float | None = None
    satellites: int | None = None
    hdop: float | None = None
    raw: str = ""

    def to_record(self) -> dict:
        return asdict(self)


def _safe_float(value) -> float | None:
    if value in ("", None):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _safe_int(value) -> int | None:
    if value in ("", None):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _fix_label_gga(quality) -> str:
    if str(quality) in ("1", "2", "4", "5"):
        return "fix"
    if str(quality) == "6":
        return "estimated"
    return "none"


def parse_nmea_sentence(sentence: str, source: str = "nmea", timestamp: float | None = None) -> GnssFix | None:
    import pynmea2

    try:
        msg = pynmea2.parse(sentence.strip())
    except pynmea2.ParseError:
        return None

    now = time.time() if timestamp is None else timestamp

    if isinstance(msg, pynmea2.types.talker.GGA):
        return GnssFix(
            timestamp=now,
            source=source,
            fix=_fix_label_gga(msg.gps_qual),
            lat=_safe_float(msg.latitude),
            lon=_safe_float(msg.longitude),
            altitude_m=_safe_float(msg.altitude),
            satellites=_safe_int(msg.num_sats),
            hdop=_safe_float(msg.horizontal_dil),
            raw=sentence.strip(),
        )

    if isinstance(msg, pynmea2.types.talker.RMC):
        speed_knots = _safe_float(msg.spd_over_grnd)
        return GnssFix(
            timestamp=now,
            source=source,
            fix="fix" if msg.status == "A" else "none",
            lat=_safe_float(msg.latitude),
            lon=_safe_float(msg.longitude),
            speed_mps=speed_knots * 0.514444 if speed_knots is not None else None,
            heading_deg=_safe_float(msg.true_course),
            raw=sentence.strip(),
        )

    return None


class NmeaReader:
    def __init__(self, port: str, baud: int = 9600, timeout: float = 1.0):
        import serial

        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)

    def read_fix(self) -> GnssFix | None:
        line = self.ser.readline().decode("ascii", errors="ignore").strip()
        if not line:
            return None
        return parse_nmea_sentence(line, source=self.ser.port)

    def close(self) -> None:
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
