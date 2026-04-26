import serial
import struct
import time


MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"

# TI mmWave demo frame header:
# magicWord[8]
# version
# totalPacketLen
# platform
# frameNumber
# timeCpuCycles
# numDetectedObj
# numTLVs
# subFrameNumber
FRAME_HEADER_FORMAT = "<Q8I"
FRAME_HEADER_SIZE = struct.calcsize(FRAME_HEADER_FORMAT)

# TLV header:
# type(uint32), length(uint32)
TLV_HEADER_FORMAT = "<II"
TLV_HEADER_SIZE = struct.calcsize(TLV_HEADER_FORMAT)


TLV_TYPE_DETECTED_POINTS = 1
TLV_TYPE_RANGE_PROFILE = 2
TLV_TYPE_NOISE_PROFILE = 3
TLV_TYPE_AZIMUTH_STATIC_HEATMAP = 4
TLV_TYPE_RANGE_DOPPLER_HEATMAP = 5
TLV_TYPE_STATS = 6
TLV_TYPE_SIDE_INFO = 7
TLV_TYPE_TEMPERATURE_STATS = 9

PACKET_PAD_ALIGNMENT = 32
MAX_PACKET_LEN = 65536
MAX_TLVS = 64

DEFAULT_NAV_CONFIG = {
    "filter_min_y": 0.15,
    "filter_max_y": 1.5,
    "filter_lateral_limit": 0.60,
    "filter_min_snr_raw": 180,
    "ahead_min_y": 0.15,
    "ahead_lateral_limit": 0.35,
    "danger_box": {
        "x_min": -0.30,
        "x_max": 0.30,
        "y_min": 0.15,
        "y_max": 0.80,
    },
    "density_y_min": 0.15,
    "density_y_max": 1.5,
}


def send_cfg(cfg_port: str, cfg_path: str, baud: int = 115200, line_delay: float = 0.05) -> None:
    """
    Send a TI .cfg file line by line over the CLI / CFG UART.
    Skips blank lines and lines starting with '%'.
    """
    cfg_lines = []
    with open(cfg_path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("%"):
                continue
            cfg_lines.append(line)

    print(f"[CFG] Opening {cfg_port} @ {baud}")
    with serial.Serial(cfg_port, baudrate=baud, timeout=0.25) as ser:
        time.sleep(0.5)

        for line in cfg_lines:
            ser.write((line + "\n").encode("utf-8"))
            ser.flush()
            print(f"[CFG] >> {line}")

            time.sleep(line_delay)

            resp = ser.read_all()
            if resp:
                try:
                    print("[CFG] <<", resp.decode("utf-8", errors="replace").strip())
                except Exception:
                    print("[CFG] <<", resp)

        print("[CFG] Config send complete.")


def hexdump(data: bytes, max_len: int = 32) -> str:
    shown = data[:max_len]
    s = " ".join(f"{b:02X}" for b in shown)
    if len(data) > max_len:
        s += " ..."
    return s


def tlv_name(tlv_type: int) -> str:
    names = {
        TLV_TYPE_DETECTED_POINTS: "DETECTED_POINTS",
        TLV_TYPE_RANGE_PROFILE: "RANGE_PROFILE",
        TLV_TYPE_NOISE_PROFILE: "NOISE_PROFILE",
        TLV_TYPE_AZIMUTH_STATIC_HEATMAP: "AZIMUTH_STATIC_HEATMAP",
        TLV_TYPE_RANGE_DOPPLER_HEATMAP: "RANGE_DOPPLER_HEATMAP",
        TLV_TYPE_STATS: "STATS",
        TLV_TYPE_SIDE_INFO: "SIDE_INFO",
        TLV_TYPE_TEMPERATURE_STATS: "TEMPERATURE_STATS",
    }
    return names.get(tlv_type, f"UNKNOWN_{tlv_type}")


def decode_detected_points(value: bytes):
    if len(value) % 16 != 0:
        return []

    points = []
    for i in range(0, len(value), 16):
        x, y, z, doppler = struct.unpack_from("<4f", value, i)
        points.append({
            "x": x,
            "y": y,
            "z": z,
            "doppler": doppler,
        })
    return points


def decode_side_info(value: bytes):
    if len(value) % 4 != 0:
        return []

    entries = []
    for i in range(0, len(value), 4):
        snr, noise = struct.unpack_from("<hh", value, i)
        entries.append({
            "snr_raw": snr,
            "noise_raw": noise,
        })
    return entries


def decode_temperature_stats(value: bytes):
    if len(value) != 28:
        return None
    return {
        "raw_u16": list(struct.unpack("<14H", value)),
    }


def combine_points_with_side_info(points, side_info):
    combined = []
    count = len(points)
    for i in range(count):
        entry = dict(points[i])
        if i < len(side_info):
            entry.update(side_info[i])
        combined.append(entry)
    return combined


def merge_nav_config(overrides=None):
    config = {
        "filter_min_y": DEFAULT_NAV_CONFIG["filter_min_y"],
        "filter_max_y": DEFAULT_NAV_CONFIG["filter_max_y"],
        "filter_lateral_limit": DEFAULT_NAV_CONFIG["filter_lateral_limit"],
        "filter_min_snr_raw": DEFAULT_NAV_CONFIG["filter_min_snr_raw"],
        "ahead_min_y": DEFAULT_NAV_CONFIG["ahead_min_y"],
        "ahead_lateral_limit": DEFAULT_NAV_CONFIG["ahead_lateral_limit"],
        "danger_box": dict(DEFAULT_NAV_CONFIG["danger_box"]),
        "density_y_min": DEFAULT_NAV_CONFIG["density_y_min"],
        "density_y_max": DEFAULT_NAV_CONFIG["density_y_max"],
    }
    if not overrides:
        return config

    for key, value in overrides.items():
        if key == "danger_box" and isinstance(value, dict):
            config["danger_box"].update(value)
        else:
            config[key] = value
    return config


def filter_points_for_navigation(
    points,
    min_y: float = 0.15,
    max_y: float = 1.5,
    lateral_limit: float = 0.60,
    min_snr_raw=None,
):
    filtered = []
    for point in points:
        if point["y"] < min_y:
            continue
        if max_y is not None and point["y"] > max_y:
            continue
        if lateral_limit is not None and abs(point["x"]) > lateral_limit:
            continue
        if min_snr_raw is not None and point.get("snr_raw", float("-inf")) < min_snr_raw:
            continue
        filtered.append(point)
    return filtered


def nearest_point_ahead(points, min_y: float = 0.0, lateral_limit: float = 0.5):
    candidates = [
        p for p in points
        if p["y"] >= min_y and abs(p["x"]) <= lateral_limit
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda p: p["y"])


def count_points_in_box(points, x_min: float, x_max: float, y_min: float, y_max: float):
    return sum(
        1
        for p in points
        if x_min <= p["x"] <= x_max and y_min <= p["y"] <= y_max
    )


def left_right_obstacle_density(points, y_min: float = 0.0, y_max: float = 1.5):
    left = sum(1 for p in points if y_min <= p["y"] <= y_max and p["x"] < 0.0)
    right = sum(1 for p in points if y_min <= p["y"] <= y_max and p["x"] > 0.0)
    return {"left": left, "right": right}


def build_navigation_summary(points, nav_config=None):
    config = merge_nav_config(nav_config)
    filtered_points = filter_points_for_navigation(
        points,
        min_y=config["filter_min_y"],
        max_y=config["filter_max_y"],
        lateral_limit=config["filter_lateral_limit"],
        min_snr_raw=config["filter_min_snr_raw"],
    )
    nearest = nearest_point_ahead(
        filtered_points,
        min_y=config["ahead_min_y"],
        lateral_limit=config["ahead_lateral_limit"],
    )
    danger_box = config["danger_box"]
    return {
        "config": config,
        "filtered_points": filtered_points,
        "filtered_count": len(filtered_points),
        "nearest_ahead": nearest,
        "danger_box_count": count_points_in_box(
            filtered_points,
            x_min=danger_box["x_min"],
            x_max=danger_box["x_max"],
            y_min=danger_box["y_min"],
            y_max=danger_box["y_max"],
        ),
        "left_right_density": left_right_obstacle_density(
            filtered_points,
            y_min=config["density_y_min"],
            y_max=config["density_y_max"],
        ),
    }


def decode_frame_summary(frame, nav_config=None):
    tlvs_by_type = {}
    points = []
    side_info = []

    for tlv in frame["tlvs"]:
        tlvs_by_type[tlv["type"]] = tlv
        if tlv["type"] == TLV_TYPE_DETECTED_POINTS:
            points = tlv["decoded"] or []
        elif tlv["type"] == TLV_TYPE_SIDE_INFO:
            side_info = tlv["decoded"] or []

    combined_points = combine_points_with_side_info(points, side_info)
    navigation = build_navigation_summary(combined_points, nav_config=nav_config)

    return {
        "header": frame["header"],
        "tlvs": frame["tlvs"],
        "tlvs_by_type": tlvs_by_type,
        "tlv_length_mode": frame.get("tlv_length_mode"),
        "trailing_padding": frame.get("trailing_padding", 0),
        "points": points,
        "side_info": side_info,
        "combined_points": combined_points,
        "range_profile": tlvs_by_type.get(TLV_TYPE_RANGE_PROFILE, {}).get("value"),
        "noise_profile": tlvs_by_type.get(TLV_TYPE_NOISE_PROFILE, {}).get("value"),
        "stats": tlvs_by_type.get(TLV_TYPE_STATS, {}).get("value"),
        "temperature_stats": tlvs_by_type.get(TLV_TYPE_TEMPERATURE_STATS, {}).get("decoded"),
        "navigation": navigation,
        "raw_packet": frame.get("raw_packet"),
    }


def _padding_is_reasonable(remaining_bytes: int, packet_len: int) -> bool:
    if remaining_bytes < 0:
        return False
    if remaining_bytes == 0:
        return True
    expected_pad = (-((packet_len - remaining_bytes) % PACKET_PAD_ALIGNMENT)) % PACKET_PAD_ALIGNMENT
    return remaining_bytes == expected_pad


def _parse_tlvs_with_mode(
    packet: bytes,
    num_tlvs: int,
    packet_len: int,
    length_mode: str,
):
    tlvs = []
    offset = FRAME_HEADER_SIZE

    for tlv_index in range(num_tlvs):
        if offset + TLV_HEADER_SIZE > packet_len:
            return None

        tlv_type, tlv_length = struct.unpack_from(TLV_HEADER_FORMAT, packet, offset)
        offset += TLV_HEADER_SIZE

        if length_mode == "payload":
            value_len = tlv_length
        elif length_mode == "inclusive":
            value_len = tlv_length - TLV_HEADER_SIZE
        else:
            raise ValueError(f"Unsupported TLV length mode: {length_mode}")

        if value_len < 0 or offset + value_len > packet_len:
            return None

        value = packet[offset: offset + value_len]
        offset += value_len

        decoded = None
        if tlv_type == TLV_TYPE_DETECTED_POINTS:
            decoded = decode_detected_points(value)
        elif tlv_type == TLV_TYPE_SIDE_INFO:
            decoded = decode_side_info(value)
        elif tlv_type == TLV_TYPE_TEMPERATURE_STATS:
            decoded = decode_temperature_stats(value)

        tlvs.append({
            "type": tlv_type,
            "type_name": tlv_name(tlv_type),
            "length": tlv_length,
            "value_length": value_len,
            "value": value,
            "decoded": decoded,
            "tlv_index": tlv_index,
        })

    trailing = packet_len - offset
    if not _padding_is_reasonable(trailing, packet_len):
        return None

    return {
        "tlvs": tlvs,
        "length_mode": length_mode,
        "trailing_padding": trailing,
    }


def parse_tlvs(packet: bytes, num_tlvs: int, packet_len: int):
    for mode in ("payload", "inclusive"):
        parsed = _parse_tlvs_with_mode(packet, num_tlvs, packet_len, mode)
        if parsed is not None:
            return parsed
    raise ValueError("Unable to parse TLVs with either payload or inclusive length mode")


class MmwaveUartParser:
    def __init__(self, data_port: str, baud: int = 921600, timeout: float = 0.1):
        self.ser = serial.Serial(data_port, baudrate=baud, timeout=timeout)
        self.buffer = bytearray()

    def close(self):
        self.ser.close()

    def _read_more(self):
        chunk = self.ser.read(4096)
        if chunk:
            self.buffer.extend(chunk)
            return True
        return False

    def _find_magic(self) -> bool:
        while True:
            idx = self.buffer.find(MAGIC_WORD)
            if idx >= 0:
                if idx > 0:
                    del self.buffer[:idx]
                return True

            if len(self.buffer) > len(MAGIC_WORD):
                del self.buffer[:-len(MAGIC_WORD)]

            if not self._read_more():
                return False

    def _fill_until(self, min_bytes: int) -> bool:
        while len(self.buffer) < min_bytes:
            if not self._read_more():
                return False
        return True

    def _header_is_reasonable(self, header) -> bool:
        total_len = header["total_packet_len"]
        num_tlvs = header["num_tlvs"]
        subframe = header["subframe_number"]

        if total_len < FRAME_HEADER_SIZE:
            return False
        if total_len > MAX_PACKET_LEN:
            return False
        if total_len % PACKET_PAD_ALIGNMENT != 0:
            return False
        if num_tlvs > MAX_TLVS:
            return False
        if total_len < FRAME_HEADER_SIZE + num_tlvs * TLV_HEADER_SIZE:
            return False
        if subframe > 3:
            return False
        if header["version"] == 0:
            return False
        return True

    def _drop_until_next_magic(self):
        next_idx = self.buffer.find(MAGIC_WORD, 1)
        if next_idx >= 0:
            del self.buffer[:next_idx]
        elif self.buffer:
            del self.buffer[0]

    def read_frame(self):
        for _ in range(8):
            if not self._find_magic():
                return None

            if not self._fill_until(FRAME_HEADER_SIZE):
                return None

            header_bytes = bytes(self.buffer[:FRAME_HEADER_SIZE])
            unpacked = struct.unpack(FRAME_HEADER_FORMAT, header_bytes)

            header = {
                "magic": unpacked[0],
                "version": unpacked[1],
                "total_packet_len": unpacked[2],
                "platform": unpacked[3],
                "frame_number": unpacked[4],
                "time_cpu_cycles": unpacked[5],
                "num_detected_obj": unpacked[6],
                "num_tlvs": unpacked[7],
                "subframe_number": unpacked[8],
            }

            if not self._header_is_reasonable(header):
                self._drop_until_next_magic()
                continue

            total_len = header["total_packet_len"]
            if not self._fill_until(total_len):
                return None

            packet = bytes(self.buffer[:total_len])

            try:
                parsed_tlvs = parse_tlvs(packet, header["num_tlvs"], total_len)
            except ValueError:
                self._drop_until_next_magic()
                continue

            del self.buffer[:total_len]
            return {
                "header": header,
                "tlvs": parsed_tlvs["tlvs"],
                "tlv_length_mode": parsed_tlvs["length_mode"],
                "trailing_padding": parsed_tlvs["trailing_padding"],
                "raw_packet": packet,
            }

        return None

    def read_decoded_frame(self, nav_config=None):
        frame = self.read_frame()
        if frame is None:
            return None
        return decode_frame_summary(frame, nav_config=nav_config)


def print_frame(frame, max_points_to_show: int = 5):
    decoded = decode_frame_summary(frame)
    h = decoded["header"]
    print(
        f"\n[FRAME] "
        f"frame_number={h['frame_number']} "
        f"packet_len={h['total_packet_len']} "
        f"num_detected_obj={h['num_detected_obj']} "
        f"num_tlvs={h['num_tlvs']} "
        f"subframe={h['subframe_number']}"
    )
    print(
        f"  parse: tlv_length_mode={decoded.get('tlv_length_mode')} "
        f"trailing_padding={decoded.get('trailing_padding', 0)}"
    )

    points = decoded["points"]
    side_info = decoded["side_info"]

    for i, tlv in enumerate(decoded["tlvs"]):
        print(
            f"  TLV {i}: type={tlv['type']} ({tlv['type_name']}), "
            f"length={tlv['length']}, value_length={tlv['value_length']}"
        )

        if tlv["type"] == TLV_TYPE_DETECTED_POINTS:
            points = tlv["decoded"] or []
            print(f"    decoded_points={len(points)}")
            for p in points[:max_points_to_show]:
                print(
                    f"    x={p['x']:.3f}, y={p['y']:.3f}, "
                    f"z={p['z']:.3f}, doppler={p['doppler']:.3f}"
                )

        elif tlv["type"] == TLV_TYPE_SIDE_INFO:
            side_info = tlv["decoded"] or []
            print(f"    side_info_entries={len(side_info)}")
            for s in side_info[:max_points_to_show]:
                print(f"    snr_raw={s['snr_raw']}, noise_raw={s['noise_raw']}")

        elif tlv["type"] == TLV_TYPE_TEMPERATURE_STATS and tlv["decoded"] is not None:
            print(f"    raw_u16={tlv['decoded']['raw_u16']}")

        else:
            print(f"    payload_preview={hexdump(tlv['value'], max_len=24)}")

    if points is not None and side_info is not None:
        combined = decoded["combined_points"]
        n = min(len(combined), max_points_to_show)
        print("  Combined point preview:")
        for i in range(n):
            p = combined[i]
            print(
                f"    pt{i}: "
                f"x={p['x']:.3f}, y={p['y']:.3f}, z={p['z']:.3f}, doppler={p['doppler']:.3f}, "
                f"snr_raw={p['snr_raw']}, noise_raw={p['noise_raw']}"
            )
        nav = decoded["navigation"]
        ahead = nav["nearest_ahead"]
        print(f"  Navigation filter count={nav['filtered_count']}/{len(combined)}")
        if ahead:
            print(
                f"  Navigation helpers: nearest_ahead_y={ahead['y']:.3f} "
                f"nearest_ahead_x={ahead['x']:.3f}"
            )
        else:
            print("  Navigation helpers: nearest_ahead=None")
        print(
            f"  danger_box_count={nav['danger_box_count']} "
            f"left_density={nav['left_right_density']['left']} "
            f"right_density={nav['left_right_density']['right']}"
        )


def _build_test_packet(points, side_info, extra_tlvs=None, frame_number: int = 7):
    extra_tlvs = extra_tlvs or []
    tlv_payloads = [
        (TLV_TYPE_DETECTED_POINTS, b"".join(struct.pack("<4f", p["x"], p["y"], p["z"], p["doppler"]) for p in points)),
        (TLV_TYPE_SIDE_INFO, b"".join(struct.pack("<hh", s["snr_raw"], s["noise_raw"]) for s in side_info)),
    ]
    tlv_payloads.extend(extra_tlvs)

    tlv_blob = bytearray()
    for tlv_type, payload in tlv_payloads:
        tlv_blob.extend(struct.pack("<II", tlv_type, len(payload)))
        tlv_blob.extend(payload)

    unpadded_len = FRAME_HEADER_SIZE + len(tlv_blob)
    total_packet_len = (unpadded_len + PACKET_PAD_ALIGNMENT - 1) // PACKET_PAD_ALIGNMENT * PACKET_PAD_ALIGNMENT

    header = struct.pack(
        FRAME_HEADER_FORMAT,
        int.from_bytes(MAGIC_WORD, "little"),
        0x03060002,
        total_packet_len,
        0xA1843,
        frame_number,
        123456,
        len(points),
        len(tlv_payloads),
        0,
    )
    packet = bytearray(header)
    packet.extend(tlv_blob)
    packet.extend(b"\x00" * (total_packet_len - len(packet)))
    return bytes(packet)


def run_self_test():
    points = [
        {"x": 0.1, "y": 0.5, "z": 0.0, "doppler": 0.0},
        {"x": -0.2, "y": 0.8, "z": 0.0, "doppler": 0.1},
    ]
    side_info = [
        {"snr_raw": 200, "noise_raw": 900},
        {"snr_raw": 150, "noise_raw": 850},
    ]
    temp_payload = struct.pack("<14H", *range(14))
    packet = _build_test_packet(points, side_info, extra_tlvs=[(TLV_TYPE_TEMPERATURE_STATS, temp_payload)], frame_number=7)
    parsed = parse_tlvs(packet, num_tlvs=3, packet_len=len(packet))

    assert parsed["length_mode"] == "payload"
    assert parsed["trailing_padding"] >= 0
    assert len(parsed["tlvs"]) == 3
    decoded_points = parsed["tlvs"][0]["decoded"]
    for expected, actual in zip(points, decoded_points):
        for key in ("x", "y", "z", "doppler"):
            assert abs(expected[key] - actual[key]) < 1e-6
    assert parsed["tlvs"][1]["decoded"] == side_info
    assert parsed["tlvs"][2]["decoded"]["raw_u16"] == list(range(14))

    combined = combine_points_with_side_info(points, side_info)
    assert nearest_point_ahead(combined)["y"] == 0.5
    assert count_points_in_box(combined, -0.3, 0.3, 0.0, 0.9) == 2
    assert left_right_obstacle_density(combined, 0.0, 1.0) == {"left": 1, "right": 1}

    frame = {
        "header": {
            "frame_number": 7,
            "total_packet_len": len(packet),
            "num_detected_obj": len(points),
            "num_tlvs": 3,
            "subframe_number": 0,
        },
        "tlvs": parsed["tlvs"],
        "tlv_length_mode": parsed["length_mode"],
        "trailing_padding": parsed["trailing_padding"],
        "raw_packet": packet,
    }
    decoded = decode_frame_summary(
        frame,
        nav_config={
            "filter_min_y": 0.2,
            "filter_min_snr_raw": 175,
            "ahead_lateral_limit": 0.25,
        },
    )
    assert len(decoded["combined_points"]) == 2
    assert decoded["navigation"]["filtered_count"] == 1
    assert abs(decoded["navigation"]["nearest_ahead"]["x"] - 0.1) < 1e-6
    assert decoded["navigation"]["danger_box_count"] == 1
    assert decoded["navigation"]["left_right_density"] == {"left": 0, "right": 1}

    class FakeSerial:
        def __init__(self, payloads):
            self.payloads = list(payloads)

        def read(self, _size):
            if self.payloads:
                return self.payloads.pop(0)
            return b""

        def close(self):
            return None

    corrupt_packet = bytearray(packet)
    struct.pack_into("<I", corrupt_packet, FRAME_HEADER_SIZE + 4, 999999)
    good_packet = _build_test_packet(points, side_info, extra_tlvs=[(TLV_TYPE_TEMPERATURE_STATS, temp_payload)], frame_number=8)

    parser = MmwaveUartParser.__new__(MmwaveUartParser)
    parser.ser = FakeSerial([bytes(corrupt_packet) + good_packet])
    parser.buffer = bytearray()
    recovered = parser.read_frame()
    assert recovered is not None
    assert recovered["header"]["frame_number"] == 8
    print("[SELFTEST] PASS")
