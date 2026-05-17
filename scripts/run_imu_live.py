from __future__ import annotations

import argparse
import json
import time
from collections import deque
from datetime import datetime
from pathlib import Path

from boat_core.config import choose, load_boat_config, section
from imu import GyroBias, ImuSample, Mpu6050


def apply_config(args) -> None:
    config = load_boat_config(args.config)
    imu = section(config, "imu")
    args.bus = choose(args.bus, imu, "bus", 2)
    args.address = choose(args.address, imu, "address", "0x68")


def parse_address(value: int | str) -> int:
    if isinstance(value, int):
        return value
    return int(value, 0)


def default_log_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"imu_{stamp}.jsonl"


def print_sample(sample: ImuSample) -> None:
    print(
        "[IMU] "
        f"accel_g=({sample.accel_x_g:+.3f},{sample.accel_y_g:+.3f},{sample.accel_z_g:+.3f}) "
        f"|a|={sample.accel_mag_g:.3f} "
        f"gyro_dps=({sample.gyro_x_dps:+.2f},{sample.gyro_y_dps:+.2f},{sample.gyro_z_dps:+.2f})"
    )


class ImuViz:
    def __init__(self, max_history: int = 250):
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError("pygame is required for --viz. Install it with: python -m pip install pygame") from exc

        self.pygame = pygame
        self.max_history = max_history
        pygame.init()
        self.width = 1000
        self.height = 720
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("MPU-6050 IMU Trace")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 26)
        self.big_font = pygame.font.SysFont(None, 38)
        self.xy_trace = deque(maxlen=max_history)
        self.ax_hist = deque(maxlen=max_history)
        self.ay_hist = deque(maxlen=max_history)
        self.az_hist = deque(maxlen=max_history)

    def update(self, sample: ImuSample, bias: GyroBias) -> bool:
        pygame = self.pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

        self.ax_hist.append(sample.accel_x_g)
        self.ay_hist.append(sample.accel_y_g)
        self.az_hist.append(sample.accel_z_g)

        self.screen.fill((18, 18, 28))
        self._draw_title("MPU-6050 IMU Trace", 25, 20)
        self._draw_accel_trace(sample)
        self._draw_text("Acceleration history", 540, 85)
        self._draw_time_graph(
            histories=[self.ax_hist, self.ay_hist, self.az_hist],
            labels=["ax", "ay", "az"],
            colors=[(255, 100, 100), (100, 255, 100), (100, 160, 255)],
            x=520,
            y=120,
            width=390,
            height=220,
            y_scale=1.5,
        )
        self._draw_text("Gyroscope after zero calibration", 540, 410)
        self._draw_bar("gx", sample.gyro_x_dps, 540, 465)
        self._draw_bar("gy", sample.gyro_y_dps, 540, 535)
        self._draw_bar("gz", sample.gyro_z_dps, 540, 605)
        self._draw_text(f"bias=({bias.x_dps:+.2f},{bias.y_dps:+.2f},{bias.z_dps:+.2f}) dps", 25, 675)
        pygame.display.flip()
        self.clock.tick(30)
        return True

    def close(self) -> None:
        self.pygame.quit()

    def _draw_text(self, text: str, x: int, y: int, color=(255, 255, 255)) -> None:
        self.screen.blit(self.font.render(text, True, color), (x, y))

    def _draw_title(self, text: str, x: int, y: int) -> None:
        self.screen.blit(self.big_font.render(text, True, (255, 255, 255)), (x, y))

    def _draw_bar(self, label: str, value: float, x: int, y: int, width: int = 280, height: int = 22, scale: int = 250) -> None:
        pygame = self.pygame
        pygame.draw.rect(self.screen, (90, 90, 90), (x, y, width, height), 1)
        mid = x + width // 2
        pygame.draw.line(self.screen, (130, 130, 130), (mid, y), (mid, y + height), 1)
        clamped = max(-scale, min(scale, value))
        pixels = int((clamped / scale) * (width // 2))
        rect = (mid, y, pixels, height) if pixels >= 0 else (mid + pixels, y, -pixels, height)
        pygame.draw.rect(self.screen, (0, 180, 255), rect)
        self._draw_text(f"{label}: {value: .2f} deg/s", x, y - 25)

    def _draw_time_graph(self, histories, labels, colors, x: int, y: int, width: int, height: int, y_scale: float = 1.5) -> None:
        pygame = self.pygame
        pygame.draw.rect(self.screen, (90, 90, 90), (x, y, width, height), 1)
        mid_y = y + height // 2
        pygame.draw.line(self.screen, (100, 100, 100), (x, mid_y), (x + width, mid_y), 1)
        self._draw_text("+1g", x + width + 8, y + int(height * 0.15))
        self._draw_text("0", x + width + 8, mid_y - 10)
        self._draw_text("-1g", x + width + 8, y + int(height * 0.80))

        for hist, _label, color in zip(histories, labels, colors):
            if len(hist) < 2:
                continue
            points = []
            for index, value in enumerate(hist):
                px = x + int(index * width / (self.max_history - 1))
                py = mid_y - int((value / y_scale) * (height / 2))
                points.append((px, max(y, min(y + height, py))))
            pygame.draw.lines(self.screen, color, False, points, 2)

        for index, (label, color) in enumerate(zip(labels, colors)):
            self._draw_text(label, x + index * 80, y + height + 12, color)

    def _draw_accel_trace(self, sample: ImuSample) -> None:
        pygame = self.pygame
        center_x = 250
        center_y = 280
        radius = 170
        pygame.draw.circle(self.screen, (90, 90, 90), (center_x, center_y), radius, 2)
        pygame.draw.line(self.screen, (80, 80, 80), (center_x - radius, center_y), (center_x + radius, center_y), 1)
        pygame.draw.line(self.screen, (80, 80, 80), (center_x, center_y - radius), (center_x, center_y + radius), 1)

        dot_x = center_x + int(sample.accel_x_g * radius)
        dot_y = center_y + int(sample.accel_y_g * radius)
        self.xy_trace.append((dot_x, dot_y))
        if len(self.xy_trace) > 2:
            for index in range(1, len(self.xy_trace)):
                brightness = int(60 + 195 * (index / len(self.xy_trace)))
                pygame.draw.line(self.screen, (brightness, 80, 80), self.xy_trace[index - 1], self.xy_trace[index], 2)
        pygame.draw.circle(self.screen, (255, 80, 80), (dot_x, dot_y), 10)

        self._draw_text("Accel XY trail", 170, 475)
        self._draw_text(f"ax = {sample.accel_x_g: .3f} g", 70, 515)
        self._draw_text(f"ay = {sample.accel_y_g: .3f} g", 70, 545)
        self._draw_text(f"az = {sample.accel_z_g: .3f} g", 70, 575)
        self._draw_text(f"|a| = {sample.accel_mag_g: .3f} g", 70, 605)


def main() -> None:
    ap = argparse.ArgumentParser(description="Read MPU-6050 IMU data over I2C.")
    ap.add_argument("--config", default="config/boat.local.json", help="Boat config JSON path")
    ap.add_argument("--bus", type=int, help="I2C bus number")
    ap.add_argument("--address", help="MPU-6050 I2C address, e.g. 0x68")
    ap.add_argument("--rate-hz", type=float, default=10.0, help="Terminal/log sample rate")
    ap.add_argument("--calibration-samples", type=int, default=200, help="Stationary gyro samples for zero calibration")
    ap.add_argument("--no-calibrate", action="store_true", help="Skip startup gyro calibration")
    ap.add_argument("--log", action="store_true", help="Write parsed IMU samples to logs/ as JSONL")
    ap.add_argument("--log-path", help="Custom JSONL log path")
    ap.add_argument("--viz", action="store_true", help="Show the pygame IMU visualization")
    args = ap.parse_args()
    apply_config(args)
    args.address = parse_address(args.address)

    log_file = None
    if args.log or args.log_path:
        path = Path(args.log_path) if args.log_path else default_log_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        log_file = path.open("a", encoding="utf-8")
        print(f"[IMU] Logging to {path}")

    delay_s = 1.0 / max(args.rate_hz, 0.1)
    viz = ImuViz() if args.viz else None

    print(f"[IMU] Opening I2C bus {args.bus}, address 0x{args.address:02x}. Ctrl+C to exit.")
    try:
        with Mpu6050(bus=args.bus, address=args.address) as imu:
            if args.no_calibrate:
                bias = GyroBias()
            else:
                print("[IMU] Keep MPU-6050 still. Calibrating gyro...")
                bias = imu.calibrate_gyro(samples=args.calibration_samples)
                print(f"[IMU] Gyro bias gx={bias.x_dps:.2f}, gy={bias.y_dps:.2f}, gz={bias.z_dps:.2f} dps")

            while True:
                sample = imu.read_sample(bias=bias)
                if log_file is not None:
                    log_file.write(json.dumps(sample.to_record(), separators=(",", ":")) + "\n")
                    log_file.flush()
                if viz is not None:
                    if not viz.update(sample, bias):
                        break
                else:
                    print_sample(sample)
                    time.sleep(delay_s)
    except KeyboardInterrupt:
        print("\n[IMU] Stopped.")
    finally:
        if viz is not None:
            viz.close()
        if log_file is not None:
            log_file.close()


if __name__ == "__main__":
    main()
