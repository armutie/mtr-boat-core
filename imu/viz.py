from __future__ import annotations

from collections import deque

from .mpu6050 import GyroBias, ImuSample


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

    def update(self, sample: ImuSample, bias: GyroBias, clock_hz: float = 30.0) -> bool:
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
        self.clock.tick(clock_hz)
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
