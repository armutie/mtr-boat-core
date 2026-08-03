import unittest

from scripts.run_thruster_ramp import hold_pwm, pwm_pair


class FakeWriter:
    def __init__(self) -> None:
        self.pairs: list[tuple[int, int]] = []

    def send_pwm_pair(self, left_us: int, right_us: int) -> str:
        self.pairs.append((left_us, right_us))
        return f"OK L{left_us} R{right_us}"


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, duration_s: float) -> None:
        self.now += duration_s


class ThrusterRampTests(unittest.TestCase):
    def test_channel_selection_keeps_other_thruster_neutral(self) -> None:
        self.assertEqual(pwm_pair("left", 1425, 1500), (1425, 1500))
        self.assertEqual(pwm_pair("right", 1425, 1500), (1500, 1425))
        self.assertEqual(pwm_pair("both", 1425, 1500), (1425, 1425))

    def test_hold_repeats_commands_faster_than_watchdog(self) -> None:
        writer = FakeWriter()
        clock = FakeClock()

        hold_pwm(
            writer,
            "right",
            1425,
            1500,
            duration_s=1.0,
            send_hz=4.0,
            clock=clock,
            sleep=clock.sleep,
        )

        self.assertEqual(
            writer.pairs,
            [(1500, 1425)] * 4,
        )


if __name__ == "__main__":
    unittest.main()
