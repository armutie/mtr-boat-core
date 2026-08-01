from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Protocol


class I2cBus(Protocol):
    def read_byte_data(self, address: int, register: int) -> int: ...

    def write_byte_data(
        self,
        address: int,
        register: int,
        value: int,
    ) -> None: ...

    def read_i2c_block_data(
        self,
        address: int,
        register: int,
        length: int,
    ) -> list[int]: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class Bno055Sample:
    orientation_w: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    angular_velocity_x_rad_s: float
    angular_velocity_y_rad_s: float
    angular_velocity_z_rad_s: float
    acceleration_x_mps2: float
    acceleration_y_mps2: float
    acceleration_z_mps2: float
    linear_acceleration_x_mps2: float
    linear_acceleration_y_mps2: float
    linear_acceleration_z_mps2: float
    magnetic_field_x_t: float
    magnetic_field_y_t: float
    magnetic_field_z_t: float
    gravity_x_mps2: float
    gravity_y_mps2: float
    gravity_z_mps2: float
    temperature_c: float


@dataclass(frozen=True)
class Bno055Status:
    system_calibration: int
    gyroscope_calibration: int
    accelerometer_calibration: int
    magnetometer_calibration: int
    system_status: int
    system_error: int

    @property
    def fully_calibrated(self) -> bool:
        return (
            self.system_calibration == 3
            and self.gyroscope_calibration == 3
            and self.accelerometer_calibration == 3
            and self.magnetometer_calibration == 3
        )


class Bno055Error(RuntimeError):
    pass


class Bno055IdentityError(Bno055Error):
    pass


class Bno055ReadError(Bno055Error):
    pass


class Bno055RecoveryPending(Bno055Error):
    pass


class Bno055Device(Protocol):
    def read_sample(self) -> Bno055Sample: ...

    def read_status(self) -> Bno055Status: ...

    def close(self) -> None: ...


class RecoveringBno055:
    """Reconnect and reconfigure a BNO055 after repeated read failures."""

    def __init__(
        self,
        imu: Bno055Device | None,
        factory: Callable[[], Bno055Device],
        *,
        failure_threshold: int = 5,
        initial_retry_delay_s: float = 1.0,
        max_retry_delay_s: float = 30.0,
        clock: Callable[[], float] = time.monotonic,
        initial_error: str = "",
    ) -> None:
        if failure_threshold < 1:
            raise ValueError("failure_threshold must be at least 1")
        if initial_retry_delay_s <= 0:
            raise ValueError("initial_retry_delay_s must be positive")
        if max_retry_delay_s < initial_retry_delay_s:
            raise ValueError(
                "max_retry_delay_s must be at least initial_retry_delay_s"
            )

        self._imu: Bno055Device | None = imu
        self._factory = factory
        self._failure_threshold = failure_threshold
        self._initial_retry_delay_s = initial_retry_delay_s
        self._max_retry_delay_s = max_retry_delay_s
        self._retry_delay_s = initial_retry_delay_s
        self._clock = clock
        self._next_retry_s = (
            clock() + initial_retry_delay_s
            if imu is None
            else 0.0
        )
        self._closed = False

        self.consecutive_failures = 0
        self.recovery_attempts = 0
        self.recovery_count = 0
        self.last_error = initial_error

    @property
    def failure_threshold(self) -> int:
        return self._failure_threshold

    @property
    def recovering(self) -> bool:
        return self._imu is None and not self._closed

    @property
    def retry_in_s(self) -> float:
        if not self.recovering:
            return 0.0
        return max(self._next_retry_s - self._clock(), 0.0)

    def read_sample(self) -> Bno055Sample:
        imu = self._require_imu()
        try:
            sample = imu.read_sample()
        except Exception as exc:
            self._record_read_failure(exc)
            raise

        self.consecutive_failures = 0
        self.last_error = ""
        return sample

    def read_status(self) -> Bno055Status:
        if self.recovering:
            raise Bno055RecoveryPending(self._pending_message())
        if self._imu is None:
            raise Bno055RecoveryPending("BNO055 driver is closed")
        return self._imu.read_status()

    def _require_imu(self) -> Bno055Device:
        if self._closed:
            raise Bno055RecoveryPending("BNO055 driver is closed")
        if self._imu is not None:
            return self._imu

        now = self._clock()
        if now < self._next_retry_s:
            raise Bno055RecoveryPending(self._pending_message())

        self.recovery_attempts += 1
        try:
            imu = self._factory()
        except Exception as exc:
            self.last_error = str(exc)
            retry_delay_s = self._retry_delay_s
            self._next_retry_s = now + retry_delay_s
            self._retry_delay_s = min(
                retry_delay_s * 2.0,
                self._max_retry_delay_s,
            )
            raise Bno055RecoveryPending(
                f"BNO055 reinitialization failed: {exc}; retrying in "
                f"{retry_delay_s:.1f}s"
            ) from exc

        self._imu = imu
        self.consecutive_failures = 0
        self.recovery_count += 1
        self.last_error = ""
        self._retry_delay_s = self._initial_retry_delay_s
        self._next_retry_s = 0.0
        return imu

    def _record_read_failure(self, exc: Exception) -> None:
        self.consecutive_failures += 1
        self.last_error = str(exc)
        if self.consecutive_failures < self._failure_threshold:
            return

        imu = self._imu
        self._imu = None
        self._next_retry_s = self._clock()
        if imu is not None:
            try:
                imu.close()
            except Exception:
                pass

    def _pending_message(self) -> str:
        message = "BNO055 automatic recovery pending"
        if self.last_error:
            message += f" after: {self.last_error}"
        retry_in_s = self.retry_in_s
        if retry_in_s > 0:
            message += f"; retrying in {retry_in_s:.1f}s"
        return message

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        imu = self._imu
        self._imu = None
        if imu is not None:
            imu.close()


class Bno055:
    """Read BNO055 I2C data in SI units using the Bosch Android format."""

    CHIP_ID = 0xA0

    REG_CHIP_ID = 0x00
    REG_PAGE_ID = 0x07
    REG_ACCEL_DATA = 0x08
    REG_LINEAR_ACCEL_DATA = 0x28
    REG_CALIBRATION_STATUS = 0x35
    REG_SYSTEM_STATUS = 0x39
    REG_SYSTEM_ERROR = 0x3A
    REG_UNIT_SELECTION = 0x3B
    REG_OPERATION_MODE = 0x3D
    REG_POWER_MODE = 0x3E
    REG_SYSTEM_TRIGGER = 0x3F
    REG_AXIS_MAP_CONFIG = 0x41
    REG_AXIS_MAP_SIGN = 0x42

    MODE_CONFIG = 0x00
    MODE_NDOF = 0x0C
    POWER_NORMAL = 0x00

    # m/s^2, rad/s, degrees, Celsius, Bosch Android orientation convention.
    # The Android orientation bit does not by itself guarantee a REP-103 ENU
    # quaternion for an arbitrary physical mounting.
    UNIT_SELECTION_SI_ANDROID = 0x82

    PLACEMENTS = {
        "P0": (0x21, 0x04),
        "P1": (0x24, 0x00),
        "P2": (0x24, 0x06),
        "P3": (0x21, 0x02),
        "P4": (0x24, 0x03),
        "P5": (0x21, 0x01),
        "P6": (0x21, 0x07),
        "P7": (0x24, 0x05),
    }

    def __init__(
        self,
        bus: int = 2,
        address: int = 0x29,
        placement: str = "P1",
        reset_on_start: bool = True,
        *,
        bus_factory: Callable[[int], I2cBus] | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        placement = placement.upper()
        if placement not in self.PLACEMENTS:
            choices = ", ".join(self.PLACEMENTS)
            raise ValueError(
                f"placement must be one of {choices}, got {placement!r}"
            )

        if bus_factory is None:
            import smbus2

            bus_factory = smbus2.SMBus

        self.address = address
        self.source = f"i2c-{bus}:0x{address:02x}"
        self._sleep = sleep
        self._bus = bus_factory(bus)
        self._closed = False

        try:
            self._verify_identity()
            if reset_on_start:
                self._reset()
            self._configure(placement)
        except Exception:
            self.close()
            raise

    def _verify_identity(self) -> None:
        chip_id = self._bus.read_byte_data(self.address, self.REG_CHIP_ID)
        if chip_id != self.CHIP_ID:
            raise Bno055IdentityError(
                f"expected BNO055 chip ID 0x{self.CHIP_ID:02x}, "
                f"received 0x{chip_id:02x} from {self.source}"
            )

    def _reset(self) -> None:
        self._write(self.REG_OPERATION_MODE, self.MODE_CONFIG)
        self._sleep(0.025)
        self._write(self.REG_SYSTEM_TRIGGER, 0x20)
        self._sleep(0.700)
        self._verify_identity()

    def _configure(self, placement: str) -> None:
        self._write(self.REG_OPERATION_MODE, self.MODE_CONFIG)
        self._sleep(0.025)
        self._write(self.REG_POWER_MODE, self.POWER_NORMAL)
        self._write(self.REG_PAGE_ID, 0x00)
        self._write(self.REG_SYSTEM_TRIGGER, 0x00)
        self._write(self.REG_UNIT_SELECTION, self.UNIT_SELECTION_SI_ANDROID)

        axis_map_config, axis_map_sign = self.PLACEMENTS[placement]
        self._write(self.REG_AXIS_MAP_CONFIG, axis_map_config)
        self._write(self.REG_AXIS_MAP_SIGN, axis_map_sign)

        self._write(self.REG_OPERATION_MODE, self.MODE_NDOF)
        self._sleep(0.500)

    def _write(self, register: int, value: int) -> None:
        self._bus.write_byte_data(self.address, register, value)

    def read_sample(self) -> Bno055Sample:
        sensor = self._read_block(self.REG_ACCEL_DATA, 32)
        fusion = self._read_block(self.REG_LINEAR_ACCEL_DATA, 13)

        accel = self._vector(sensor, 0, 100.0)
        magnetic = self._vector(sensor, 6, 16_000_000.0)
        gyroscope = self._vector(sensor, 12, 900.0)
        quaternion = [
            self._signed_16(sensor[index], sensor[index + 1]) / 16384.0
            for index in range(24, 32, 2)
        ]
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-6:
            raise Bno055ReadError(
                f"{self.source} returned an invalid zero quaternion"
            )
        quaternion = [value / norm for value in quaternion]

        linear_accel = self._vector(fusion, 0, 100.0)
        gravity = self._vector(fusion, 6, 100.0)
        temperature = float(self._signed_8(fusion[12]))

        return Bno055Sample(
            orientation_w=quaternion[0],
            orientation_x=quaternion[1],
            orientation_y=quaternion[2],
            orientation_z=quaternion[3],
            angular_velocity_x_rad_s=gyroscope[0],
            angular_velocity_y_rad_s=gyroscope[1],
            angular_velocity_z_rad_s=gyroscope[2],
            acceleration_x_mps2=accel[0],
            acceleration_y_mps2=accel[1],
            acceleration_z_mps2=accel[2],
            linear_acceleration_x_mps2=linear_accel[0],
            linear_acceleration_y_mps2=linear_accel[1],
            linear_acceleration_z_mps2=linear_accel[2],
            magnetic_field_x_t=magnetic[0],
            magnetic_field_y_t=magnetic[1],
            magnetic_field_z_t=magnetic[2],
            gravity_x_mps2=gravity[0],
            gravity_y_mps2=gravity[1],
            gravity_z_mps2=gravity[2],
            temperature_c=temperature,
        )

    def read_status(self) -> Bno055Status:
        calibration = self._bus.read_byte_data(
            self.address,
            self.REG_CALIBRATION_STATUS,
        )
        return Bno055Status(
            system_calibration=(calibration >> 6) & 0x03,
            gyroscope_calibration=(calibration >> 4) & 0x03,
            accelerometer_calibration=(calibration >> 2) & 0x03,
            magnetometer_calibration=calibration & 0x03,
            system_status=self._bus.read_byte_data(
                self.address,
                self.REG_SYSTEM_STATUS,
            ),
            system_error=self._bus.read_byte_data(
                self.address,
                self.REG_SYSTEM_ERROR,
            ),
        )

    def _read_block(self, register: int, length: int) -> list[int]:
        values = self._bus.read_i2c_block_data(self.address, register, length)
        if len(values) != length:
            raise Bno055ReadError(
                f"{self.source} returned {len(values)} bytes from register "
                f"0x{register:02x}; expected {length}"
            )
        return values

    @classmethod
    def _vector(
        cls,
        values: list[int],
        offset: int,
        scale: float,
    ) -> tuple[float, float, float]:
        return tuple(
            cls._signed_16(values[index], values[index + 1]) / scale
            for index in range(offset, offset + 6, 2)
        )

    @staticmethod
    def _signed_16(low: int, high: int) -> int:
        value = low | (high << 8)
        return value - 0x10000 if value & 0x8000 else value

    @staticmethod
    def _signed_8(value: int) -> int:
        return value - 0x100 if value & 0x80 else value

    def close(self) -> None:
        if not self._closed:
            self._bus.close()
            self._closed = True

    def __enter__(self) -> Bno055:
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()
