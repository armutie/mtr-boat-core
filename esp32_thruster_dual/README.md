# `esp32_thruster_dual` — two-channel ESP32 thruster firmware

Drives two BLDC ESCs from a single ESP32 over USB serial. Sibling of
`../esp32_thruster/esp32_thruster.ino` — the original single-channel
firmware is kept untouched so you always have a known-good build to fall
back on.

Same defaults, same arming behaviour, same hard PWM clamp. The only thing
this sketch adds is a second `Servo` instance, a small parser for the
`PWM L<us> R<us>` form the dashboard speaks, and a `READY` banner the host
can wait on.

## Wiring

| ESP32 pin    | Connects to                               |
|--------------|-------------------------------------------|
| GPIO 33      | LEFT  ESC signal wire (orange/yellow)     |
| GPIO 32      | RIGHT ESC signal wire (orange/yellow)     |
| GND          | Common ground with both ESCs              |
| USB 5V       | Logic power from host (laptop / Pi)       |

The ESCs **must** have their own battery + BEC. The Pi/laptop USB cannot drive
thrusters. The ESP32 GND must be tied to the ESC GND so the PWM signals share
a reference.

To change the pins, edit `ESC_LEFT_PIN` / `ESC_RIGHT_PIN` at the top of
`esp32_thruster_dual.ino`. ESP32 GPIOs 12, 13, 14, 25, 26, 27, 32, and 33 all
work well with `ESP32Servo`. Avoid the input-only pins (34–39) and the
strapping pins (0, 2, 5, 12, 15) for ESCs.

## Boot sequence

1. ESP32 boots, opens USB-CDC at 115200 baud.
2. Both channels are driven to `STALL_US` (1500 µs).
3. `THRUSTER_CALIBRATION_DELAY` ms (default 5000) of held-neutral so the ESCs
   complete their arming sequence.
4. `READY L=GPIO33 R=GPIO32 STALL=1500us ARM=5000ms WD=500ms` is printed once.
5. The serial loop starts accepting commands.

`Esp32ThrusterSerial` (`thruster_control/esp32_serial.py`) waits for that
`READY` line before returning from `__init__`, so the dashboard never sends
throttle to a not-yet-armed ESC. If a running board does not replay its
one-time banner when the port opens, the ROS node verifies pair support with
`PING` and requires a `PONG L... R...` response before accepting commands.

## Wire protocol

ASCII, line-terminated with `\n`. Whitespace tolerant. Commands are
case-insensitive.

| Request                | Response                  | Behaviour                                 |
|------------------------|---------------------------|-------------------------------------------|
| `PWM L<us> R<us>`      | `OK L<us> R<us>`          | Drive each channel independently.         |
| `PWM <us>`             | `OK PWM <us>`             | Legacy form. Drives **both** channels.    |
| `STOP`                 | `OK STOP`                 | Snap both channels to 1500 µs.            |
| `PING`                 | `PONG L<us> R<us>`        | Report the most recent values.            |
| _(any malformed line)_ | `ERR <detail>`            | Channels are unchanged.                   |

In addition the firmware emits unsolicited:

- `READY ...` once after arming completes.
- `STALE neutral` if no command is received for `COMMAND_TIMEOUT_MS`
  (default 500 ms). Both channels are forced to neutral; the host must send
  a fresh command to clear staleness on the firmware side.

PWM values are clamped to `[DUTY_CYCLE_MIN, DUTY_CYCLE_MAX]` (default
1350–2000 µs) **in firmware**, so an out-of-range or runaway value coming over
USB cannot push the ESCs past the configured envelope.

## Flashing (Arduino IDE)

1. Install the ESP32 board package: in Arduino IDE → File → Preferences,
   add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   to the board manager URLs, then install "esp32" from Tools → Board → Boards
   Manager.
2. Install the **ESP32Servo** library (Tools → Manage Libraries → search
   "ESP32Servo" by Kevin Harrington).
3. Tools → Board → "ESP32 Dev Module" (or whatever your specific board is).
4. Plug in the ESP32, pick the matching COM port, click Upload.

## Smoke test (no thrusters)

1. Open a serial monitor at **115200 baud** with line ending = "Newline".
2. After upload you should see one `READY ...` line.
3. Send `PING` — expect `PONG L1500 R1500`.
4. Send `PWM L1600 R1400` — expect `OK L1600 R1400`. If you have an
   oscilloscope or a logic analyzer, verify the corresponding pulse widths
   on GPIO33 and GPIO32.
5. Stop sending. After ~500 ms you should see `STALE neutral` printed
   automatically.
6. Send `STOP` — expect `OK STOP` and a return to neutral.

## Smoke test (motors only, props OFF)

1. Power the ESCs from a battery. Power the ESP32 from USB **first**, so the
   ESCs see neutral pulses immediately on their power-up.
2. After arming you should hear the ESC arming chirp / beep sequence.
3. Send `PWM L1530 R1530`. Both motors should spin slowly.
4. Send `PWM L1600 R1400`. Right motor speeds up, left slows. (If they're
   reversed from what you expect, you wired LEFT and RIGHT backwards — fix it
   in software by swapping `ESC_LEFT_PIN` and `ESC_RIGHT_PIN`, or in the
   harness.)
5. Send `STOP` and verify both go to a stop.

## Falling back to the single-channel firmware

If anything goes sideways with the dual-channel sketch, flash
`../esp32_thruster/esp32_thruster.ino` — it's the proven single-channel
firmware, untouched. The Python host (`Esp32ThrusterSerial`) is backward
compatible: it can still drive a single channel via the legacy `PWM <us>`
command path; only the dashboard's "differential drive" mix needs two
channels to feel right.
