// Two-channel thruster bridge for ESP32.
//
// Sibling of the proven single-channel `esp32_thruster.ino`. Same arming
// behaviour, same hard PWM clamp, same watchdog, but drives two ESCs and
// understands the `PWM L<us> R<us>` form that the dashboard speaks.
//
// Wire protocol over USB serial (115200 baud, ASCII, '\n'-terminated):
//
//   PWM L<us> R<us>   -- drive left and right ESCs to the given microsecond
//                        pulse widths. Each value is independently clamped
//                        into [DUTY_CYCLE_MIN, DUTY_CYCLE_MAX].
//   PWM <us>          -- backward-compatible single-channel form. Drives
//                        BOTH ESCs to the same value (lets you keep using
//                        the legacy host code on this firmware).
//   STOP              -- snap both channels to STALL_US immediately.
//   PING              -- emits PONG L<us> R<us> with the most recent values.
//
// Responses (one per command, line-terminated):
//
//   OK L<us> R<us>   for an accepted PWM pair
//   OK PWM <us>      for an accepted single-channel PWM
//   OK STOP          for an accepted STOP
//   PONG L<us> R<us> for a PING
//   ERR <detail>     for malformed or out-of-range commands
//   READY ...        emitted once after ESC arming completes
//   STALE            emitted once when the command watchdog fires
//
// Safety behaviour:
//
// 1. ESC arming. setup() drives neutral for THRUSTER_CALIBRATION_DELAY ms
//    (default 5 s, mirrors the proven single-channel firmware) before the
//    host-facing serial loop starts. Hosts should wait for the READY line
//    before sending throttle.
//
// 2. Hard PWM clamp. Every accepted command is clamped into
//    [DUTY_CYCLE_MIN, DUTY_CYCLE_MAX] in firmware, so a runaway value
//    coming over USB cannot push the ESCs past the configured envelope.
//
// 3. Command watchdog. If no command arrives for COMMAND_TIMEOUT_MS the
//    firmware snaps both channels to neutral and reports STALE.
//
// To wire two thrusters:
//   - LEFT  ESC signal -> ESP32 GPIO ESC_LEFT_PIN  (default 33, same as the
//                                                  single-channel firmware)
//   - RIGHT ESC signal -> ESP32 GPIO ESC_RIGHT_PIN (default 32)
//   - Common ground between ESCs and the ESP32. ESCs powered separately
//     from a battery via their own BEC; the ESP32 logic 5V from USB.

#include <ESP32Servo.h>

#define ESC_LEFT_PIN  33
#define ESC_RIGHT_PIN 32

#define THRUSTER_CALIBRATION_DELAY 5000  // ms held at neutral before accepting commands
#define STALL_US                   1500
#define DUTY_CYCLE_MIN             1350  // hard floor
#define DUTY_CYCLE_MAX             2000  // hard ceiling
#define SERIAL_BAUD                115200
#define COMMAND_TIMEOUT_MS         500   // watchdog window before snapping to neutral

Servo thruster_left;
Servo thruster_right;

unsigned long last_command_ms = 0;
int last_left_us  = STALL_US;
int last_right_us = STALL_US;
bool went_stale  = false;

int clampPwm(int value) {
  if (value < DUTY_CYCLE_MIN) return DUTY_CYCLE_MIN;
  if (value > DUTY_CYCLE_MAX) return DUTY_CYCLE_MAX;
  return value;
}

void writePair(int left_us, int right_us) {
  last_left_us  = clampPwm(left_us);
  last_right_us = clampPwm(right_us);
  thruster_left.writeMicroseconds(last_left_us);
  thruster_right.writeMicroseconds(last_right_us);
}

void writeNeutral() {
  writePair(STALL_US, STALL_US);
}

// Pull the integer that follows `tag` (e.g. 'L' or 'R') in `args`.
// Returns true on success and writes into `out`. Accepts optional whitespace
// between the tag and the number.
bool extractTagged(const String &args, char tag, int &out) {
  int idx = args.indexOf(tag);
  if (idx < 0) return false;
  int cursor = idx + 1;
  while (cursor < args.length() && (args[cursor] == ' ' || args[cursor] == '\t')) {
    cursor += 1;
  }
  int end = cursor;
  if (end < args.length() && (args[end] == '+' || args[end] == '-')) end += 1;
  while (end < args.length() && args[end] >= '0' && args[end] <= '9') {
    end += 1;
  }
  if (end == cursor) return false;
  out = args.substring(cursor, end).toInt();
  return true;
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();

  if (upper == "STOP") {
    writeNeutral();
    last_command_ms = millis();
    went_stale = false;
    Serial.println("OK STOP");
    return;
  }

  if (upper == "PING") {
    Serial.print("PONG L");
    Serial.print(last_left_us);
    Serial.print(" R");
    Serial.println(last_right_us);
    return;
  }

  if (upper.startsWith("PWM ")) {
    String args = upper.substring(4);
    args.trim();

    // Two-channel form: PWM L<us> R<us>
    if (args.indexOf('L') >= 0 && args.indexOf('R') >= 0) {
      int left_us  = -1;
      int right_us = -1;
      if (!extractTagged(args, 'L', left_us) || !extractTagged(args, 'R', right_us)) {
        Serial.println("ERR bad pair");
        return;
      }
      if (left_us <= 0 || right_us <= 0) {
        Serial.println("ERR bad pair value");
        return;
      }
      writePair(left_us, right_us);
      last_command_ms = millis();
      went_stale = false;
      Serial.print("OK L");
      Serial.print(last_left_us);
      Serial.print(" R");
      Serial.println(last_right_us);
      return;
    }

    // Single-channel form: PWM <us> -> drive both for legacy hosts.
    int pwm_us = args.toInt();
    if (pwm_us <= 0) {
      Serial.println("ERR bad single");
      return;
    }
    writePair(pwm_us, pwm_us);
    last_command_ms = millis();
    went_stale = false;
    Serial.print("OK PWM ");
    Serial.println(last_left_us);
    return;
  }

  Serial.print("ERR unknown ");
  Serial.println(line);
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  thruster_left.setPeriodHertz(50);
  thruster_right.setPeriodHertz(50);
  thruster_left.attach(ESC_LEFT_PIN,   1000, 2000);
  thruster_right.attach(ESC_RIGHT_PIN, 1000, 2000);

  // Hold both at neutral so the ESCs see a steady arming signal.
  writeNeutral();
  delay(THRUSTER_CALIBRATION_DELAY);

  last_command_ms = millis();
  went_stale = false;

  Serial.print("READY L=GPIO");
  Serial.print(ESC_LEFT_PIN);
  Serial.print(" R=GPIO");
  Serial.print(ESC_RIGHT_PIN);
  Serial.print(" STALL=");
  Serial.print(STALL_US);
  Serial.print("us ARM=");
  Serial.print(THRUSTER_CALIBRATION_DELAY);
  Serial.print("ms WD=");
  Serial.print(COMMAND_TIMEOUT_MS);
  Serial.println("ms");
}

void loop() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  if (!went_stale && (millis() - last_command_ms) > COMMAND_TIMEOUT_MS) {
    writeNeutral();
    went_stale = true;
    Serial.println("STALE neutral");
  }
}
