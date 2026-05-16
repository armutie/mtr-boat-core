#include <ESP32Servo.h>

#define ESC_SIGNAL_PIN 33

#define THRUSTER_CALIBRATION_DELAY 5000
#define STALL_US 1500
#define DUTY_CYCLE_MIN 1350
#define DUTY_CYCLE_MAX 2000
#define SERIAL_BAUD 115200
#define COMMAND_TIMEOUT_MS 1000

Servo thruster;
unsigned long last_command_ms = 0;
int current_pwm_us = STALL_US;

int clampPwm(int value) {
  if (value < DUTY_CYCLE_MIN) {
    return DUTY_CYCLE_MIN;
  }
  if (value > DUTY_CYCLE_MAX) {
    return DUTY_CYCLE_MAX;
  }
  return value;
}

void writeThruster(int pwm_us) {
  current_pwm_us = clampPwm(pwm_us);
  thruster.writeMicroseconds(current_pwm_us);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  thruster.attach(ESC_SIGNAL_PIN, 1000, 2000);
  writeThruster(STALL_US);
  delay(THRUSTER_CALIBRATION_DELAY);
  last_command_ms = millis();
  Serial.println("READY");
}

void handleCommand(String line) {
  line.trim();
  line.toUpperCase();

  if (line == "STOP") {
    writeThruster(STALL_US);
    last_command_ms = millis();
    Serial.println("OK STOP");
    return;
  }

  if (line.startsWith("PWM ")) {
    int pwm_us = line.substring(4).toInt();
    writeThruster(pwm_us);
    last_command_ms = millis();
    Serial.print("OK PWM ");
    Serial.println(current_pwm_us);
    return;
  }

  Serial.print("ERR ");
  Serial.println(line);
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    writeThruster(STALL_US);
  }
}
