#include "motor.h"

MotorController::MotorController()
    : leftMotor{0, 0, true, MotorState::IDLE, 0, D0, D1},
      rightMotor{0, 0, true, MotorState::IDLE, 0, D2, D3} {}

void MotorController::updateMotorState(MotorControl &motor, int16_t speed) {
  motor.targetSpeed = speed;
  motor.forward = (speed >= 0);
  int absSpeed = abs(speed);

  if (absSpeed == 0) {
    if (motor.state != MotorState::IDLE) {
      stop(motor);
    }
    return;
  }

  bool needReset = (motor.state == MotorState::IDLE) ||
                   (motor.forward && speed < 0) ||
                   (!motor.forward && speed > 0);

  if (needReset) {
    initializeMotor(motor, absSpeed);
  } else {
    processMotorState(motor, absSpeed);
  }
}

void MotorController::initializeMotor(MotorControl &motor, int absSpeed) {
  unsigned long currentTime = millis();
  if (absSpeed < MotorConfig::MIN_THROTTLE) {
    motor.state = MotorState::START_PULSE;
    motor.stateStartTime = currentTime;
    motor.currentSpeed = MotorConfig::START_PULSE_POWER;
  } else {
    motor.state = MotorState::RAMP_UP;
    motor.stateStartTime = currentTime;
    motor.currentSpeed = MotorConfig::MIN_THROTTLE;
  }

  applySpeed(motor);
}

void MotorController::processMotorState(MotorControl &motor, int absSpeed) {
  switch (motor.state) {
    case MotorState::START_PULSE:
      processStartPulse(motor, absSpeed);
      break;

    case MotorState::RAMP_UP:
      processRampUp(motor, absSpeed);
      break;

    case MotorState::RUNNING:
      processRunning(motor, absSpeed);
      break;

    default:
      motor.state = MotorState::IDLE;
      break;
  }
}

void MotorController::processStartPulse(MotorControl &motor, int absSpeed) {
  unsigned long currentTime = millis();
  if (currentTime - motor.stateStartTime >= MotorConfig::START_PULSE_DURATION) {
    motor.state = MotorState::RAMP_UP;
    motor.stateStartTime = currentTime;
    motor.currentSpeed = MotorConfig::MIN_THROTTLE;
    applySpeed(motor);
  }
}

void MotorController::processRampUp(MotorControl &motor, int absSpeed) {
  unsigned long currentTime = millis();
  if (currentTime - motor.stateStartTime >= MotorConfig::RAMP_UP_TIME) {
    motor.state = MotorState::RUNNING;
    motor.currentSpeed = absSpeed;
  } else {
    float progress = static_cast<float>(currentTime - motor.stateStartTime) /
                     MotorConfig::RAMP_UP_TIME;
    motor.currentSpeed =
        MotorConfig::MIN_THROTTLE +
        static_cast<int>((absSpeed - MotorConfig::MIN_THROTTLE) * progress);
  }
  applySpeed(motor);
}

void MotorController::processRunning(MotorControl &motor, int absSpeed) {
  motor.currentSpeed = static_cast<int>(
      absSpeed * MotorConfig::SMOOTHING_FACTOR +
      motor.currentSpeed * (1.0f - MotorConfig::SMOOTHING_FACTOR));
  applySpeed(motor);
}

void MotorController::rotateMotorLeft(int16_t speed) {
  updateMotorState(leftMotor, speed);
}

void MotorController::rotateMotorRight(int16_t speed) {
  updateMotorState(rightMotor, speed);
}

void MotorController::stopMotors() {
  stop(leftMotor);
  stop(rightMotor);
}

void MotorController::stop(MotorControl &motor) {
  analogWrite(motor.forwardPin, LOW);
  analogWrite(motor.reversePin, LOW);
  motor.state = MotorState::IDLE;
  motor.currentSpeed = 0;
}

void MotorController::applySpeed(MotorControl &motor) {
  if (motor.forward) {
    analogWrite(motor.forwardPin, motor.currentSpeed);
    analogWrite(motor.reversePin, LOW);
  } else {
    analogWrite(motor.reversePin, motor.currentSpeed);
    analogWrite(motor.forwardPin, LOW);
  }
}

void MotorController::reset(MotorControl &motor) {
  motor.state = MotorState::IDLE;
  motor.currentSpeed = 0;
}

void MotorController::initMotors() {
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  stopMotors();
}