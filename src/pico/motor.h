#ifndef MOTOR_H_
#define MOTOR_H_

#include <Arduino.h>

// first
#define D0 4
#define D1 5
// second
#define D2 6
#define D3 7

namespace MotorConfig {
constexpr uint16_t START_PULSE_DURATION = 100;
constexpr uint8_t START_PULSE_POWER = 150;
constexpr uint8_t MIN_THROTTLE = 40;
constexpr uint16_t RAMP_UP_TIME = 500;
constexpr float SMOOTHING_FACTOR = 0.2f;
}  // namespace MotorConfig

enum class MotorState { IDLE, START_PULSE, RAMP_UP, RUNNING };

struct MotorControl {
  int16_t targetSpeed;
  int16_t currentSpeed;
  bool forward;
  MotorState state;
  unsigned long stateStartTime;
  uint8_t forwardPin;
  uint8_t reversePin;
};

class MotorController {
 private:
  MotorControl leftMotor;
  MotorControl rightMotor;

 public:
  MotorController();
  void rotateMotorLeft(int16_t speed);
  void rotateMotorRight(int16_t speed);
  void stopMotors();
  void initMotors();

 private:
  void stop(MotorControl& motor);
  void applySpeed(MotorControl& motor);
  void reset(MotorControl& motor);
  void updateMotorState(MotorControl& motor, int16_t speed);
  void initializeMotor(MotorControl& motor, int absSpeed);
  void processMotorState(MotorControl& motor, int absSpeed);
  void processRunning(MotorControl& motor, int absSpeed);
  void processRampUp(MotorControl& motor, int absSpeed);
  void processStartPulse(MotorControl& motor, int absSpeed);
};

#endif