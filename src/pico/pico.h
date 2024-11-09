#ifndef ARDUINO_BOAT_H_
#define ARDUINO_BOAT_H_

#include <Arduino.h>
#include <FreeRTOS.h>
#include <RF24.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <data.h>
#include <nRF24L01.h>
#include <semphr.h>
#include <task.h>

#define MOTOR_TASK 40
#define SEND_TIMER 20
#define RECEIVE_TIMER 10
#define GPS_TIMER 20
#define STOP_DELAY 2000

// radio
#define CE_PIN 20
#define CSN_PIN 17

#define GPS_TX 13
#define GPS_RX 12

#define ENA 4   // left engine speed
#define ENB 5   // right engine speed
#define IN1 10  // left engine direction
#define IN2 7   // left engine direction
#define IN3 8   // right engine direction
#define IN4 9   // right engine direction
#define START_IMPULSE 120
#define THROTTLE_THRESHOLD 200

#define RUDDER_PIN 2

class PicoBoatController {
 private:
  enum MOTOR { RIGHT, LEFT };

  RF24 radio;
  TinyGPSPlus gps;
  Data receivedData;
  GPSData gpsData;
  SemaphoreHandle_t xMutex;
  unsigned long lastRadioDataReceive;
  int16_t prevLpwm;
  int16_t prevRpwm;
  Servo rudderServo;

 private:
  void rotateMotor(MOTOR motor, int16_t speed);
  void setMotors(int16_t leftMotor, int16_t rightMotor);
  void navigateToWaypoint(float lat, float lon);
  void manualControl();
  void updateGPSData();
  void stopMotors();
  void receiveData();
  void initMotors();
  void actionOnStopReceive();
  void motorControl();

  static void receiveTask(void *param);
  static void sendTask(void *param);
  static void gpsTask(void *param);
  static void motorTask(void *param);

 public:
  PicoBoatController();
  void begin();
};

#endif