#ifndef ARDUINO_BOAT_H_
#define ARDUINO_BOAT_H_

#include <Arduino.h>
#include <FreeRTOS.h>
#include <QMC5883LCompass.h>
#include <RF24.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <data.h>
#include <nRF24L01.h>
#include <semphr.h>
#include <task.h>

#include "motor.h"

#define MOTOR_TASK 40
#define SEND_TIMER 20
#define RECEIVE_TIMER 10
#define GPS_TIMER 20
#define STOP_DELAY 2000
#define COMPASS_TIMER 50

// radio
#define CE_PIN 20
#define CSN_PIN 17

#define GPS_TX 13
#define GPS_RX 12

#define THROTTLE_THRESHOLD 200

#define RUDDER_PIN 22

// gps
#define I2C_SDA 4
#define I2C_SCL 5

class PicoBoatController {
 private:
  RF24 radio;
  TinyGPSPlus gps;
  Data receivedData;
  GPSData gpsData;
  QMC5883LCompass compass;
  SemaphoreHandle_t xMutex;
  unsigned long lastRadioDataReceive;
  Servo rudderServo;
  MotorController motorController;

 private:
  void setMotors(int16_t leftMotor, int16_t rightMotor);
  void navigateToWaypoint(float lat, float lon);
  void updateCompassData();
  void manualControl();
  void updateGPSData();
  void stopMotors();
  void receiveData();
  void initControls();
  void actionOnStopReceive();
  void motorControl();
  void controlRudder(int throttle, int steering, int leftMotor, int rightMotor);

  static void receiveTask(void *param);
  static void sendTask(void *param);
  static void gpsTask(void *param);
  static void motorTask(void *param);
  static void compassTask(void *param);

 public:
  PicoBoatController();
  void begin();
};

#endif