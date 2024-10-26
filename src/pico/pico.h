#ifndef ARDUINO_BOAT_H_
#define ARDUINO_BOAT_H_

#include <Arduino.h>
#include <FreeRTOS.h>
#include <RF24.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <data.h>
#include <nRF24L01.h>
#include <task.h>

#define SEND_TIMER 20
#define RECEIVE_TIMER 10
#define GPS_TIMER 100
#define sdfds MISO
// radio

#define CE_PIN 20
#define CSN_PIN 17

#define GPS_TX 3
#define GPS_RX 2

#define ENA 4  // left engine speed
#define ENB 5  // right engine speed
#define IN1 6  // left engine direction
#define IN2 7  // left engine direction
#define IN3 8  // right engine direction
#define IN4 9  // right engine direction

class PicoBoatController {
 private:
  RF24 radio;
  TinyGPSPlus gps;
  Data receivedData;
  GPSData gpsData;
  SoftwareSerial gpsSerial;

 private:
  void rotateMotor(uint8_t in, uint8_t in2, uint8_t pwm, int motor);
  void setMotors(int leftMotor, int rightMotor);
  void navigateToWaypoint(float lat, float lon);
  void manualControl();
  void updateGPSData();
  void stopMotors();

  static void receiveTask(void *param);
  static void sendTask(void *param);
  static void gpsTask(void *param);

 public:
  PicoBoatController();
};

#endif