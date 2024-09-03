#ifndef ARDUINO_BOAT_H_
#define ARDUINO_BOAT_H_

#include <RF24.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Timer.h>
#include <TinyGPS++.h>
#include <data.h>
#include <nRF24L01.h>

#define CE_PIN 9
#define CSN_PIN 10

#define ENA 3 // left engine speed
#define ENB 5 // right engine speed
#define IN1 4 // left engine direction
#define IN2 6 // left engine direction
#define IN3 7 // right engine direction
#define IN4 8 // right engine direction

class ArduinoBoatController {
private:
  RF24 radio;
  TinyGPSPlus gps;
  SoftwareSerial gpsSerial;
  Data receivedData;
  GPSData gpsData;
  Timer sendTimer, receiveTimer;

private:
  void rotateMotor(uint8_t in, uint8_t in2, uint8_t pwm, int motor);
  void setMotors(int leftMotor, int rightMotor);
  void navigateToWaypoint(float lat, float lon);
  void manualControl();
  void updateGPSData();
  void stopMotors();

public:
  ArduinoBoatController();
  void loop();
};

#endif