#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define CE_PIN 9
#define CSN_PIN 10

#define ENA 3  // left engine speed
#define ENB 5  // right engine speed
#define IN1 4  // left engine direction
#define IN2 6  // left engine direction
#define IN3 7  // right engine direction
#define IN4 8  // right engine direction

RF24 radio(CE_PIN, CSN_PIN);
TinyGPSPlus gps;
SoftwareSerial gpsSerial(A0, A1);

const byte address[6] = "00001";

struct Data {
  int16_t y;
  int16_t x;
  float autopilotLat;
  float autopilotLon;
  bool autopilotEnabled;
  bool isHome;
  float homeLat;
  float homeLon;
};

Data receivedData;

struct GPSData {
  float latitude;
  float longitude;
  float course;
};

GPSData gpsData;

void setup() {
  // Serial.begin(115200);
  gpsSerial.begin(9600);

  while (!radio.begin()) {
    // Serial.println("Radio doesn t work");
    delay(1000);
  }
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      updateGPSData();
    }
  }
  if (radio.available()) {
    radio.read(&receivedData, sizeof(Data));
    if (receivedData.autopilotEnabled && gps.location.isValid()) {
      navigateToWaypoint(receivedData.autopilotLat, receivedData.autopilotLon);
    } else {
      manualControl();
    }
  } else {
    if (receivedData.isHome) {
      navigateToWaypoint(receivedData.homeLat, receivedData.homeLon);
    } else {
      stopMotors();
    }
  }

  delay(50);
}

void updateGPSData() {
  if (gps.location.isValid()) {
    gpsData.latitude = gps.location.lat();
    gpsData.longitude = gps.location.lng();
    if (gps.course.isValid()) {
      gpsData.course = gps.course.deg();
    }
    radio.stopListening();
    radio.write(&gpsData, sizeof(GPSData));
    radio.startListening();
  }
}

void navigateToWaypoint(float lat, float lon) {
  float distanceToWaypoint = TinyGPSPlus::distanceBetween(
    gpsData.latitude, gpsData.longitude,
    lat, lon);

  float courseToWaypoint = TinyGPSPlus::courseTo(
    gpsData.latitude, gpsData.longitude,
    lat, lon);

  float courseError = courseToWaypoint - gpsData.course;
  if (courseError > 180) courseError -= 360;
  if (courseError < -180) courseError += 360;

  int throttle = 0;
  int steering = 0;

  if (distanceToWaypoint > 5) {
    throttle = 200;
    steering = constrain(courseError * 2, -100, 100);
  } else if (distanceToWaypoint > 1) {
    throttle = 100;
    steering = constrain(courseError * 1.5, -50, 50);
  } else {
    stopMotors();
    return;
  }

  int leftMotor = throttle + steering;
  int rightMotor = throttle - steering;

  setMotors(leftMotor, rightMotor);

  // Serial.print("Distance to waypoint: ");
  // Serial.print(distanceToWaypoint);
  // Serial.print(" m, Course error: ");
  // Serial.println(courseError);
}

void manualControl() {
  int throttle = receivedData.y;
  int steering = receivedData.x;
  int leftMotor = throttle + steering;
  int rightMotor = throttle - steering;

  setMotors(leftMotor, rightMotor);
}

void setMotors(int leftMotor, int rightMotor) {
  leftMotor = constrain(leftMotor, -255, 255);
  rightMotor = constrain(rightMotor, -255, 255);

  if (leftMotor > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftMotor);
  } else if (leftMotor < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(leftMotor));
  } else {
    analogWrite(ENA, 0);
  }

  if (rightMotor > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightMotor);
  } else if (rightMotor < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(rightMotor));
  } else {
    analogWrite(ENB, 0);
  }

  // Serial.print("Left Motor: ");
  // Serial.print(leftMotor);
  // Serial.print(" Right Motor: ");
  // Serial.println(rightMotor);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
