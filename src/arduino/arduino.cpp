#include "arduino.h"

ArduinoBoatController::ArduinoBoatController()
    : radio(CE_PIN, CSN_PIN), gpsSerial(A0, A1),
      receivedData{0, 0, 0.0f, 0.0f, false, false, 0.0f, 0.0f},
      gpsData{0.0f, 0.0f, 0.0f} {
#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif
  gpsSerial.begin(9600);

  while (!radio.begin()) {
#ifdef DEBUG_MODE
    Serial.println("Radio doesn t work");
#endif
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
  sendTimer.start(50);
  receiveTimer.start(20);
}

void ArduinoBoatController::loop() {
  bool isSend = sendTimer.ready();
  bool isReceive = receiveTimer.ready();
  while ((isSend || isReceive) && gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      updateGPSData();
    }
  }
  if (isSend) {
    radio.stopListening();
    radio.write(&gpsData, sizeof(GPSData));
    radio.startListening();
  }
  if (isReceive) {
    if (radio.available()) {
      radio.read(&receivedData, sizeof(Data));
      if (receivedData.autopilotEnabled && gps.location.isValid()) {
        navigateToWaypoint(receivedData.autopilotLat,
                           receivedData.autopilotLon);
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
  }
}

void ArduinoBoatController::updateGPSData() {
  if (gps.location.isValid()) {
    gpsData.latitude = gps.location.lat();
    gpsData.longitude = gps.location.lng();
    if (gps.course.isValid()) {
      gpsData.course = gps.course.deg();
    }
  }
}

void ArduinoBoatController::navigateToWaypoint(float lat, float lon) {
  float distanceToWaypoint = TinyGPSPlus::distanceBetween(
      gpsData.latitude, gpsData.longitude, lat, lon);

  float courseToWaypoint =
      TinyGPSPlus::courseTo(gpsData.latitude, gpsData.longitude, lat, lon);

  float courseError = courseToWaypoint - gpsData.course;
  if (courseError > 180)
    courseError -= 360;
  if (courseError < -180)
    courseError += 360;

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

#ifdef DEBUG_MODE
  Serial.print("Distance to waypoint: ");
  Serial.print(distanceToWaypoint);
  Serial.print(" m, Course error: ");
  Serial.println(courseError);
#endif
}

void ArduinoBoatController::manualControl() {
  int throttle = receivedData.y;
  int steering = receivedData.x;
  int leftMotor = throttle + steering;
  int rightMotor = throttle - steering;

  setMotors(leftMotor, rightMotor);
}

void ArduinoBoatController::setMotors(int leftMotor, int rightMotor) {
  leftMotor = constrain(leftMotor, -255, 255);
  rightMotor = constrain(rightMotor, -255, 255);

  rotateMotor(IN1, IN2, ENA, leftMotor);
  rotateMotor(IN3, IN4, ENB, rightMotor);

#ifdef DEBUG_MODE
  Serial.print("Left Motor: ");
  Serial.print(leftMotor);
  Serial.print(" Right Motor: ");
  Serial.println(rightMotor);
#endif
}

void ArduinoBoatController::rotateMotor(uint8_t in, uint8_t in2, uint8_t pwm,
                                        int motor) {
  if (motor > 0) {
    digitalWrite(in, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, motor);
  } else if (motor < 0) {
    digitalWrite(in, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, abs(motor));
  } else {
    analogWrite(pwm, 0);
  }
}

void ArduinoBoatController::stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
