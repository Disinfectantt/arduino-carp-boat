#include "pico.h"

PicoBoatController::PicoBoatController()
    : radio(CE_PIN, CSN_PIN),
      receivedData{0, 0, 0.0f, 0.0f, false, false, 0.0f, 0.0f},
      gpsData{0.0f, 0.0f, 0.0f} {
#ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Pico spi started");
#endif
  // Serial2.setRX(GPS_RX);
  // Serial2.setTX(GPS_TX);
  // Serial2.begin(9600);

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
  xTaskCreate(receiveTask, "receive", 2048, this, 3, NULL);
  xTaskCreate(sendTask, "send", 2048, this, 3, NULL);
  xTaskCreate(gpsTask, "gps", 2048, this, 3, NULL);
}

void PicoBoatController::receiveTask(void *param) {
  PicoBoatController *p = (PicoBoatController *)param;
  while (1) {
    if (p->radio.available()) {
      p->radio.read(&p->receivedData, sizeof(Data));
      if (p->receivedData.autopilotEnabled && p->gps.location.isValid()) {
        p->navigateToWaypoint(p->receivedData.autopilotLat,
                              p->receivedData.autopilotLon);
      } else {
        p->manualControl();
      }
    } else {
      if (p->receivedData.isHome) {
        p->navigateToWaypoint(p->receivedData.homeLat, p->receivedData.homeLon);
      } else {
        p->stopMotors();
      }
    }
    vTaskDelay(RECEIVE_TIMER / portTICK_PERIOD_MS);
  }
}

void PicoBoatController::sendTask(void *param) {
  PicoBoatController *p = (PicoBoatController *)param;
  while (1) {
    p->radio.stopListening();
    p->radio.write(&p->gpsData, sizeof(GPSData));
    p->radio.startListening();
    vTaskDelay(SEND_TIMER / portTICK_PERIOD_MS);
  }
}

void PicoBoatController::gpsTask(void *param) {
  PicoBoatController *p = (PicoBoatController *)param;
  while (1) {
    while (Serial2.available() > 0) {
      if (p->gps.encode(Serial2.read())) {
        p->updateGPSData();
      }
    }
    vTaskDelay(GPS_TIMER / portTICK_PERIOD_MS);
  }
}

void PicoBoatController::updateGPSData() {
  if (gps.location.isValid()) {
    gpsData.latitude = gps.location.lat();
    gpsData.longitude = gps.location.lng();
    if (gps.course.isValid()) {
      gpsData.course = gps.course.deg();
    }
    // #ifdef DEBUG_MODE
    //   Serial.printf("lat: %f\nlon: %f\ncourse: %f", gpsData.latitude,
    //   gpsData.longitude, gpsData.course);
    // #endif
  }
}

void PicoBoatController::navigateToWaypoint(float lat, float lon) {
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

  // #ifdef DEBUG_MODE
  //   Serial.print("Distance to waypoint: ");
  //   Serial.print(distanceToWaypoint);
  //   Serial.print(" m, Course error: ");
  //   Serial.println(courseError);
  // #endif
}

void PicoBoatController::manualControl() {
  int throttle = receivedData.y;
  int steering = receivedData.x;
  int leftMotor = throttle + steering;
  int rightMotor = throttle - steering;

  setMotors(leftMotor, rightMotor);
}

void PicoBoatController::setMotors(int leftMotor, int rightMotor) {
  leftMotor = constrain(leftMotor, -255, 255);
  rightMotor = constrain(rightMotor, -255, 255);

  rotateMotor(IN1, IN2, ENA, leftMotor);
  rotateMotor(IN3, IN4, ENB, rightMotor);

  // #ifdef DEBUG_MODE
  //   Serial.print("Left Motor: ");
  //   Serial.print(leftMotor);
  //   Serial.print(" Right Motor: ");
  //   Serial.println(rightMotor);
  // #endif
}

void PicoBoatController::rotateMotor(uint8_t in, uint8_t in2, uint8_t pwm,
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

void PicoBoatController::stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
