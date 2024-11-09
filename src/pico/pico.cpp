#include "pico.h"

PicoBoatController::PicoBoatController()
    : radio(CE_PIN, CSN_PIN),
      receivedData{0, 0, 0.0f, 0.0f, false, false, 0.0f, 0.0f},
      gpsData{0.0f, 0.0f, 0.0f},
      xMutex(NULL),
      lastRadioDataReceive(0),
      prevLpwm(0),
      prevRpwm(0),
      rudderServo() {}

void PicoBoatController::begin() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial);
#endif
  Serial1.setRX(GPS_TX);
  Serial1.setTX(GPS_RX);
  Serial1.begin(9600);

  while (!radio.begin()) {
#ifdef DEBUG_MODE
    Serial.println("Radio doesn t work");
#endif
    delay(1000);
  }
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(0, address[0]);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);
  radio.startListening();

  initMotors();

  xMutex = xSemaphoreCreateMutex();
  xTaskCreate(motorTask, "motor", 2048, this, 3, NULL);
  xTaskCreate(receiveTask, "receive", 2048, this, 3, NULL);
  xTaskCreate(sendTask, "send", 2048, this, 2, NULL);
  xTaskCreate(gpsTask, "gps", 2048, this, 3, NULL);
}

void PicoBoatController::receiveTask(void *param) {
  PicoBoatController *p = (PicoBoatController *)param;
  while (1) {
    if (xSemaphoreTake(p->xMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      p->receiveData();
      xSemaphoreGive(p->xMutex);
    } else {
      p->actionOnStopReceive();
    }
    vTaskDelay(pdMS_TO_TICKS(RECEIVE_TIMER));
  }
}

void PicoBoatController::sendTask(void *param) {
  PicoBoatController *p = (PicoBoatController *)param;
  while (1) {
    if (xSemaphoreTake(p->xMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      p->radio.stopListening();
      p->radio.write(&p->gpsData, sizeof(GPSData));
      p->radio.startListening();
      xSemaphoreGive(p->xMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(SEND_TIMER));
  }
}

void PicoBoatController::gpsTask(void *param) {
  PicoBoatController *p = (PicoBoatController *)param;
  while (1) {
    p->updateGPSData();
    vTaskDelay(pdMS_TO_TICKS(GPS_TIMER));
  }
}

void PicoBoatController::motorTask(void *param) {
  PicoBoatController *p = (PicoBoatController *)param;
  while (1) {
    p->motorControl();
    vTaskDelay(pdMS_TO_TICKS(MOTOR_TASK));
  }
}

void PicoBoatController::motorControl() {
  if (lastRadioDataReceive && millis() - lastRadioDataReceive >= STOP_DELAY) {
    actionOnStopReceive();
  } else {
    if (receivedData.autopilotEnabled && gps.location.isValid()) {
      navigateToWaypoint(receivedData.autopilotLat, receivedData.autopilotLon);
    } else {
      manualControl();
    }
  }
}

void PicoBoatController::receiveData() {
  if (radio.available()) {
    radio.read(&receivedData, sizeof(Data));
    lastRadioDataReceive = millis();
  }
}

void PicoBoatController::updateGPSData() {
  while (Serial1.available()) {
    // Serial.print((char)Serial1.read());
    gps.encode(Serial1.read());
  }
  if (gps.location.isValid()) {
    gpsData.latitude = gps.location.lat();
    gpsData.longitude = gps.location.lng();
    if (gps.course.isValid()) {
      gpsData.course = gps.course.deg();
    }
  }
  // #ifdef DEBUG_MODE
  //   Serial.printf("lat: %f\nlon: %f\ncourse: %f\n", gpsData.latitude,
  //   gpsData.longitude, gpsData.course);
  //   Serial.printf("Satellites:%d\n", gps.satellites.value());
  // Serial.printf("Failed checksum: %d\n", gps.failedChecksum());
  // #endif
}

void PicoBoatController::navigateToWaypoint(float lat, float lon) {
  float distanceToWaypoint = TinyGPSPlus::distanceBetween(
      gpsData.latitude, gpsData.longitude, lat, lon);

  float courseToWaypoint =
      TinyGPSPlus::courseTo(gpsData.latitude, gpsData.longitude, lat, lon);

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

void PicoBoatController::setMotors(int16_t leftMotor, int16_t rightMotor) {
  leftMotor = constrain(leftMotor, -255, 255);
  rightMotor = constrain(rightMotor, -255, 255);

  // if (prevLpwm == 0 && leftMotor != 0) {
  //   rotateMotor(LEFT, leftMotor > 0 ? START_IMPULSE : -START_IMPULSE);
  // }

  // if (prevRpwm == 0 && rightMotor != 0) {
  //   rotateMotor(RIGHT, rightMotor > 0 ? START_IMPULSE : -START_IMPULSE);
  // }

  if (abs(receivedData.y) > THROTTLE_THRESHOLD &&
      (leftMotor * rightMotor > 0)) {
    int rudderAngle = map(receivedData.x, -255, 255, 45, 135);
    rudderServo.write(rudderAngle);
  } else {
    rudderServo.write(90);
  }

  rotateMotor(LEFT, leftMotor);
  rotateMotor(RIGHT, rightMotor);
  prevLpwm = leftMotor;
  prevRpwm = rightMotor;

  // #ifdef DEBUG_MODE
  //   Serial.print("Left Motor: ");
  //   Serial.print(leftMotor);
  //   Serial.print(" Right Motor: ");
  //   Serial.println(rightMotor);
  // #endif
}

void PicoBoatController::rotateMotor(MOTOR motor, int16_t speed) {
  if (motor == LEFT) {
    if (speed > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, speed);
    } else if (speed < 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, abs(speed));
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
    }
  }

  if (motor == RIGHT) {
    if (speed > 0) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, speed);
    } else if (speed < 0) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, abs(speed));
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 0);
    }
  }
}

void PicoBoatController::stopMotors() {
  receivedData.x = 0;
  receivedData.y = 0;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, receivedData.x);
  analogWrite(ENB, receivedData.y);
  prevRpwm = 0;
  prevLpwm = 0;
  rudderServo.write(90);
}

void PicoBoatController::initMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  rudderServo.attach(RUDDER_PIN);
  stopMotors();
}

void PicoBoatController::actionOnStopReceive() {
  if (receivedData.isHome) {
    navigateToWaypoint(receivedData.homeLat, receivedData.homeLon);
  } else {
    stopMotors();
  }
}
