#include "./pico/pico.h"

#ifdef CALIBRATE_COMPASS

QMC5883LCompass compass;

void setup() {
  Serial.begin(115200);
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  compass.init();

  Serial.println(
      "This will provide calibration settings for your QMC5883L chip. When "
      "prompted, move the magnetometer in all directions until the calibration "
      "is complete.");
  Serial.println("Calibration will begin in 10 seconds.");
  delay(10000);

  Serial.println("CALIBRATING. Keep moving your sensor...");
  compass.calibrate();

  Serial.println(
      "DONE. Copy the lines below and paste it into your projects sketch.);");
  Serial.println();
  Serial.print("compass.setCalibrationOffsets(");
  Serial.print(compass.getCalibrationOffset(0));
  Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(1));
  Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(2));
  Serial.println(");");
  Serial.print("compass.setCalibrationScales(");
  Serial.print(compass.getCalibrationScale(0));
  Serial.print(", ");
  Serial.print(compass.getCalibrationScale(1));
  Serial.print(", ");
  Serial.print(compass.getCalibrationScale(2));
  Serial.println(");");
}

void loop() { delay(10000); }

#elif CALIBRATE_GYRO

GY521 sensor(GYRO_ADDRESS);

uint32_t counter = 0;

float ax, ay, az;
float gx, gy, gz;
float t;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("GY521_LIB_VERSION: ");
  Serial.println(GY521_LIB_VERSION);

  Wire.begin();
  delay(100);
  if (sensor.wakeup() == false) {
    Serial.println(
        "\tCould not connect to GY521: please check the GY521 address "
        "(0x68/0x69)");
  }
  //  adjust when needed.
  sensor.setAccelSensitivity(0);  //  2g
  sensor.setGyroSensitivity(0);   //  250 degrees/s
  sensor.setThrottle(false);

  //  set all calibration errors to zero
  sensor.axe = 0;
  sensor.aye = 0;
  sensor.aze = 0;
  sensor.gxe = 0;
  sensor.gye = 0;
  sensor.gze = 0;

  Serial.println("\n\nReading calibration numbers...");
}

void loop() {
  ax = ay = az = 0;
  gx = gy = gz = 0;
  t = 0;
  for (int i = 0; i < 100; i++) {
    sensor.read();
    ax -= sensor.getAccelX();
    ay -= sensor.getAccelY();
    az -= sensor.getAccelZ();
    gx -= sensor.getGyroX();
    gy -= sensor.getGyroY();
    gz -= sensor.getGyroZ();
    t += sensor.getTemperature();
  }

  if (counter % 10 == 0) {  
    Serial.println("\n\tCOPY CODE SNIPPET");
    Serial.print("gyro.axe = ");
    Serial.print(sensor.axe, 7);
    Serial.print(";\n");

    Serial.print("gyro.aye = ");
    Serial.print(sensor.aye, 7);
    Serial.print(";\n");

    Serial.print("gyro.aze = ");
    Serial.print(sensor.aze, 7);
    Serial.print(";\n");

    Serial.print("gyro.gxe = ");
    Serial.print(sensor.gxe, 7);
    Serial.print(";\n");

    Serial.print("gyro.gye = ");
    Serial.print(sensor.gye, 7);
    Serial.print(";\n");

    Serial.print("gyro.gze = ");
    Serial.print(sensor.gze, 7);
    Serial.print(";\n");

    Serial.println("\taxe\taye\taze\tgxe\tgye\tgze\tT");
  }

  if (counter % 10 == 0) {
    Serial.print(counter);
    Serial.print('\t');
    Serial.print(ax * 0.01, 3);
    Serial.print('\t');
    Serial.print(ay * 0.01, 3);
    Serial.print('\t');
    Serial.print(az * 0.01, 3);
    Serial.print('\t');
    Serial.print(gx * 0.01, 3);
    Serial.print('\t');
    Serial.print(gy * 0.01, 3);
    Serial.print('\t');
    Serial.print(gz * 0.01, 3);
    Serial.print('\t');
    Serial.print(t * 0.01, 2);
    Serial.println();
  }
  //  adjust calibration errors so table should get all zero's.
  sensor.axe += ax * 0.01;
  sensor.aye += ay * 0.01;
  sensor.aze += az * 0.01;
  sensor.gxe += gx * 0.01;
  sensor.gye += gy * 0.01;
  sensor.gze += gz * 0.01;

  counter++;
}

#else

PicoBoatController boat;

void setup() { boat.begin(); }

void loop() { vTaskDelete(NULL); }

void setup1() {}

void loop1() { vTaskDelete(NULL); }

#endif
