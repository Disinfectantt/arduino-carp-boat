#include "./arduino/arduino.h"

ArduinoBoatController *boat;

void setup() { boat = new ArduinoBoatController(); }

void loop() { boat->loop(); }
