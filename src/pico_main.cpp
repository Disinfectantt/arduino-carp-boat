#include "./pico/pico.h"

PicoBoatController *boat;

void setup() { boat = new PicoBoatController(); }

void loop() { vTaskDelete(NULL); }

void setup1() {}

void loop1() { vTaskDelete(NULL); }
