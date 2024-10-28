#include "./pico/pico.h"

PicoBoatController boat;

void setup() { boat.begin(); }

void loop() { vTaskDelete(NULL); }

void setup1() {}

void loop1() { vTaskDelete(NULL); }
