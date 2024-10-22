#include "./esp32/esp32.h"

Esp32Controller *esp32 = nullptr;

void setup() { esp32 = new Esp32Controller(); }

void loop() { vTaskDelete(NULL); }
