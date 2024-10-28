#include "./esp32/esp32.h"

Esp32Controller esp32;

void setup() { esp32.begin(); }

void loop() {
#ifdef DEBUG_MODE
  Serial.printf("min free heap %d\n", esp_get_minimum_free_heap_size());
  vTaskDelay(40000 / portTICK_PERIOD_MS);
#else
  vTaskDelete(NULL);
#endif
}
