#include "gpio.h"

#define PIN 13

void app_main(void) {
  init(PIN, OUTPUT);
  set(PIN, HIGH);
}