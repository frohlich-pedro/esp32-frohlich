#include "gpio/gpio.h"

void app_main(void) {
  gpio_t led = {
    .pin = 13,
    .mode = OUTPUT,
    .state = LOW
  };
  
  gpio_t button = {
    .pin = 12,
    .mode = INPUT_PULLUP,
    .state = HIGH
  };
  
  gpio_op[OP_INIT](&led, NULL);
  gpio_op[OP_INIT](&button, NULL);

  while (true) {
    gpio_op[OP_READ](&button, NULL);
  
    if (!(button.state == HIGH)) {
      gpio_op[OP_TOGGLE](&led, NULL);
    }
  }
}
