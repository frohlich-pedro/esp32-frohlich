#include "gpio/gpio.h"

void app_main(void) {
  gpio_t led = {
    .pin = 13,
    .mode = OUTPUT,
    .state = LOW
  };
  
  gpio_t but = {
    .pin = 12,
    .mode = INPUT_PULLUP,
    .state = HIGH
  };
  
  gpio_op[OP_INIT](&led, NULL);
  gpio_op[OP_INIT](&but, NULL);

  while (true) {
    int r = gpio_op[OP_READ](&but, NULL);
    if (r == GPIO_OK) {
      if (!(button.state == HIGH)) {
        gpio_op[OP_TOGGLE](&led, NULL);
      }
    }
  }
}
