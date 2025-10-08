# ESP32-Frohlich GPIO Library

A high-performance, Data-Oriented Design (DOD) GPIO implementation for ESP32 microcontrollers, designed to minimize cache misses and maximize real-time performance through function pointer arrays.

## Overview

This library provides a zero-cache-miss GPIO interface for ESP32 using Data-Oriented Design principles. By utilizing function pointer arrays and direct register manipulation, it achieves superior performance compared to traditional object-oriented approaches.

## Architecture

### Data-Oriented Design Approach

The library centers around a `gpio_t` data structure and a function pointer array `gpio_op[]` that provides direct, cache-efficient access to GPIO operations:

```c
typedef struct {
  uint8_t pin;
  gpio_mode_t mode;
  gpio_state_t state;
} gpio_t;

gpio_op_t gpio_op[] = {
  gpio_init,    // OP_INIT
  gpio_write,   // OP_WRITE  
  gpio_read,    // OP_READ
  gpio_toggle   // OP_TOGGLE
};

```

Code example:
```c
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
```
