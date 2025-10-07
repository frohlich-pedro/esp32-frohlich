#include "gpio.h"

static const uint32_t gpio_iomux_reg[GPIO_PIN_COUNT] = {
  IO_MUX_GPIO0_REG,
  IO_MUX_GPIO1_REG,
  IO_MUX_GPIO2_REG,
  IO_MUX_GPIO3_REG,
  IO_MUX_GPIO4_REG,
  IO_MUX_GPIO5_REG,
  IO_MUX_GPIO6_REG,
  IO_MUX_GPIO7_REG,
  IO_MUX_GPIO8_REG,
  IO_MUX_GPIO9_REG,
  IO_MUX_GPIO10_REG,
  IO_MUX_GPIO11_REG,
  IO_MUX_GPIO12_REG,
  IO_MUX_GPIO13_REG,
  IO_MUX_GPIO14_REG,
  IO_MUX_GPIO15_REG,
  IO_MUX_GPIO16_REG,
  IO_MUX_GPIO17_REG,
  IO_MUX_GPIO18_REG,
  IO_MUX_GPIO19_REG,
  0,
  IO_MUX_GPIO21_REG,
  IO_MUX_GPIO22_REG,
  IO_MUX_GPIO23_REG,
  0,
  IO_MUX_GPIO25_REG,
  IO_MUX_GPIO26_REG,
  IO_MUX_GPIO27_REG,
  0,
  0,
  0,
  0,
  IO_MUX_GPIO32_REG,
  IO_MUX_GPIO33_REG,
  IO_MUX_GPIO34_REG,
  IO_MUX_GPIO35_REG,
  IO_MUX_GPIO36_REG,
  IO_MUX_GPIO37_REG,
  IO_MUX_GPIO38_REG,
  IO_MUX_GPIO39_REG
};

static const uint8_t gpio_capabilities[GPIO_PIN_COUNT] = {
  INPUT | OUTPUT | GPIO_STRAPPING | GPIO_PWM,
  INPUT | OUTPUT | GPIO_PWM,
  INPUT | OUTPUT | GPIO_STRAPPING | GPIO_PWM | GPIO_ADC,
  INPUT | OUTPUT | GPIO_PWM,
  INPUT | OUTPUT | GPIO_PWM | GPIO_ADC,
  INPUT | OUTPUT | GPIO_STRAPPING | GPIO_PWM,
  0,
  0,
  0,
  0,
  0,
  0,
  INPUT | OUTPUT | GPIO_STRAPPING | GPIO_PWM | GPIO_ADC,
  INPUT | OUTPUT | GPIO_PWM | GPIO_ADC,
  INPUT | OUTPUT | GPIO_PWM | GPIO_ADC,
  INPUT | OUTPUT | GPIO_STRAPPING | GPIO_PWM | GPIO_ADC,
  INPUT | OUTPUT | GPIO_PWM,
  INPUT | OUTPUT | GPIO_PWM,
  INPUT | OUTPUT | GPIO_PWM,
  INPUT | OUTPUT | GPIO_PWM,
  0,
  INPUT | OUTPUT | GPIO_PWM,
  INPUT | OUTPUT | GPIO_PWM,
  INPUT | OUTPUT | GPIO_PWM,
  0,
  INPUT | OUTPUT | GPIO_PWM | GPIO_ADC,
  INPUT | OUTPUT | GPIO_PWM | GPIO_ADC,
  INPUT | OUTPUT | GPIO_PWM | GPIO_ADC,
  0,
  0,
  0,
  0,
  INPUT | OUTPUT | GPIO_PWM | GPIO_ADC,
  INPUT | OUTPUT | GPIO_PWM | GPIO_ADC,
  INPUT | GPIO_ADC,
  INPUT | GPIO_ADC,
  INPUT | GPIO_ADC,
  INPUT | GPIO_ADC,
  INPUT | GPIO_ADC,
  INPUT | GPIO_ADC
};

int gpio_init(gpio_t *gpio, void *args) {
  if (!gpio) return GPIO_ERR_NULL;
  if (!gpio_is_valid_pin(gpio->pin)) return GPIO_ERR_INVALID_PIN;
  if (gpio_is_flash_pin(gpio->pin)) return GPIO_ERR_FLASH_PIN;

  uint8_t pin = gpio->pin;

  uint8_t caps = *(gpio_capabilities + pin);
  if (gpio->mode == OUTPUT && !(caps & GPIO_OUTPUT)) return GPIO_ERR_UNSUPPORTED;
  if ((gpio->mode == INPUT || gpio->mode == INPUT_PULLUP || gpio->mode == INPUT_PULLDOWN) && !(caps & GPIO_INPUT)) return GPIO_ERR_UNSUPPORTED;

  if (pin < 32) {
    volatile uint32_t *enable_reg = (uint32_t *)GPIO_ENABLE_REG;
    if (gpio->mode == OUTPUT) {
      *enable_reg |= (1U << pin);
    } else {
      *enable_reg &= ~(1U << pin);
    }
  } else {
    volatile uint32_t *enable_reg = (uint32_t *)GPIO_ENABLE1_REG;
    if (gpio->mode == OUTPUT) {
      *enable_reg |= (1U << (pin - 32));
    } else {
      *enable_reg &= ~(1U << (pin - 32));
    }
  }

  uint32_t iomux_addr = gpio_get_iomux_reg(pin);
  if (iomux_addr != 0) {
    volatile uint32_t *iomux_reg = (uint32_t *)iomux_addr;
    uint32_t val = *iomux_reg;

    val &= ~(IO_MUX_FUN_WPU | IO_MUX_FUN_WPD);

    if (gpio->mode == INPUT_PULLUP) {
      val |= IO_MUX_FUN_WPU;
    } else if (gpio->mode == INPUT_PULLDOWN) {
      val |= IO_MUX_FUN_WPD;
    }

    val &= ~(IO_MUX_MCU_SEL_M << IO_MUX_MCU_SEL_S);
    val |= (IO_MUX_MCU_SEL_GPIO << IO_MUX_MCU_SEL_S);

    *iomux_reg = val;
  }

  return GPIO_OK;
}

int gpio_write(gpio_t *gpio, void *args) {
  if (!gpio) return GPIO_ERR_NULL;
  if (!gpio_is_valid_pin(gpio->pin)) return GPIO_ERR_INVALID_PIN;

  return gpio_write_fast(gpio->pin, gpio->state);
}

int gpio_read(gpio_t *gpio, void *args) {
  if (!gpio) return GPIO_ERR_NULL;
  if (!gpio_is_valid_pin(gpio->pin)) return GPIO_ERR_INVALID_PIN;

  gpio->state = gpio_read_fast(gpio->pin);
  return GPIO_OK;
}

int gpio_toggle(gpio_t *gpio, void *args) {
  if (!gpio) return GPIO_ERR_NULL;
  if (!gpio_is_valid_pin(gpio->pin)) return GPIO_ERR_INVALID_PIN;

  int result = gpio_toggle_fast(gpio->pin);
  if (result == GPIO_OK) gpio->state = gpio_read_fast(gpio->pin);

  return result;
}

int gpio_init_batch(gpio_batch_t *batch) {
  if (!batch || !batch->pins || !batch->modes) return GPIO_ERR_NULL;

  uint8_t *pin = batch->pins;
  uint8_t *mode = batch->modes;
  uint8_t *end = pin + batch->count;

  while (pin < end) {
    gpio_t gpio = {
      .pin = *pin++,
      .mode = *mode++,
      .state = LOW
    };

    int result = gpio_init(&gpio, NULL);
    if (result != GPIO_OK) return result;
  }

  return GPIO_OK;
}

int gpio_write_batch(gpio_batch_t *batch) {
  if (!batch || !batch->pins || !batch->states) return GPIO_ERR_NULL;

  uint32_t set_mask_0 = 0, clr_mask_0 = 0;
  uint32_t set_mask_1 = 0, clr_mask_1 = 0;

  uint8_t *pin = batch->pins;
  uint8_t *state = batch->states;
  uint8_t *end = pin + batch->count;

  while (pin < end) {
    uint8_t p = *pin++;
    uint8_t s = *state++;

    if (!gpio_is_valid_pin(p)) continue;

    if (p < 32) {
      if (s == HIGH) {
        set_mask_0 |= (1U << p);
      } else {
        clr_mask_0 |= (1U << p);
      }
    } else {
      if (s == HIGH) {
        set_mask_1 |= (1U << (p - 32));
      } else {
        clr_mask_1 |= (1U << (p - 32));
      }
    }
  }

  if (set_mask_0) *(volatile uint32_t *)GPIO_OUT_W1TS_REG = set_mask_0;
  if (clr_mask_0) *(volatile uint32_t *)GPIO_OUT_W1TC_REG = clr_mask_0;
  if (set_mask_1) *(volatile uint32_t *)GPIO_OUT1_W1TS_REG = set_mask_1;
  if (clr_mask_1) *(volatile uint32_t *)GPIO_OUT1_W1TC_REG = clr_mask_1;

  return GPIO_OK;
}

int gpio_read_batch(gpio_batch_t *batch) {
  if (!batch || !batch->pins || !batch->states) return GPIO_ERR_NULL;

  uint8_t *pin = batch->pins;
  uint8_t *state = batch->states;
  uint8_t *end = pin + batch->count;

  uint32_t val_0 = *(volatile uint32_t *)GPIO_IN_REG;
  uint32_t val_1 = *(volatile uint32_t *)GPIO_IN1_REG;

  while (pin < end) {
    uint8_t p = *pin++;
    if (!gpio_is_valid_pin(p)) {
      state++;
      continue;
    }

    *state++ = (p < 32) ? ((val_0 >> p) & 1U ? HIGH : LOW) : ((val_1 >> (p - 32)) & 1U ? HIGH : LOW);
  }

  return GPIO_OK;
}

int gpio_toggle_batch(gpio_batch_t *batch) {
  if (!batch || !batch->pins) return GPIO_ERR_NULL;

  uint8_t *pin = batch->pins;
  uint8_t *end = pin + batch->count;

  while (pin < end) {
    uint8_t p = *pin++;
    if (!gpio_is_valid_pin(p)) continue;
    gpio_toggle_fast(p);
  }

  return batch->states ? gpio_read_batch(batch) : GPIO_OK;
}

gpio_op_t gpio_op[OP_COUNT] = {
  gpio_init,
  gpio_write,
  gpio_read,
  gpio_toggle
};