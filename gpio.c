#include "gpio.h"

int gpio_init(gpio_t* gpio, void* args) {
    if (!gpio) return GPIO_ERR_NULL;
    if (gpio->pin > 39) return GPIO_ERR_INVALID_PIN;

    volatile uint32_t* dir_reg = (uint32_t*)(GPIO + 0x04);
    switch (gpio->mode) {
    case GPIO_OUTPUT:
        *dir_reg |= (1 << gpio->pin);
        break;
    case GPIO_INPUT:
    case GPIO_INPUT_PULLUP:
    case GPIO_INPUT_PULLDOWN:
        *dir_reg &= ~(1 << gpio->pin);
        break;
    default:
        return GPIO_ERR_UNSUPPORTED;
    }

    volatile uint32_t* pad_reg = (uint32_t*)(IO_MUX + gpio->pin * 4);
    if (gpio->mode == GPIO_INPUT_PULLUP) {
        *pad_reg |= (1 << 7);
    } else if (gpio->mode == GPIO_INPUT_PULLDOWN) {
        *pad_reg |= (1 << 6);
    }

    return GPIO_OK;
}

int gpio_write(gpio_t* gpio, void* args) {
    if (!gpio) return GPIO_ERR_NULL;

    volatile uint32_t* out_reg = (uint32_t*)(GPIO + 0x08);
    if (gpio->state == GPIO_HIGH) {
        *out_reg |= (1 << gpio->pin);
    } else {
        *out_reg &= ~(1 << gpio->pin);
    }

    return GPIO_OK;
}

int gpio_read(gpio_t* gpio, void* args) {
    if (!gpio) return GPIO_ERR_NULL;

    volatile uint32_t* in_reg = (uint32_t*)(GPIO + 0x3C);
    gpio->state = ((*in_reg >> gpio->pin) & 1) ? GPIO_HIGH : GPIO_LOW;

    return GPIO_OK;
}

int gpio_toggle(gpio_t* gpio, void* args) {
    if (!gpio) return GPIO_ERR_NULL;

    volatile uint32_t* out_reg = (uint32_t*)(GPIO + 0x08);
    *out_reg ^= (1 << gpio->pin);

    gpio->state = ((*out_reg >> gpio->pin) & 1) ? GPIO_HIGH : GPIO_LOW;
    return GPIO_OK;
}

int gpio_analog_read(gpio_t* gpio, void* args) {

}

int gpio_analog_write(gpio_t* gpio, void* args) {

}

int gpio_attach_interrupt(gpio_t* gpio, void* args) {

}

int gpio_debounce(gpio_t* gpio, void* args) {

}

int gpio_sleep(gpio_t* gpio, void* args) {

}

int gpio_wake(gpio_t* gpio, void* args) {

}