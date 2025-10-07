#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>

#define INTERNAL_ROM_0 0x40000000
#define INTERNAL_ROM_1 0x3FF90000
#define INTERNAL_SRAM_0 0x40070000
#define INTERNAL_SRAM_1_1 0x3FFE0000
#define INTERNAL_SRAM_1_2 0x400A0000
#define RTC_FAST_MEMORY_1 0x3FF80000
#define RTC_FAST_MEMORY_2 0x400C0000
#define RTC_SLOW_MEMORY 0x50000000

#define EXTERNAL_FLASH_1 0x3F400000
#define EXTERNAL_FLASH_2 0x400C2000
#define EXTERNAL_RAM 0x3F800000

#define DPORT_REGISTER 0x3FF00000
#define AES_ACCELERATOR 0x3FF01000
#define RSA_ACCELERATOR 0x3FF02000
#define SHA_ACCELERATOR 0x3FF03000
#define SECURE_BOOT 0x3FF04000
#define CACHE_MMU_TABLE 0x3FF10000
#define PID_CONTROLLER 0x3FF1F000
#define UART0 0x3FF40000
#define SPI1 0x3FF42000
#define SPI0 0x3FF43000
#define GPIO 0x3FF44000
#define RTC 0x3FF48000
#define IO_MUX 0x3FF49000
#define SDIO_SLAVE_1 0x3FF4B000
#define UDMA1 0x3FF4C000
#define I2S0 0x3FF4F000
#define UART1 0x3FF50000
#define I2C0 0x3FF53000
#define UDMA0 0x3FF53000
#define SDIO_SLAVE_2 0x3FF55000
#define RMT 0x3FF56000
#define PCNT 0x3FF57000
#define SDIO_SLAVE_3 0x3FF58000
#define LED_PWM 0x3FF59000
#define EFUSE_CONTROLLER 0x3FF5A000
#define FLASH_ENCRYPTION 0x3FF5B000
#define PWM0 0x3FF5E000
#define TIMG0 0x3FF5F000
#define TIMG1 0x3FF60000
#define SPI2 0x3FF64000
#define SPI3 0x3FF65000

#define SYSCON 0x3FF66000
#define I2C1 0x3FF67000
#define SDMMC 0x3FF68000
#define EMAC 0x3FF69000
#define TWAI 0x3FF6B000
#define PWM1 0x3FF6C000
#define I2S1 0x3FF6D000
#define UART2 0x3FF6E000
#define PWM2 0x3FF6F000
#define PWM3 0x3FF70000
#define RNG 0x3FF75000

typedef enum {
    GPIO_INPUT,
    GPIO_OUTPUT,
    GPIO_INPUT_PULLUP,
    GPIO_INPUT_PULLDOWN
} gpio_mode_t;

typedef enum {
    GPIO_LOW,
    GPIO_HIGH
} gpio_state_t;

typedef enum {
    GPIO_OK,
    GPIO_ERR_INVALID_PIN,
    GPIO_ERR_UNSUPPORTED,
    GPIO_ERR_NULL,
    GPIO_ERR_UNKNOWN
} gpio_error_t;

#define INPUT     (1 << 0)
#define OUTPUT    (1 << 1)
#define SAFE      (1 << 2)
#define STRAPPING (1 << 3)
#define ADC       (1 << 4)
#define PWM       (1 << 5)

typedef struct {
    uint8_t pin;
    uint32_t flags;
} gpio_capability_t;

typedef struct {
    uint8_t pin;
    gpio_mode_t mode;
    gpio_state_t state;
} gpio_t;

typedef int (*gpio_op_t)(gpio_t*, void*);

int gpio_init(gpio_t* gpio, void* args);
int gpio_write(gpio_t* gpio, void* args);
int gpio_read(gpio_t* gpio, void* args);
int gpio_toggle(gpio_t* gpio, void* args);
int gpio_analog_read(gpio_t* gpio, void* args);
int gpio_analog_write(gpio_t* gpio, void* args);
int gpio_attach_interrupt(gpio_t* gpio, void* args);
int gpio_debounce(gpio_t* gpio, void* args);
int gpio_sleep(gpio_t* gpio, void* args);
int gpio_wake(gpio_t* gpio, void* args);

enum {
    OP_INIT,
    OP_WRITE,
    OP_READ,
    OP_TOGGLE,
    OP_ANALOG_READ,
    OP_ANALOG_WRITE,
    OP_ATTACH_INTERRUPT,
    OP_DEBOUNCE,
    OP_SLEEP,
    OP_WAKE,
    OP_COUNT
};

gpio_op_t gpio_op[OP_COUNT] = {
    gpio_init,
    gpio_write,
    gpio_read,
    gpio_toggle,
    gpio_analog_read,
    gpio_analog_write,
    gpio_attach_interrupt,
    gpio_debounce,
    gpio_sleep,
    gpio_wake
};

#endif