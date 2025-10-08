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

#define GPIO_OUT_REG (GPIO + 0x0004)
#define GPIO_OUT_W1TS_REG (GPIO + 0x0008)
#define GPIO_OUT_W1TC_REG (GPIO + 0x000C)
#define GPIO_ENABLE_REG (GPIO + 0x0020)
#define GPIO_ENABLE_W1TS_REG (GPIO + 0x0024)
#define GPIO_ENABLE_W1TC_REG (GPIO + 0x0028)
#define GPIO_IN_REG (GPIO + 0x003C)

#define GPIO_OUT1_REG (GPIO + 0x0010)
#define GPIO_OUT1_W1TS_REG (GPIO + 0x0014)
#define GPIO_OUT1_W1TC_REG (GPIO + 0x0018)
#define GPIO_ENABLE1_REG (GPIO + 0x002C)
#define GPIO_ENABLE1_W1TS_REG (GPIO + 0x0030)
#define GPIO_ENABLE1_W1TC_REG (GPIO + 0x0034)
#define GPIO_IN1_REG (GPIO + 0x0040)

#define GPIO_STATUS_REG (GPIO + 0x0044)
#define GPIO_STATUS_W1TS_REG (GPIO + 0x0048)
#define GPIO_STATUS_W1TC_REG (GPIO + 0x004C)
#define GPIO_STATUS1_REG (GPIO + 0x0050)
#define GPIO_STATUS1_W1TS_REG (GPIO + 0x0054)
#define GPIO_STATUS1_W1TC_REG (GPIO + 0x0058)

#define GPIO_PIN_COUNT 40
#define GPIO_INVALID_PIN 0xFF

#define IO_MUX_GPIO0_REG (IO_MUX + 0x44)
#define IO_MUX_GPIO1_REG (IO_MUX + 0x88)
#define IO_MUX_GPIO2_REG (IO_MUX + 0x40)
#define IO_MUX_GPIO3_REG (IO_MUX + 0x84)
#define IO_MUX_GPIO4_REG (IO_MUX + 0x48)
#define IO_MUX_GPIO5_REG (IO_MUX + 0x6C)
#define IO_MUX_GPIO6_REG (IO_MUX + 0x60)
#define IO_MUX_GPIO7_REG (IO_MUX + 0x64)
#define IO_MUX_GPIO8_REG (IO_MUX + 0x68)
#define IO_MUX_GPIO9_REG (IO_MUX + 0x54)
#define IO_MUX_GPIO10_REG (IO_MUX + 0x58)
#define IO_MUX_GPIO11_REG (IO_MUX + 0x5C)
#define IO_MUX_GPIO12_REG (IO_MUX + 0x34)
#define IO_MUX_GPIO13_REG (IO_MUX + 0x38)
#define IO_MUX_GPIO14_REG (IO_MUX + 0x30)
#define IO_MUX_GPIO15_REG (IO_MUX + 0x3C)
#define IO_MUX_GPIO16_REG (IO_MUX + 0x4C)
#define IO_MUX_GPIO17_REG (IO_MUX + 0x50)
#define IO_MUX_GPIO18_REG (IO_MUX + 0x70)
#define IO_MUX_GPIO19_REG (IO_MUX + 0x74)
#define IO_MUX_GPIO21_REG (IO_MUX + 0x78)
#define IO_MUX_GPIO22_REG (IO_MUX + 0x7C)
#define IO_MUX_GPIO23_REG (IO_MUX + 0x80)
#define IO_MUX_GPIO25_REG (IO_MUX + 0x24)
#define IO_MUX_GPIO26_REG (IO_MUX + 0x28)
#define IO_MUX_GPIO27_REG (IO_MUX + 0x2C)
#define IO_MUX_GPIO32_REG (IO_MUX + 0x1C)
#define IO_MUX_GPIO33_REG (IO_MUX + 0x20)
#define IO_MUX_GPIO34_REG (IO_MUX + 0x14)
#define IO_MUX_GPIO35_REG (IO_MUX + 0x18)
#define IO_MUX_GPIO36_REG (IO_MUX + 0x04)
#define IO_MUX_GPIO37_REG (IO_MUX + 0x08)
#define IO_MUX_GPIO38_REG (IO_MUX + 0x0C)
#define IO_MUX_GPIO39_REG (IO_MUX + 0x10)

#define IO_MUX_FUN_DRV_S 10
#define IO_MUX_FUN_DRV_M 0x3
#define IO_MUX_FUN_IE (1 << 9)
#define IO_MUX_FUN_WPU (1 << 7)
#define IO_MUX_FUN_WPD (1 << 8)
#define IO_MUX_MCU_SEL_S 0
#define IO_MUX_MCU_SEL_M 0x7
#define IO_MUX_MCU_SEL_GPIO 2

typedef enum {
  INPUT,
  OUTPUT,
  INPUT_PULLUP,
  INPUT_PULLDOWN
} gpio_mode_t;

typedef enum {
  LOW,
  HIGH
} gpio_state_t;

typedef enum {
  GPIO_OK = 0,
  GPIO_ERR_INVALID_PIN = -1,
  GPIO_ERR_UNSUPPORTED = -2,
  GPIO_ERR_NULL = -3,
  GPIO_ERR_FLASH_PIN = -4,
  GPIO_ERR_UNKNOWN = -5
} gpio_error_t;

#define GPIO_INPUT (1 << 0)
#define GPIO_OUTPUT (1 << 1)
#define GPIO_SAFE (1 << 2)
#define GPIO_STRAPPING (1 << 3)
#define GPIO_ADC (1 << 4)
#define GPIO_PWM (1 << 5)

typedef struct {
  uint8_t pin;
  uint32_t flags;
} gpio_capability_t;

typedef struct {
  uint8_t pin;
  gpio_mode_t mode;
  gpio_state_t state;
} gpio_t;

typedef int (*gpio_op_t)(gpio_t *, void *);

int gpio_init(gpio_t *gpio, void *args);
int gpio_write(gpio_t *gpio, void *args);
int gpio_read(gpio_t *gpio, void *args);
int gpio_toggle(gpio_t *gpio, void *args);

static inline int gpio_write_fast(uint8_t pin, gpio_state_t state) {
  if (pin < 32) {
    volatile uint32_t *reg = (uint32_t *)(state ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG);
    *reg = (1U << pin);
  } else if (pin < 40) {
    volatile uint32_t *reg = (uint32_t *)(state ? GPIO_OUT1_W1TS_REG : GPIO_OUT1_W1TC_REG);
    *reg = (1U << (pin - 32));
  } else {
    return GPIO_ERR_INVALID_PIN;
  }

  return GPIO_OK;
}

static inline gpio_state_t gpio_read_fast(uint8_t pin) {
  if (pin < 32) {
    volatile uint32_t *reg = (uint32_t *)GPIO_IN_REG;
    return (*reg >> pin) & 1U ? HIGH : LOW;
  } else if (pin < 40) {
    volatile uint32_t *reg = (uint32_t *)GPIO_IN1_REG;
    return (*reg >> (pin - 32)) & 1U ? HIGH : LOW;
  }

  return LOW;
}

static inline int gpio_toggle_fast(uint8_t pin) {
  if (pin < 32) {
    volatile uint32_t *reg = (uint32_t *)GPIO_OUT_REG;
    *reg ^= (1U << pin);
  } else if (pin < 40) {
    volatile uint32_t *reg = (uint32_t *)GPIO_OUT1_REG;
    *reg ^= (1U << (pin - 32));
  } else {
    return GPIO_ERR_INVALID_PIN;
  }

  return GPIO_OK;
}

static inline bool gpio_is_valid_pin(uint8_t pin) {
  if (pin >= GPIO_PIN_COUNT) return false;
  if (pin == 20 || pin == 24 || (pin >= 28 && pin <= 31)) return false;
  
  return true;
}

static inline bool gpio_is_flash_pin(uint8_t pin) {
  return (pin >= 6 && pin <= 11);
}

static inline uint32_t gpio_get_iomux_reg(uint8_t pin) {
  if (pin >= GPIO_PIN_COUNT) return 0;
  
  return *(gpio_iomux_reg + pin);
}

extern gpio_op_t gpio_op[];

enum {
  OP_INIT,
  OP_WRITE,
  OP_READ,
  OP_TOGGLE,
  OP_COUNT
};

#endif
