#include "led.h"

#include "fsl_gpio.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "fsl_iomuxc.h"
#pragma GCC diagnostic pop

#define USER_LED_GPIO     GPIO9
#define USER_LED_GPIO_PIN (3U)

namespace led {
    void init() {

        gpio_pin_config_t gpio_config = {.direction     = kGPIO_DigitalOutput,
                                         .outputLogic   = 0U,
                                         .interruptMode = kGPIO_NoIntmode};

        GPIO_PinInit(USER_LED_GPIO, USER_LED_GPIO_PIN, &gpio_config);

        CLOCK_EnableClock(kCLOCK_Iomuxc);
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_04_GPIO9_IO03, 0U);
    }

    void on() { GPIO_PortSet(USER_LED_GPIO, (1U << USER_LED_GPIO_PIN)); }

    void off() { GPIO_PortClear(USER_LED_GPIO, (1U << USER_LED_GPIO_PIN)); }

    void toggle() {
        GPIO_PinWrite(
            USER_LED_GPIO,
            USER_LED_GPIO_PIN,
            (uint8_t)(0x1 ^ GPIO_PinRead(USER_LED_GPIO, USER_LED_GPIO_PIN)));
    }
}
