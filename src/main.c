#include <stdio.h>

#include "stm32f4xx.h"
#include "gpio_driver.h"

#define LED_PIN             0
#define BTN_PIN             1

int main(void) {
    GPIO_Handle_t gpio_handle = {
        .pGPIOx = GPIOA,
        .GPIO_PinConfig = {
            .GPIO_PinNumber = LED_PIN,
            .GPIO_PinMode = GPIO_MODE_OUTPUT,
            .GPIO_PinOPType = GPIO_OP_TYPE_PP,
            .GPIO_PinPuPdControl = GPIO_PD,
            .GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH
        }
    };

    // Init the LED
    GPIO_Init(&gpio_handle);

    // Init the BTN
    gpio_handle.GPIO_PinConfig.GPIO_PinNumber = BTN_PIN;
    gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT_R_EDGE;
    GPIO_Init(&gpio_handle);
    GPIO_IRQConfig(BTN_PIN, 1, ENABLE);

    while (1);

    return 0;
}

void EXTI9_5_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    GPIO_ToggleOutputPin(GPIOA, LED_PIN);
}