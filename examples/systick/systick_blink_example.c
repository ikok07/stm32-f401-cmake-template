//
// Created by Kok on 6/30/25.
//

#include <clock_driver.h>
#include <flash_driver.h>
#include <generic_methods.h>
#include <string.h>

#include "stm32f4xx.h"
#include "gpio_driver.h"

#define LED_PIN             13          // PORT C
#define BTN_PIN             0           // PORT A

volatile uint8_t button_trigger = 0;

GPIO_Handle_t gpioHandle = {
    .pGPIOx = GPIOC,
    .GPIO_PinConfig = {
        .GPIO_PinNumber = LED_PIN,
        .GPIO_PinMode = GPIO_ModeOutput,
        .GPIO_PinOPType = GPIO_OpTypePP,
        .GPIO_PinPuPdControl = GPIO_NoPuPd,
        .GPIO_PinSpeed = GPIO_SpeedHigh
    }
};

void SystemInit() {
    // Setup for 84MHz

    CLOCK_SelectSysClock(CLOCK_SysClockHSI);
    CLOCK_Disable(CLOCK_SrcPLL);
    FLASH_SetLatency(FLASH_WaitState1);
    CLOCK_SetPLLFactors(16, 336, CLOCK_PLLSysClkPresc4);
    CLOCK_Enable(CLOCK_SrcPLL);
    CLOCK_SelectSysClock(CLOCK_SysClockPLL);
    CLOCK_SetAHBBusPrescaler(CLOCK_AHBPresc1);
    CLOCK_SetAPBBusPrescaler(CLOCK_BusAPB1, CLOCK_APBPresc2);
    CLOCK_SetAPBBusPrescaler(CLOCK_BusAPB2, CLOCK_APBPresc1);
}

int main(void) {
    // Init the LED
    GPIO_Init(&gpioHandle);

    // Init the BTN
    gpioHandle.pGPIOx = GPIOA;
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = BTN_PIN;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_Pu;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeInputFEdge;
    GPIO_Init(&gpioHandle);
    GPIO_IRQConfig(BTN_PIN, 1, ENABLE);

    while (1) {
        Generic_Delay(1000);
        GPIO_ToggleOutputPin(GPIOC, LED_PIN);
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}
