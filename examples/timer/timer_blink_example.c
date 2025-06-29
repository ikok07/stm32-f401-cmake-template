//
// Created by Kok on 6/29/25.
//

#include <clock_driver.h>
#include <flash_driver.h>

#include <string.h>

#include "stm32f4xx.h"
#include "gpio_driver.h"
#include "timer_driver.h"

#define LED_PIN             13          // PORT C
#define BTN_PIN             0           // PORT A
#define TIM2_CH1_PIN        15          // PORT A

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

TIM_Handle_t timHandle = {
    .pTIMx = TIM2,
    .TIM_Config = {
        .Direction = TIM_DirectionUp,
        .PreloadEnabled = TIM_ARRPreloadEnabled,
        .OnePulseMode = TIM_OnePulseDisabled,
        .UpdateRequestSourceMode = TIM_UpdateReqSrcMode1,
        .UpdateEventToggle = TIM_UpdateEventEnabled,
        // .CenterModeSelection =
    }
};

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

    // Init TIM2 CH1
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = TIM2_CH1_PIN;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NoPuPd;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeAlternate;
    gpioHandle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF1;
    GPIO_Init(&gpioHandle);

    // Init timer
    TIM_PeriClockControl(timHandle.pTIMx, ENABLE);
    TIM_Init(&timHandle);

    TIM_OutputCompareConfig_t outputCompareConfig = {
        .Mode = TIM_OutputCompareMode4,
        .Preload = TIM_OutputComparePreloadEnabled
    };
    TIM_ConfigureOutputCompare(&timHandle, TIM_CaptureCompareChan1, outputCompareConfig);

    TIM_CaptureCompareChannelControl(&timHandle, TIM_CaptureCompareChan1, ENABLE);

    TIM_SetPrescaler(&timHandle, 16000 - 1);
    TIM_SetAutoReload(&timHandle, 1000 - 1);
    // TIM_SetAutoReload(&timHandle, 2 - 1);
    TIM_SetCompareValue(&timHandle, TIM_CaptureCompareChan1, 1000 - 1);
    // TIM_SetCompareValue(&timHandle, TIM_CaptureCompareChan1, 1);

    TIM_EnableInterrupts(&timHandle, TIM_IT_CC1);
    TIM_IRQEnable(&timHandle, 1);

    // Reset the auto reload register
    TIM_GenerateUpdateEvent(&timHandle);

    while (1) {
        if (button_trigger) {
            // Small delay because of debouncing
            for (int i = 0; i < 1000000; i++);

            TIM_Start(&timHandle);

            button_trigger = 0;
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}

void TIM2_IRQHandler() {
    if (TIM_GetStatusFlag(&timHandle, TIM_FlagUpdate)) {
        TIM_ClearStatusFlag(&timHandle, TIM_FlagUpdate);
        GPIO_ToggleOutputPin(GPIOC, LED_PIN);
    }
}

