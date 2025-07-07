//
// Created by Kok on 7/7/25.
//


#include <generic_methods.h>
#include <string.h>

#include "gpio_driver.h"
#include "adc_driver.h"

/*
 * ADC GPIOs: PA3 (ADC1_IN3), PA4 (ADC1_IN4)
*/

#define LED_PIN             13
#define BTN_PIN             0
#define ADC1_PIN            3
#define ADC2_PIN            4

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

ADC_Handle_t adcHandle = {
    .Config = {
        .Prescaler = ADC_Perscaler2,
        .Resolution = ADC_Resolution12bit,
        .DataAlignment = ADC_DataAlignRight,
        .SamplingTimes = {0},
        .ScanModeEnabled = ENABLE,
        .AutoInjectedGroupConversion = DISABLE,
        .ContinuousModeEnabled = DISABLE,           // Only for regular channels
        .WatchdogConfig = {
            .WatchdogMode = ADC_WatchdogSingleRegularChannel,
            .WatchdogChannel = ADC_WatchdogChan4,
            .WatchdogHigherThreshold = 3000,
            .WatchdogLowerThreshold = 1000
        },
        // .SampledRegularChannelsSequence = {ADC_Channel3, ADC_Channel4},
        .SampledRegularChannelsSequenceLength = 0,
        .SampledInjectedChannelsSequence = {ADC_Channel3, ADC_Channel4},
        .SampledInjectedChannelsSequenceLength = 2,
        .DMAConfig = {
            .Enabled = DISABLE
        }
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

    // Init ADC GPIOs
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = ADC1_PIN;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl =  GPIO_NoPuPd;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeAnalog;
    GPIO_Init(&gpioHandle);

    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = ADC2_PIN;
    GPIO_Init(&gpioHandle);

    // Init ADC
    ADC_PeriClockControl(ENABLE);
    ADC_Error_e err = ADC_Init(&adcHandle);

    ADC_Interrupt_e adcInterrupts[4] = {
        ADC_InterruptEOC,
        ADC_InterruptJEOC,
        ADC_InterruptAWD,
        ADC_InterruptOVR
    };
    ADC_EnableInterrupts(&adcHandle, adcInterrupts, 4);
    ADC_IRQEnable(1);

    ADC_PeripheralControl(ENABLE);

    Generic_InitSysTick();

    while (1) {
        if (button_trigger) {
            // Small delay because of debouncing
            for (int i = 0; i < 1000000; i++);

            GPIO_ToggleOutputPin(GPIOC, LED_PIN);

            uint16_t buffer[2] = {0};
            err = ADC_ReadMultipleInjectedChannelsIT(&adcHandle, buffer, 2);
            (void)err;
            button_trigger = 0;
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}

void ADC_IRQHandler() {
    ADC_IRQHandling(&adcHandle);
}

void ADC_ApplicationCallback(ADC_Handle_t *pADCHandle, ADC_Flag_e Flag) {
    uint16_t value;
    if (Flag == ADC_FlagEndOfRegularConversion) {
        value = *pADCHandle->ITState.pBuffer;
    } else if (Flag == ADC_FlagEndOfInjectedConversion) {
        value = *pADCHandle->ITState.pBuffer;
    } else if (Flag == ADC_FlagAnalogWatchdog) {
        GPIO_ToggleOutputPin(GPIOC, LED_PIN);
    }
    (void)value;
}