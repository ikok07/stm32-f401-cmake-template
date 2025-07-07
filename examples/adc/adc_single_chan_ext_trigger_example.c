//
// Created by Kok on 7/7/25.
//


#include <clock_driver.h>
#include <generic_methods.h>
#include <string.h>

#include "gpio_driver.h"
#include "adc_driver.h"
#include "timer_driver.h"

/*
 * ADC GPIOs: PA4 (ADC1_IN4)
*/

#define LED_PIN             13
#define BTN_PIN             0
#define ADC_PIN             4
#define TIM_PIN             8

volatile uint8_t button_trigger = 0;
uint16_t adcBuffer[2];

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
        .ScanModeEnabled = DISABLE,
        .AutoInjectedGroupConversion = DISABLE,
        .ContinuousModeEnabled = DISABLE,
        .RegularExternalTrigger = ADC_ExternalTriggerREdge,
        .RegularExternalEvent = ADC_RegularExternalEventTIM1CC1,
        .SampledRegularChannelsSequence = {ADC_Channel4},
        .SampledRegularChannelsSequenceLength = 1,
        .SampledInjectedChannelsSequenceLength = 0,
        .DMAConfig = {
            .Enabled = DISABLE
        }
    }
};

TIM_Handle_t timHandle = {
    .pTIMx = TIM1,
    .TIM_Config = {
        .Direction = TIM_DirectionUp,
        .UpdateEventToggle = TIM_UpdateEventEnabled,
        .UpdateRequestSourceMode = TIM_UpdateReqSrcMode1,
        .PreloadEnabled = ENABLE
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

    // Init ADC GPIO
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = ADC_PIN;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl =  GPIO_NoPuPd;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeAnalog;
    GPIO_Init(&gpioHandle);

    // Start SysTick
    Generic_InitSysTick();

    // Init ADC
    ADC_PeriClockControl(ENABLE);
    ADC_Error_e err = ADC_Init(&adcHandle);
    ADC_PeripheralControl(ENABLE);

    ADC_Interrupt_e adcInterrupts[] = {ADC_InterruptEOC};
    ADC_EnableInterrupts(&adcHandle, adcInterrupts, 1);
    ADC_IRQEnable(1);
    ADC_ConfigureExternalTriggerIT(&adcHandle, adcBuffer, 1);

    // Init TIM1
    TIM_PeriClockControl(timHandle.pTIMx, ENABLE);
    TIM_Init(&timHandle);

    TIM_OutputCompareConfig_t outputCompareConfig = {
        .Mode = TIM_OutputCompareMode4,
        .Preload = ENABLE
    };
    TIM_ConfigureOutputCompare(&timHandle, TIM_CaptureCompareChan1, outputCompareConfig);
    TIM_CaptureCompareChannelControl(&timHandle, TIM_CaptureCompareChan1, ENABLE);

    uint32_t pclk2 = CLOCK_GetApb2Hz();
    TIM_SetPrescaler(&timHandle, ((pclk2 / 1000000) * 1000) - 1);        // 1KHz
    TIM_SetAutoReload(&timHandle, 1000 - 1);        // 1s
    TIM_SetCompareValue(&timHandle, TIM_CaptureCompareChan1, 1000 - 1);

    // Reset ARR
    TIM_GenerateUpdateEvent(&timHandle);

    // Start timer
    TIM_Start(&timHandle);
    TIM_MasterMainOutputControl(&timHandle, ENABLE);

    while (1) {
        if (button_trigger) {
            // Small delay because of debouncing
            for (int i = 0; i < 1000000; i++);

            GPIO_ToggleOutputPin(GPIOC, LED_PIN);

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
    if (Flag == ADC_FlagEndOfRegularConversion) {
        uint16_t value = *pADCHandle->ITState.pBuffer;
        (void)value;
    }
}
