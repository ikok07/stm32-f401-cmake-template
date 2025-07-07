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
 * ADC GPIOs: PA3, PA4 (ADC1_IN3, ADC1_IN4)
*/

#define LED_PIN             13
#define BTN_PIN             0
#define ADC1_PIN            3
#define ADC2_PIN            4
#define TIM_PIN             8

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
        .ContinuousModeEnabled = DISABLE,
        .RegularExternalTrigger = ADC_ExternalTriggerREdge,
        .RegularExternalEvent = ADC_RegularExternalEventTIM1CC1,
        .SampledRegularChannelsSequence = {ADC_Channel3, ADC_Channel4},
        .SampledRegularChannelsSequenceLength = 2,
        .SampledInjectedChannelsSequenceLength = 0,
        .DMAConfig = {
            .Enabled = ENABLE
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

    // Init ADC GPIOs
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = ADC1_PIN;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl =  GPIO_NoPuPd;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeAnalog;
    GPIO_Init(&gpioHandle);

    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = ADC2_PIN;
    GPIO_Init(&gpioHandle);

    // Start SysTick
    Generic_InitSysTick();

    // Init ADC
    ADC_PeriClockControl(ENABLE);
    ADC_Error_e err = ADC_Init(&adcHandle);
    ADC_PeripheralControl(ENABLE);
    err = ADC_ConfigureDMA(&adcHandle);

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

    TIM_EnableInterrupts(&timHandle, TIM_IT_CC1);

    TIM_AdvancedTimerIRQPair_t irqPairs[1] = {
        {
            .IRQNumber = TIM1_CC_IRQn,
            .Priority = 1
        }
    };
    TIM_IRQTIM1Enable(irqPairs, 1);

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

void TIM1_CC_IRQHandler() {
    if (TIM_GetStatusFlag(&timHandle, TIM_FlagCC1)) {
        TIM_ClearStatusFlag(&timHandle, TIM_FlagCC1);
        GPIO_ToggleOutputPin(GPIOC, LED_PIN);
    }
}
