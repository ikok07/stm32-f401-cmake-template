
#include <clock_driver.h>
#include <generic_methods.h>
#include <power_driver.h>
#include <string.h>

#include "gpio_driver.h"
#include "timer_driver.h"
#include "rtc_driver.h"

#define RTC_AF1_PIN         13
#define BTN_PIN             0
#define TIM_PIN             8

volatile uint8_t button_trigger = 0;
uint16_t adcBuffer[2];

PWR_Handle_t pwrHandle = {
    .Config = {
        .BackupRegulatorEnabled = DISABLE,
        .RegulatorVoltageScaling = PWR_RegulatorScale2,
        .PVDEnabled = DISABLE,
        .LowPowerRegulatorInStopMode = ENABLE,
        .LowPowerRegulatorLowVoltageEnabled = ENABLE,
        .FlashPowerDownInStopMode = ENABLE,
        .WakeUpConfig = {
            .Source = PWR_WakeUpSrcInterrupt,
            .WKUPPinEnabled = DISABLE
        }
    }
};

GPIO_Handle_t gpioHandle = {
    .pGPIOx = GPIOC,
    .GPIO_PinConfig = {
        .GPIO_PinNumber = RTC_AF1_PIN,
        .GPIO_PinMode = GPIO_ModeOutput,
        .GPIO_PinOPType = GPIO_OpTypePP,
        .GPIO_PinPuPdControl = GPIO_NoPuPd,
        .GPIO_PinSpeed = GPIO_SpeedHigh
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

RTC_Handle_t rtcHandle = {
    .Config = {
        .ClockSource = RTC_ClkLSE,
        .HourFormat = RTC_HourFormat24,
        .AsyncPrescaler = 127,
        .SyncPrescaler = 255,
        .StartDate = {
            .Day = 10,
            .Month = 7,
            .Year = 25,
            .Hours = 16,
            .Minutes = 0,
            .Seconds = 0,
            .Notation = RTC_NotationAM_24H,
            .Weekday = RTC_Thursday
        }
    }
};

int main(void) {
    // Configure power options
    PWR_PeriClockControl(ENABLE);
    PWR_UnlockBackupRegisters();
    PWR_Init(&pwrHandle);

    // Init the LED
    GPIO_Init(&gpioHandle);

    // Init the BTN
    gpioHandle.pGPIOx = GPIOA;
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = BTN_PIN;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_Pu;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeInputFEdge;
    GPIO_Init(&gpioHandle);
    GPIO_IRQConfig(BTN_PIN, 1, ENABLE);

    // Start SysTick
    Generic_InitSysTick();

    // Init TIM1
    TIM_PeriClockControl(timHandle.pTIMx, ENABLE);
    TIM_Init(&timHandle);

    // Init RTC
    CLOCK_ResetBackupDomain();
    RTC_ClockControl(ENABLE);
    RTC_Error_e err = RTC_Init(&rtcHandle);
    (void)err;
    Generic_Delay(5000);
    RTC_CalendarDate_t date;
    RTC_ReadCalendar(&rtcHandle, &date);

    TIM_OutputCompareConfig_t outputCompareConfig = {
        .Mode = TIM_OutputCompareMode4,
        .Preload = ENABLE
    };
    TIM_ConfigureOutputCompare(&timHandle, TIM_CaptureCompareChan4, outputCompareConfig);
    TIM_CaptureCompareChannelControl(&timHandle, TIM_CaptureCompareChan4, ENABLE);

    uint32_t pclk2 = CLOCK_GetApb2Hz();
    TIM_SetPrescaler(&timHandle, ((pclk2 / 1000000) * 1000) - 1);        // 1KHz
    TIM_SetAutoReload(&timHandle, 1000 - 1);        // 1s
    TIM_SetCompareValue(&timHandle, TIM_CaptureCompareChan4, 1000 - 1);

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

    while (1) {
        if (button_trigger) {
            // Small delay because of debouncing
            for (int i = 0; i < 1000000; i++);

            // Start timer
            TIM_Start(&timHandle);

            // Stop SysTick
            SYSTICK_CounterControl(DISABLE);

            PWR_EnterSleepMode(&pwrHandle);

            // Start SysTick
            SYSTICK_CounterControl(DISABLE);

            button_trigger = 0;
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}

void TIM1_CC_IRQHandler() {
    if (TIM_GetStatusFlag(&timHandle, TIM_FlagCC1)) {
        TIM_ClearStatusFlag(&timHandle, TIM_FlagCC1);
    }
}