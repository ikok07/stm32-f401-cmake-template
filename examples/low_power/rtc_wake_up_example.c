//
// Created by Kok on 7/10/25.
//


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
        .BackupRegulatorEnabled = ENABLE,
        .RegulatorVoltageScaling = PWR_RegulatorScale2,
        .PVDEnabled = DISABLE,
        .LowPowerRegulatorInStopMode = ENABLE,
        .LowPowerRegulatorLowVoltageEnabled = ENABLE,
        .StandbyModeInDeepSleepEnabled = ENABLE,           // Stop or Standby Mode
        .FlashPowerDownInStopMode = ENABLE,
        .WakeUpConfig = {
            .Source = PWR_WakeUpSrcInterrupt,
            .WKUPPinEnabled = DISABLE        // PA0
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
    // Start SysTick
    Generic_InitSysTick();

    // Configure power options
    PWR_PeriClockControl(ENABLE);
    PWR_UnlockBackupRegisters();
    PWR_Error_e pwrErr = PWR_Init(&pwrHandle);
    (void)pwrErr;

    // Init the RTC AF1 Pin
    GPIO_Init(&gpioHandle);

    // Init the BTN
    gpioHandle.pGPIOx = GPIOA;
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = BTN_PIN;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_Pu;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeInputFEdge;
    GPIO_Init(&gpioHandle);
    GPIO_IRQConfig(BTN_PIN, 1, ENABLE);

    // Init RTC
    RTC_Error_e err;
    // if (!RTC_IsCalendarInitialized()) {
        CLOCK_ResetBackupDomain();
        RTC_ClockControl(ENABLE);
        err = RTC_Init(&rtcHandle);
        (void)err;
    // }

    RTC_AlarmConfig_t alarmConfig = {
        .AlarmX = RTC_AlarmA,
        // .Date = 10,
        // .Hours = 16,
        // .Minutes = 0,
        .Seconds = 10,
        .Notation = RTC_NotationAM_24H,
        .DateDisabled = ENABLE,
        .HoursDisabled = ENABLE,
        .MinutesDisabled = ENABLE,
    };
    err = RTC_ConfigureAlarm(&rtcHandle, alarmConfig);
    // RTC_SelectAlarmOutput(&rtcHandle, RTC_AlarmOutputWKUP, ENABLE, DISABLE);

    RTC_WakeUpTimerConfig_t timerConfig = {
        .ClockSource = RTC_WakeUpClockSPRE,
        .AutoReloadValue = 1
    };
    err = RTC_ConfigureWakeUpTimer(timerConfig);

    RTC_EnableInterrupts(RTC_IT_ALARM_A | RTC_IT_WKUP);
    RTC_IQREnable(RTC_IRQ_GROUP_ALARMS | RTC_IRQ_GROUP_WKUP, 1);

    while (1) {
        if (button_trigger) {
            // Small delay because of debouncing
            for (int i = 0; i < 1000000; i++);

            // Stop SysTick
            SYSTICK_CounterControl(DISABLE);

            // Enable alarm
            // RTC_AlarmControl(RTC_AlarmA, ENABLE);

            // Enable wake-up timer
            RTC_WakeUpTimerControl(ENABLE);

            PWR_EnterDeepSleepMode(&pwrHandle);

            // Start SysTick
            SYSTICK_CounterControl(ENABLE);

            button_trigger = 0;
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}

// Required for STOP mode
void EXTI17_RTC_Alarm_IRQHandler() {
    RTC_IRQHandling(&rtcHandle);
}

// Required for STOP mode
void EXTI22_RTC_WKUP_IRQHandler() {
    RTC_IRQHandling(&rtcHandle);
}
