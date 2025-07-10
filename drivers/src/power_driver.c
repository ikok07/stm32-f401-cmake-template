//
// Created by Kok on 7/8/25.
//

#include "stm32f4xx.h"

#include <commons.h>
#include <systick_driver.h>

#include "power_driver.h"

#include <rtc_driver.h>

/**
 * @brief Controls the power registers clock
 * @param Enabled If the power clock should be enabled
 */
void PWR_PeriClockControl(uint8_t Enabled) {
    if (Enabled) {
        RCC->APB1ENR |= (1 << RCC_APB1ENR_PWREN_Pos);
    } else {
        RCC->APB1ENR &=~ (1 << RCC_APB1ENR_PWREN_Pos);
    }
}

/**
 * @brief Configures the PWR control register
 * @param pwrHandle Power handle
 */
PWR_Error_e PWR_Init(PWR_Handle_t *pwrHandle) {

    if (!(PWR->CR & (1 << PWR_CR_DBP_Pos))) return PWR_ErrBackupRegLocked;

    // Regulator voltage scaling
    PWR->CR |= (pwrHandle->Config.RegulatorVoltageScaling << PWR_CR_VOS_Pos);

    // Main regulator low voltage
    PWR->CR |= (pwrHandle->Config.MainRegulatorLowVoltageEnabled << PWR_CR_MRLVDS_Pos);

    // Low power regulator low voltage
    PWR->CR |= (pwrHandle->Config.LowPowerRegulatorLowVoltageEnabled << PWR_CR_LPLVDS_Pos);

    // Flash power down in stop mode
    PWR->CR |= (pwrHandle->Config.FlashPowerDownInStopMode << PWR_CR_FPDS_Pos);

    // Programmable Voltage Detector
    PWR->CR |= (pwrHandle->Config.PVDEnabled << PWR_CR_PLS_Pos);

    // PVD Level
    PWR->CR |= (pwrHandle->Config.PVDThreshold << PWR_CR_PLS_Pos);

    // Standby mode
    PWR->CR |= (pwrHandle->Config.StandbyModeInDeepSleepEnabled << PWR_CR_PDDS_Pos);

    // Low power voltage regulator in stop mode
    PWR->CR |= (pwrHandle->Config.LowPowerRegulatorInStopMode << PWR_CR_LPDS_Pos);

    // Backup regulator
    PWR->CSR |= (pwrHandle->Config.BackupRegulatorEnabled << PWR_CSR_BRE_Pos);
    if (pwrHandle->Config.BackupRegulatorEnabled) {
        WAIT_WITH_TIMEOUT(!(PWR->CSR & (1 << PWR_CSR_BRR_Pos)), PWR_ErrTimeout, PWR_TIMEOUT_MS);
    }

    // Wake up pin
    PWR->CSR |= (pwrHandle->Config.WakeUpConfig.WKUPPinEnabled << PWR_CSR_EWUP_Pos);

    return PWR_ErrOK;
}

void PWR_DeInit() {
    PWR->CSR = 0x00;
    PWR->CR = 0x8000;
}

/**
 * Call this method in order to put the cpu into sleep mode. During sleep mode the periherals continue running
 * @param pwrHandle Power handle
 */
void PWR_EnterSleepMode(PWR_Handle_t *pwrHandle) {
    if (pwrHandle->Config.WakeUpConfig.Source == PWR_WakeUpSrcInterrupt) {
        __WFI();
    } else {
        // Clear previous events
        __SEV();
        __WFE();

        // Enter sleep mode
        __WFE();
    }
}

/**
 * @brief Depending on the StandbyModeInDeepSleepEnabled option this method sets the MCU in Stop or Standby mode
 * @param pwrHandle Power handle
 */
void PWR_EnterDeepSleepMode(PWR_Handle_t *pwrHandle) {
    PWR_ClearWakeUpFlags();

    // Enable deep sleep
    SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP_Pos);

    // Clear all EXTI pending bits
    EXTI->PR = 0xFFFFFFFF;

    // Clear RTC flags
    RTC_ClearFlag(RTC_FlagAlarmA);
    RTC_ClearFlag(RTC_FlagAlarmB);
    RTC_ClearFlag(RTC_FlagWKUPTimer);
    RTC_ClearFlag(RTC_FlagTamper);
    RTC_ClearFlag(RTC_FlagTimestamp);

    // Enter deep sleep
    if (pwrHandle->Config.WakeUpConfig.Source == PWR_WakeUpSrcInterrupt) {
        __WFI();
    } else {
        // Clear previous events
        __SEV();
        __WFE();

        __WFE();
    }

    // Disable deep sleep after wake up
    SCB->SCR &=~ (1 << SCB_SCR_SLEEPDEEP_Pos);
}

/**
 * @note After reset the backup registers are locked. Before writing to them the user should unlock them with this method.
 */
void PWR_UnlockBackupRegisters() {
    PWR->CR |= (1 << PWR_CR_DBP_Pos);
}

void PWR_LockBackupRegisters() {
    PWR->CR &=~ (1 << PWR_CR_DBP_Pos);
}

/**
 * @brief Clears all wake up flags. \b WARNING: This action should be called every time before entering low-power modes
 */
void PWR_ClearWakeUpFlags() {
    PWR->CR |= (1 << PWR_CR_CWUF_Pos);
    PWR->CR |= (1 << PWR_CR_CSBF_Pos);
}
