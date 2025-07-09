//
// Created by Kok on 7/9/25.
//

#include "rtc_driver.h"

#include <commons.h>
#include <systick_driver.h>

#include "stm32f4xx.h"

uint8_t dec_to_bcd(uint8_t dec);

RTC_Error_e enter_init_mode();
void exit_init_mode();


/**
 * @brief After backup domain reset, all the RTC registers are write-protected. Writing to the RTC registers is enabled by disabling the protection with this method.
 */
void RTC_DisableWriteProtection() {
    // Disables the protection by writing specific keys
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
}

/**
 * @brief Initializes the RTC Peripheral
 * @param rtcHandle RTC handle
 */
RTC_Error_e RTC_Init(RTC_Handle_t *rtcHandle) {
    if (!(PWR->CR & (1 << PWR_CR_DBP_Pos))) return RTC_ErrBackupRegLocked;

    RTC_Error_e err = RTC_ErrOK;

    // Enter INIT mode
    if ((err = enter_init_mode()) != RTC_ErrOK) return err;

    // Prescalers
    RTC->PRER = rtcHandle->Config.AsyncPrescaler << RTC_PRER_PREDIV_A_Pos | rtcHandle->Config.SyncPrescaler;

    // Select clock source
    if (rtcHandle->Config.ClockSource == RTC_ClkLSE) {
        // Enable LSE
        RCC->BDCR |= (1 << RCC_BDCR_LSEON_Pos);
        WAIT_WITH_TIMEOUT(!(RCC->BDCR & (1 << RCC_BDCR_LSERDY_Pos)), RTC_ErrTimeout, RTC_TIMEOUT_MS);
    } else if (rtcHandle->Config.ClockSource == RTC_ClkHSE) {
        // Configure RTC HSE prescaler
        if (rtcHandle->Config.HSEPrescaler < RTC_MIN_HSE_PRESC || rtcHandle->Config.HSEPrescaler > RTC_MAX_HSE_PRESC) return RCC_ErrInvalidHSEPresc;
        RCC->CFGR |= (rtcHandle->Config.HSEPrescaler << RCC_CFGR_RTCPRE_Pos);
    }

    RCC->BDCR |= (rtcHandle->Config.ClockSource << RCC_BDCR_RTCSEL_Pos);

    VALIDATE_DATE_VALUE(rtcHandle->Config.StartDate.Seconds, 0, 60, RTC_ErrInvalidSeconds);
    VALIDATE_DATE_VALUE(rtcHandle->Config.StartDate.Minutes, 0, 60, RTC_ErrInvalidMinutes);
    VALIDATE_DATE_VALUE(rtcHandle->Config.StartDate.Hours, 0, rtcHandle->Config.HourFormat == RTC_HourFormat24 ? 24 : 12, RTC_ErrInvalidHours);
    VALIDATE_DATE_VALUE(rtcHandle->Config.StartDate.Day, 0, 31, RTC_ErrInvalidDay);
    VALIDATE_DATE_VALUE(rtcHandle->Config.StartDate.Month, 0, 12, RTC_ErrInvalidMonth);
    VALIDATE_DATE_VALUE(rtcHandle->Config.StartDate.Weekday, RTC_Monday, RTC_Sunday, RTC_ErrInvalidWeekday);
    VALIDATE_DATE_VALUE(rtcHandle->Config.StartDate.Year, 0x00, 99, RTC_ErrInvalidYear);

    uint8_t secondsBCD = dec_to_bcd(rtcHandle->Config.StartDate.Seconds);
    uint8_t minutesBCD = dec_to_bcd(rtcHandle->Config.StartDate.Minutes);
    uint8_t hoursBCD = dec_to_bcd(rtcHandle->Config.StartDate.Hours);
    RTC->TR = secondsBCD | (minutesBCD << RTC_TR_MNU_Pos) | (hoursBCD << RTC_TR_HU_Pos) | (rtcHandle->Config.StartDate.Notation << RTC_TR_PM_Pos);

    uint8_t dayBCD = dec_to_bcd(rtcHandle->Config.StartDate.Day);
    uint8_t monthBCD = dec_to_bcd(rtcHandle->Config.StartDate.Month);
    uint8_t weekday = rtcHandle->Config.StartDate.Weekday;
    uint8_t yearBCD = dec_to_bcd(rtcHandle->Config.StartDate.Year);
    RTC->DR = dayBCD | (monthBCD << RTC_DR_MU_Pos) | (weekday << RTC_DR_WDU_Pos) | (yearBCD << RTC_DR_YU_Pos);

    // Hour format
    RTC->CR |= (rtcHandle->Config.HourFormat << RTC_CR_FMT_Pos);

    // Exit INIT mode
    exit_init_mode();

    return RTC_ErrOK;
}


/**
 * @note \b WARNING: Works only with two digit decimals
 * @param dec Two digit decimal
 */
uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | dec % 10;
}

/**
 * @note \b WARNING: Works only with two digit binary coded decimals
 * @param bcd Binary coded decimal
 */
uint8_t bdc_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0xF);
}

RTC_Error_e enter_init_mode() {
    RTC->ISR |= (1 << RTC_ISR_INIT_Pos);
    WAIT_WITH_TIMEOUT(!(RTC->ISR & (1 << RTC_ISR_INITF_Pos)), RTC_ErrTimeout, RTC_TIMEOUT_MS);
    return RTC_ErrOK;
}

void exit_init_mode() {
    RTC->ISR &=~ (1 << RTC_ISR_INIT_Pos);
}