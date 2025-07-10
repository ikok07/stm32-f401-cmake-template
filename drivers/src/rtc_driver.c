//
// Created by Kok on 7/9/25.
//

#include "stm32f4xx.h"

#include <commons.h>
#include <clock_driver.h>
#include <systick_driver.h>

#include "rtc_driver.h"

static uint8_t dec_to_bcd(uint8_t dec);
static uint8_t bcd_to_dec(uint8_t dec);

static uint32_t get_rtclk_hz(RTC_Handle_t *pRTCHandle);

static RTC_Error_e enter_init_mode();
static void exit_init_mode();

/**
 * @brief Enables the write protection to the RTC registers
 */
void RTC_EnableWriteProtection() {
    RTC->WPR = 0x00;
}

/**
 * @brief After backup domain reset, all the RTC registers are write-protected. Writing to the RTC registers is enabled by disabling the protection with this method.
 */
void RTC_DisableWriteProtection() {
    // Disables the protection by writing specific keys
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
}

/**
 * @brief Enables or disables the RTC clock
 * @param Enabled If the RTC clock should be enabled
 */
void RTC_ClockControl(uint8_t Enabled) {
    if (Enabled) {
        RCC->BDCR |= (1 << RCC_BDCR_RTCEN_Pos);
    } else {
        RCC->BDCR &=~ (1 << RCC_BDCR_RTCEN_Pos);
    }
}

/**
 * @returns TRUE if the RTC is already initialized
 */
uint8_t RTC_IsCalendarInitialized() {
    return (RTC->ISR & (1 << RTC_ISR_INITS_Pos)) > 0;
}

/**
 * @brief Initializes the RTC Peripheral
 * @param rtcHandle RTC handle
 */
RTC_Error_e RTC_Init(RTC_Handle_t *pRTCHandle) {
    if (!(PWR->CR & (1 << PWR_CR_DBP_Pos))) return RTC_ErrBackupRegLocked;
    if (!(RCC->BDCR & (1 << RCC_BDCR_RTCEN_Pos))) return RTC_ErrDisabled;

    RTC_Error_e err = RTC_ErrOK;

    // Select clock source
    if (pRTCHandle->Config.ClockSource == RTC_ClkLSE) {
        // Enable LSE
        RCC->BDCR |= (1 << RCC_BDCR_LSEON_Pos);
        WAIT_WITH_TIMEOUT(!(RCC->BDCR & (1 << RCC_BDCR_LSERDY_Pos)), RTC_ErrTimeout, RTC_TIMEOUT_MS);
    } else if (pRTCHandle->Config.ClockSource == RTC_ClkLSI) {
        // Enable LSI
        RCC->CSR |= (1 << RCC_CSR_LSION_Pos);
        WAIT_WITH_TIMEOUT(!(RCC->CSR & (1 << RCC_CSR_LSIRDY_Pos)), RTC_ErrTimeout, RTC_TIMEOUT_MS);
    } else if (pRTCHandle->Config.ClockSource == RTC_ClkHSE) {
        // Configure RTC HSE prescaler
        if (pRTCHandle->Config.HSEPrescaler < RTC_MIN_HSE_PRESC || pRTCHandle->Config.HSEPrescaler > RTC_MAX_HSE_PRESC) return RTC_ErrInvalidHSEPresc;
        RCC->CFGR |= (pRTCHandle->Config.HSEPrescaler << RCC_CFGR_RTCPRE_Pos);
    }

    RCC->BDCR &=~ (0x03 << RCC_BDCR_RTCSEL_Pos);
    RCC->BDCR |= (pRTCHandle->Config.ClockSource << RCC_BDCR_RTCSEL_Pos);

    RTC_DisableWriteProtection();

    // Enter INIT mode
    if ((err = enter_init_mode()) != RTC_ErrOK) return err;

    // Prescalers
    RTC->PRER = pRTCHandle->Config.AsyncPrescaler << RTC_PRER_PREDIV_A_Pos | pRTCHandle->Config.SyncPrescaler;

    VALIDATE_DATE_VALUE(pRTCHandle->Config.StartDate.Seconds, 0, 59, RTC_ErrInvalidSeconds);
    VALIDATE_DATE_VALUE(pRTCHandle->Config.StartDate.Minutes, 0, 59, RTC_ErrInvalidMinutes);
    VALIDATE_DATE_VALUE(pRTCHandle->Config.StartDate.Hours, 0, (pRTCHandle->Config.HourFormat == RTC_HourFormat24 ? 24 : 12), RTC_ErrInvalidHours);
    VALIDATE_DATE_VALUE(pRTCHandle->Config.StartDate.Day, 0, 31, RTC_ErrInvalidDay);
    VALIDATE_DATE_VALUE(pRTCHandle->Config.StartDate.Month, 0, 12, RTC_ErrInvalidMonth);
    VALIDATE_DATE_VALUE(pRTCHandle->Config.StartDate.Weekday, RTC_Monday, RTC_Sunday, RTC_ErrInvalidWeekday);
    VALIDATE_DATE_VALUE(pRTCHandle->Config.StartDate.Year, 0x00, 99, RTC_ErrInvalidYear);

    uint8_t secondsBCD = dec_to_bcd(pRTCHandle->Config.StartDate.Seconds);
    uint8_t minutesBCD = dec_to_bcd(pRTCHandle->Config.StartDate.Minutes);
    uint8_t hoursBCD = dec_to_bcd(pRTCHandle->Config.StartDate.Hours);
    RTC->TR = secondsBCD | (minutesBCD << RTC_TR_MNU_Pos) | (hoursBCD << RTC_TR_HU_Pos) | (pRTCHandle->Config.StartDate.Notation << RTC_TR_PM_Pos);

    uint8_t dayBCD = dec_to_bcd(pRTCHandle->Config.StartDate.Day);
    uint8_t monthBCD = dec_to_bcd(pRTCHandle->Config.StartDate.Month);
    uint8_t weekday = pRTCHandle->Config.StartDate.Weekday;
    uint8_t yearBCD = dec_to_bcd(pRTCHandle->Config.StartDate.Year);
    RTC->DR = dayBCD | (monthBCD << RTC_DR_MU_Pos) | (weekday << RTC_DR_WDU_Pos) | (yearBCD << RTC_DR_YU_Pos);

    // Hour format
    RTC->CR |= (pRTCHandle->Config.HourFormat << RTC_CR_FMT_Pos);

    // Exit INIT mode
    exit_init_mode();

    return RTC_ErrOK;
}

RTC_Error_e RTC_ReadCalendar(RTC_Handle_t *pRTCHandle, RTC_CalendarDate_t *pCalendarDate) {
    uint32_t pclk1 = CLOCK_GetApb1Hz();
    uint32_t rtcclk = get_rtclk_hz(pRTCHandle);
    if (pclk1 < rtcclk * 7) return RTC_ErrInvalidClocks;

    // Wait for latest calendar data
    WAIT_WITH_TIMEOUT(!(RTC->ISR & (1 << RTC_ISR_RSF_Pos)), RTC_ErrTimeout, RTC_TIMEOUT_MS);

    // Read calendar registers
    uint32_t tr = RTC->TR;
    uint16_t ssr = RTC->SSR;
    uint32_t dr = RTC->DR;

    pCalendarDate->Subseconds = ((pRTCHandle->Config.SyncPrescaler - ssr) * 1000) / (pRTCHandle->Config.SyncPrescaler + 1);
    pCalendarDate->Seconds = bcd_to_dec(tr & 0x7F);
    pCalendarDate->Minutes = bcd_to_dec((tr >> RTC_TR_MNU_Pos) & 0x7F);
    pCalendarDate->Hours = bcd_to_dec((tr >> RTC_TR_HU_Pos) & 0x3F);
    pCalendarDate->Notation = (tr >> RTC_TR_PM_Pos) & 0x01;

    pCalendarDate->Day = bcd_to_dec(dr & 0x3F);
    pCalendarDate->Weekday = (dr >> RTC_DR_WDU_Pos) & 0x07;
    pCalendarDate->Month = bcd_to_dec((dr >> RTC_DR_MU_Pos) & 0x1F);
    pCalendarDate->Year = bcd_to_dec((dr >> RTC_DR_YU_Pos) & 0xFF);

    return RTC_ErrOK;
}

/**
 * @brief Configures the desired alarm
 * @param AlarmConfig Configuration for the desired alarm
 */
RTC_Error_e RTC_ConfigureAlarm(RTC_Handle_t *pRTCHandle, RTC_AlarmConfig_t AlarmConfig) {
    uint32_t alarmRegister = 0x00;
    uint32_t alarmSubSecondRegister = 0x00;

    // Set date, hour, minutes, seconds and subseconds
    if (!AlarmConfig.SecondsDisabled) {
        VALIDATE_DATE_VALUE(AlarmConfig.Seconds, 0, 60, RTC_ErrInvalidSeconds);
        alarmRegister = dec_to_bcd(AlarmConfig.Seconds);
    } else alarmRegister |= (1 << RTC_ALRMAR_MSK1_Pos);

    if (!AlarmConfig.MinutesDisabled) {
        VALIDATE_DATE_VALUE(AlarmConfig.Minutes, 0, 60, RTC_ErrInvalidMinutes);
        alarmRegister |= (dec_to_bcd(AlarmConfig.Minutes) << RTC_ALRMAR_MNU_Pos);
    } else alarmRegister |= (1 << RTC_ALRMAR_MSK2_Pos);

    if (!AlarmConfig.HoursDisabled) {
        VALIDATE_DATE_VALUE(AlarmConfig.Hours, 0, (pRTCHandle->Config.HourFormat == RTC_HourFormat24 ? 24 : 12), RTC_ErrInvalidHours);
        alarmRegister |= (dec_to_bcd(AlarmConfig.Hours) << RTC_ALRMAR_HU_Pos);
    } else alarmRegister |= (1 << RTC_ALRMAR_MSK3_Pos);

    if (!AlarmConfig.DateDisabled) {
        if (AlarmConfig.DateIsWeekday) {
            VALIDATE_DATE_VALUE(AlarmConfig.Date, RTC_Monday, RTC_Sunday, RTC_ErrInvalidWeekday);
        } else {
            VALIDATE_DATE_VALUE(AlarmConfig.Date, 0, 31, RTC_ErrInvalidDay);
        };

        alarmRegister |= (dec_to_bcd(AlarmConfig.DateIsWeekday) << RTC_ALRMAR_WDSEL_Pos);
        alarmRegister |= (dec_to_bcd(AlarmConfig.Date) << RTC_ALRMAR_DU_Pos);
    } else {
        alarmRegister |= (1 << RTC_ALRMAR_MSK4_Pos);
    }

    if (AlarmConfig.SubsSecondsMask != RTC_SubsecondsMaskDisabled) {
        alarmSubSecondRegister = (AlarmConfig.SubSeconds & 0x3FFF) | (AlarmConfig.SubsSecondsMask << RTC_ALRMASSR_MASKSS_Pos);
    }

    // Set hour notation
    alarmRegister |= (AlarmConfig.Notation << RTC_ALRMAR_PM_Pos);

    // Write to registers
    if (AlarmConfig.AlarmX == RTC_AlarmA) {
        RTC->ALRMAR = alarmRegister;
        RTC->ALRMASSR = alarmSubSecondRegister;
    }
    else {
        RTC->ALRMBR = alarmRegister;
        RTC->ALRMBSSR = alarmSubSecondRegister;
    }

    return RTC_ErrOK;
}


/**
 * @brief Enables and disables the desired alarm
 * @param Alarm Desired alarm
 * @param Enabled If the alarm should be enabled
 */
void RTC_AlarmControl(RTC_Alarm_e Alarm, uint8_t Enabled) {
    if (Enabled) {
        if (Alarm == RTC_AlarmA) RTC->CR |= (1 << RTC_CR_ALRAE_Pos);
        else RTC->CR |= (1 << RTC_CR_ALRBE_Pos);
    } else {
        if (Alarm == RTC_AlarmA) RTC->CR &=~ (1 << RTC_CR_ALRAE_Pos);
        else RTC->CR &=~ (1 << RTC_CR_ALRBE_Pos);
    }
}

RTC_Error_e RTC_ConfigureWakeUpTimer(RTC_WakeUpTimerConfig_t Config) {
    if (Config.AutoReloadValue < 0x00 || Config.AutoReloadValue > 0xFFFF) return RTC_ErrInvalidWKUPAutoRealoadValue;
    if (RTC->CR & (1 << RTC_CR_WUTE_Pos)) return RTC_ErrWKUPEnabled;

    // Wake-Up timer clock source
    RTC->CR |= (Config.ClockSource << RTC_CR_WUCKSEL_Pos);

    // Auto-reload value
    RTC->WUTR = Config.AutoReloadValue;

    return RTC_ErrOK;
}

/**
 * @brief Enables or disables the wake-up timer
 * @param Enabled If the wake-up timer should be enabled
 */
void RTC_WakeUpTimerControl(uint8_t Enabled) {
    if (Enabled) {
        RTC->CR |= (1 << RTC_CR_WUTE_Pos);
    } else {
        RTC->CR &=~ (1 << RTC_CR_WUTE_Pos);
    }
}

/**
 * @brief Configures the desired alarm output
 * @param pRTCHandle RTC Handle
 * @param Output Desired alarm output
 */
void RTC_SelectAlarmOutput(RTC_Handle_t *pRTCHandle, RTC_AlarmOutput_e Output, uint8_t PinLowOnTrigger, uint8_t PinOutputPushPull) {
    RTC->CR |= (Output << RTC_CR_OSEL_Pos);
    RTC->CR |= (PinLowOnTrigger << RTC_CR_POL_Pos);
    RTC->TAFCR |= (PinOutputPushPull << RTC_TAFCR_ALARMOUTTYPE_Pos);
    pRTCHandle->AlarmOutput = Output;
}

/**
 * @brief Maps specific RTC functionality to AF1
 * @param pRTCHandle RTC handle
 * @param Mapping The desired functionality to which the AF1 should be mapped
 */
RTC_Error_e RTC_MapAdditionalFunction(RTC_Handle_t *pRTCHandle, RTC_AFMapping_e Mapping) {
    if (pRTCHandle->AlarmOutput != RTC_AlarmOutputDisabled) return RTC_ErrAFPinInOutputMode;

    if (Mapping == RTC_AFTimestamp) {
        // Disable tamper mapping
        RTC->TAFCR &=~ (1 << RTC_TAFCR_TAMP1INSEL_Pos);

        // Enable timestamp mapping
        RTC->TAFCR |= (1 << RTC_TAFCR_TSINSEL_Pos);
    } else if (Mapping == RTC_AFTamper) {
        // Disable timestamp mapping
        RTC->TAFCR &=~ (1 << RTC_TAFCR_TSINSEL_Pos);

        // Enable tamper mapping
        RTC->TAFCR |= (1 << RTC_TAFCR_TAMP1INSEL_Pos);
    }

    pRTCHandle->AFMapping = Mapping;

    return RTC_ErrOK;
}

/**
 * @brief Reads the current timestamp data. \b WARNING: The timestamp flag should be set before calling this method!
 * @param pRTCHandle RTC Handle
 * @param pCalendarDate The structure where the timestamp data will be stored
 */
RTC_Error_e RTC_ReadTimestamp(RTC_Handle_t *pRTCHandle, RTC_CalendarDate_t *pCalendarDate) {
    if (!(RTC->ISR & (1 << RTC_ISR_TSF_Pos))) return RTC_ErrTimestampNotAvailable;

    // Read timestamp registers
    uint32_t tstr = RTC->TSTR;
    uint16_t tsssr = RTC->TSSSR;
    uint32_t tsdr = RTC->TSDR;

    pCalendarDate->Subseconds = ((pRTCHandle->Config.SyncPrescaler - tsssr) * 1000) / (pRTCHandle->Config.SyncPrescaler + 1);
    pCalendarDate->Seconds = bcd_to_dec(tstr & 0x7F);
    pCalendarDate->Minutes = bcd_to_dec((tstr >> RTC_TSTR_MNU_Pos) & 0x7F);
    pCalendarDate->Hours = bcd_to_dec((tstr >> RTC_TSTR_HU_Pos) & 0x3F);
    pCalendarDate->Notation = (tstr >> RTC_TSTR_PM_Pos) & 0x01;

    pCalendarDate->Day = bcd_to_dec(tsdr & 0x3F);
    pCalendarDate->Weekday = (tsdr >> RTC_TSDR_WDU_Pos) & 0x07;
    pCalendarDate->Month = bcd_to_dec((tsdr >> RTC_TSDR_MU_Pos) & 0x1F);

    // Clear TSF
    RTC_ClearFlag(RTC_FlagTimestamp);

    return RTC_ErrOK;
}

/**
 * @brief Enables or disables the RTC timestamp functionality
 * @param Enabled If the timestamp should be enabled or disabled
 */
void RTC_TimestampControl(uint8_t Enabled) {
    if (Enabled) {
        RTC->CR |= (1 << RTC_CR_TSE_Pos);
    } else {
        RTC->CR &=~ (1 << RTC_CR_TSE_Pos);
    }
}

/**
 * @brief Configures the RTC's tamper detection
 * @param Config Tamper config
 */
void RTC_ConfigureTamperDetection(RTC_TamperDetectionConfig_t Config) {
    // Timestamp on tamper detection
    RTC->TAFCR |= (Config.TimestampOnDetectionEnabled << RTC_TAFCR_TAMPTS_Pos);

    // Filter count
    RTC->TAFCR |= (Config.FilterCount << RTC_TAFCR_TAMPFLT_Pos);

    // Frequency mode
    RTC->TAFCR |= (Config.FrequencyMode << RTC_TAFCR_TAMPFREQ_Pos);

    // Pre charge cycles
    RTC->TAFCR |= (Config.PreChargeCycles << RTC_TAFCR_TAMPPRCH_Pos);

    // Tamper trigger level
    RTC->TAFCR |= (Config.TamperTriggerWhenLOW << RTC_TAFCR_TAMP1TRG_Pos);

    // Pull up
    RTC->TAFCR |= (Config.PullUpDisabled << RTC_TAFCR_TAMPPUDIS_Pos);
}

/**
 * @brief Enables or disables the RTC tamper detection
 * @param Enabled If the tamper detection should be enabled or disabled
 */
void RTC_TamperDetectionControl(uint8_t Enabled) {
    if (Enabled) {
        RTC->TAFCR |= (1 << RTC_TAFCR_TAMP1E_Pos);
    } else {
        RTC->TAFCR &=~ (1 << RTC_TAFCR_TAMP1E_Pos);
    }
}

/**
 * @brief Clears the desired RTC Flag
 * @param Flag RTC Flag
 */
void RTC_ClearFlag(RTC_Flag_e Flag) {
    switch (Flag) {
        case RTC_FlagAlarmA:
            RTC->ISR &=~ (1 << RTC_ISR_ALRAF_Pos);
        break;
        case RTC_FlagAlarmB:
            RTC->ISR &=~ (1 << RTC_ISR_ALRBF_Pos);
        break;
        case RTC_FlagWKUPTimer:
            RTC->ISR &=~ (1 << RTC_ISR_WUTF_Pos);
        break;
        case RTC_FlagTimestamp:
            RTC->ISR &=~ (1 << RTC_ISR_TSF_Pos);
        break;
        case RTC_FlagTimestampOverflow:
            RTC->ISR &=~ (1 << RTC_ISR_TSOVF_Pos);
        break;
        case RTC_FlagTamper:
            RTC->ISR &=~ (1 << RTC_ISR_TAMP1F_Pos);
        break;
    }
}

/**
 * @brief Enables the desired RTC interrupts
 * @param InterruptsMask Mask selecting the desired RTC interrupts
 */
void RTC_EnableInterrupts(uint8_t InterruptsMask) {
    /* ***** Alarm A and B ***** */
    if (InterruptsMask & (1 << RTC_IT_ALARM_A_POS) || InterruptsMask & (1 << RTC_IT_ALARM_B_POS)) {
        // Enable EXTI 17
        EXTI->IMR |= (1 << EXTI_IMR_MR17_Pos);
        EXTI->RTSR |= (1 << EXTI_RTSR_TR17_Pos);
    }

    if (InterruptsMask & (1 << RTC_IT_ALARM_A_POS)) RTC->CR |= (1 << RTC_CR_ALRAIE_Pos);
    if (InterruptsMask & (1 << RTC_IT_ALARM_B_POS)) RTC->CR |= (1 << RTC_CR_ALRBIE_Pos);

    /* ***** Wake-up ***** */
    if (InterruptsMask & (1 << RTC_IT_WKUP_POS)) {
        // Enable EXTI 22
        EXTI->IMR |= (1 << EXTI_IMR_MR22_Pos);
        EXTI->RTSR |= (1 << EXTI_RTSR_TR22_Pos);

        RTC->CR |= (1 << RTC_CR_WUTIE_Pos);
    }

    /* ***** Tamper detection and timestamp ***** */
    if (InterruptsMask & (1 << RTC_IT_TAMPER_POS) || InterruptsMask & (1 << RTC_IT_TIMESTAMP_POS)) {
        // Enable EXTI 21
        EXTI->IMR |= (1 << EXTI_IMR_MR21_Pos);
        EXTI->RTSR |= (1 << EXTI_RTSR_TR21_Pos);
    }

    if (InterruptsMask & (1 << RTC_IT_TAMPER_POS)) RTC->TAFCR |= (1 << RTC_TAFCR_TAMP1E_Pos);
    if (InterruptsMask & (1 << RTC_IT_TIMESTAMP_POS)) RTC->CR |= (1 << RTC_CR_TSIE_Pos);
}

/**
 * @brief Disables the desired RTC interrupts
 * @param InterruptsMask Mask selecting the desired RTC interrupts
 */
void RTC_DisableInterrupts(uint8_t InterruptsMask) {
    /* ***** Alarm A and B ***** */
    if (InterruptsMask & (1 << RTC_IT_ALARM_A_POS)) RTC->CR &=~ (1 << RTC_CR_ALRAIE_Pos);
    if (InterruptsMask & (1 << RTC_IT_ALARM_B_POS)) RTC->CR &=~ (1 << RTC_CR_ALRBIE_Pos);

    if (!(InterruptsMask & (1 << RTC_IT_ALARM_A_POS)) && !(InterruptsMask & (1 << RTC_IT_ALARM_B_POS))) {
        // Disable EXTI 17
        EXTI->IMR &=~ (1 << EXTI_IMR_MR17_Pos);
    }

    /* ***** Wake-up ***** */
    if (InterruptsMask & (1 << RTC_IT_WKUP_POS)) {
        // Disable EXTI22
        EXTI->IMR &=~ (1 << EXTI_IMR_MR22_Pos);

        RTC->CR &=~ (1 << RTC_CR_WUTIE_Pos);
    }

    /* ***** Tamper detection and timestamp ***** */
    if (InterruptsMask & (1 << RTC_IT_TAMPER_POS)) RTC->TAFCR &=~ (1 << RTC_TAFCR_TAMP1E_Pos);
    if (InterruptsMask & (1 << RTC_IT_TIMESTAMP_POS)) RTC->CR &=~ (1 << RTC_CR_TSIE_Pos);

    if (!(InterruptsMask & (1 << RTC_IT_TAMPER_POS)) && !(InterruptsMask & (1 << RTC_IT_TIMESTAMP_POS))) {
        // Disable EXTI 21
        EXTI->IMR &=~ (1 << EXTI_IMR_MR21_Pos);
    }
}

/**
 * @brief Enables the desired IRQ numbers with the desired priority level
 * @param IRQGroupsMask Masks selecting the desired IRQ number groups
 * @param Priority The desired IRQ priority
 */
void RTC_IQREnable(uint8_t IRQGroupsMask, uint8_t Priority) {
    if (IRQGroupsMask & (1 << RTC_IRQ_GROUP_ALARMS_POS)) {
        NVIC_EnableIRQ(RTC_Alarm_IRQn);
        NVIC_SetPriority(RTC_Alarm_IRQn, Priority);
    }

    if (IRQGroupsMask & (1 << RTC_IRQ_GROUP_WKUP_POS)) {
        NVIC_EnableIRQ(RTC_WKUP_IRQn);
        NVIC_SetPriority(RTC_WKUP_IRQn, Priority);
    }

    if (IRQGroupsMask & (1 << RTC_IRQ_GROUP_TS_TAMP_POS)) {
        NVIC_EnableIRQ(TAMP_STAMP_IRQn);
        NVIC_SetPriority(TAMP_STAMP_IRQn, Priority);
    }
}

/**
 * @brief Disables the desired IRQ numbers
 * @param IRQGroupsMask Masks selecting the desired IRQ number groups
 */
void RTC_IQRDisable(uint8_t IRQGroupsMask) {
    if (IRQGroupsMask & (1 << RTC_IRQ_GROUP_ALARMS)) NVIC_EnableIRQ(RTC_Alarm_IRQn);
    else if (IRQGroupsMask & (1 << RTC_IRQ_GROUP_WKUP)) NVIC_DisableIRQ(RTC_WKUP_IRQn);
    else if (IRQGroupsMask & (1 << RTC_IRQ_GROUP_TS_TAMP)) NVIC_DisableIRQ(TAMP_STAMP_IRQn);
}

/**
 * @brief Handles the RTC interrupts and calls the RTC_ApplicationEventCallback()
 * @param pRTCHandle RTC handle
 */
void RTC_IRQHandling(RTC_Handle_t *pRTCHandle) {
    if (RTC->CR & (1 << RTC_CR_ALRAIE_Pos) && RTC->ISR & (1 << RTC_ISR_ALRAF_Pos)) {
        // Alarm A
        RTC->ISR &=~ (1 << RTC_ISR_ALRAF_Pos);
        EXTI->PR |= (1 << EXTI_PR_PR17_Pos);
        RTC_ApplicationEventCallback(pRTCHandle, RTC_EventAlarmA);
    }

    if (RTC->CR & (1 << RTC_CR_ALRBIE_Pos) && RTC->ISR & (1 << RTC_ISR_ALRBF_Pos)) {
        // Alarm B
        RTC->ISR &=~ (1 << RTC_ISR_ALRBF_Pos);
        EXTI->PR |= (1 << EXTI_PR_PR17_Pos);

        RTC_ApplicationEventCallback(pRTCHandle, RTC_EventAlarmB);
    }

    if (RTC->CR & (1 << RTC_CR_WUTIE_Pos) && RTC->ISR & (1 << RTC_ISR_WUTF_Pos)) {
        // Wake-up
        RTC->ISR &=~ (1 << RTC_ISR_WUTF_Pos);
        EXTI->PR |= (1 << EXTI_PR_PR22_Pos);
        RTC_ApplicationEventCallback(pRTCHandle, RTC_EventWakeUp);
    }

    if (RTC->CR & (1 << RTC_CR_TSIE_Pos) && RTC->ISR & (1 << RTC_ISR_TSF_Pos)) {
        // Timestamp
        RTC->ISR &=~ (1 << RTC_ISR_TSF_Pos);
        EXTI->PR |= (1 << EXTI_PR_PR21_Pos);
        RTC_ApplicationEventCallback(pRTCHandle, RTC_EventTimestamp);
    }

    if (RTC->TAFCR & (1 << RTC_TAFCR_TAMPIE_Pos) && RTC->ISR & (1 << RTC_ISR_TAMP1F_Pos)) {
        // Tamper 1
        RTC->ISR &=~ (1 << RTC_ISR_TAMP1F_Pos);
        EXTI->PR |= (1 << EXTI_PR_PR21_Pos);
        RTC_ApplicationEventCallback(pRTCHandle, RTC_EventTamperDetection);
    }
}

/**
 * @brief Configures the RTC for winter or summer time
 */
RTC_Error_e RTC_ConfigureDayLightSaving(RTC_Handle_t *pRTCHandle, RTC_DaylightSavingConfig_t Config) {
    // TODO: Read the rtc and decide whether one hour should be added or subtracted
    RTC_Error_e err;
    RTC_CalendarDate_t calendarDate;
    if ((err = RTC_ReadCalendar(pRTCHandle, &calendarDate)) != RTC_ErrOK) return err;

    uint8_t calendarWeek = (calendarDate.Day / 7) + 1;
    if (calendarDate.Month >= Config.SummerTimeMonth && calendarDate.Month < Config.WinterTimeMonth) {
        if (calendarDate.Month != Config.SummerTimeMonth || (calendarWeek == Config.SummerTimeWeek && calendarDate.Weekday == RTC_Sunday)) {
            // Summer
            if (!(RTC->CR & (1 << RTC_CR_BKP_Pos))) {
                RTC->CR |= (1 << RTC_CR_ADD1H_Pos);
                RTC->CR |= (1 << RTC_CR_BKP_Pos);
            }
        }
    } else {
        if (calendarDate.Month != Config.WinterTimeMonth || (calendarWeek == Config.WinterTimeWeek && calendarDate.Weekday == RTC_Sunday)) {
            // Winter
            if (RTC->CR & (1 << RTC_CR_BKP_Pos)) {
                RTC->CR |= (1 << RTC_CR_SUB1H_Pos);
                RTC->CR &=~ (1 << RTC_CR_BKP_Pos);
            }
        }
    }

    return RTC_ErrOK;
}

__weak void RTC_ApplicationEventCallback(RTC_Handle_t *pRTCHandle, RTC_Event_e AppEvent) {};


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
uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0xF);
}

uint32_t get_rtclk_hz(RTC_Handle_t *pRTCHandle) {
    switch (pRTCHandle->Config.ClockSource) {
        case RTC_ClkLSI:
            return 32000;
        case RTC_ClkLSE:
            return 32768;
        case RTC_ClkHSE:
            return HSE_VALUE;
        default:
            return 32000;
    }
}

RTC_Error_e enter_init_mode() {
    RTC->ISR |= (1 << RTC_ISR_INIT_Pos);
    WAIT_WITH_TIMEOUT(!(RTC->ISR & (1 << RTC_ISR_INITF_Pos)), RTC_ErrTimeout, RTC_TIMEOUT_MS);
    return RTC_ErrOK;
}

void exit_init_mode() {
    RTC->ISR &=~ (1 << RTC_ISR_INIT_Pos);
}