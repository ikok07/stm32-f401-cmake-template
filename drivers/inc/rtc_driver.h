//
// Created by Kok on 7/9/25.
//

#ifndef RTC_DRIVER_H
#define RTC_DRIVER_H

#include <stdint.h>

/* ------------ MACROS ------------ */

#define RTC_TIMEOUT_MS                  3000
#define RTC_MIN_HSE_PRESC               0x02
#define RTC_MAX_HSE_PRESC               0x1F        // 31

#define VALIDATE_DATE_VALUE(value, min, max, err)       if (value < min || value > max) return err;

/* ------------ ERROR CODES ------------ */

typedef enum {
    RTC_ErrOK,
    RTC_ErrTimeout,
    RTC_ErrBackupRegLocked,
    RTC_ErrRegistersLocked,
    RTC_ErrInvalidSeconds,
    RTC_ErrInvalidMinutes,
    RTC_ErrInvalidHours,
    RTC_ErrInvalidDay,
    RTC_ErrInvalidMonth,
    RTC_ErrInvalidWeekday,
    RTC_ErrInvalidYear,
    RTC_ErrInvalidHSEPresc,
    RTC_ErrInvalidWKUPAutoRealoadValue,
    RTC_ErrWKUPEnabled
} RTC_Error_e;

/* ------------ CONFIG STRUCTURES ------------ */

typedef enum {
    RTC_ClkLSE,
    RTC_ClkLSI,
    RTC_ClkHSE,
} RTC_ClkSource_e;

typedef enum {
    RTC_HourFormat24,
    RTC_HourFormatAMPM,
} RTC_HourFormat_e;

typedef enum {
    RTC_NotationAM_24H,
    RTC_NotationPM
} RTC_Notation_e;

typedef enum {
    RTC_Monday = 1,
    RTC_Tuesday,
    RTC_Wednesday,
    RTC_Thursday,
    RTC_Friday,
    RTC_Saturday,
    RTC_Sunday,
} RTC_Weekday_e;

typedef enum {
    RTC_AlarmA,
    RTC_AlarmB,
} RTC_Alarm_e;

typedef enum {
    RTC_SubsecondsMaskDisabled,     // No comparison on sub seconds
    RTC_SubsecondsMask0,
    RTC_SubsecondsMask1_0,
    RTC_SubsecondsMask2_0,
    RTC_SubsecondsMask3_0,
    RTC_SubsecondsMask4_0,
    RTC_SubsecondsMask5_0,
    RTC_SubsecondsMask6_0,
    RTC_SubsecondsMask7_0,
    RTC_SubsecondsMask8_0,
    RTC_SubsecondsMask9_0,
    RTC_SubsecondsMask10_0,
    RTC_SubsecondsMask11_0,
    RTC_SubsecondsMask12_0,
    RTC_SubsecondsMask13_0,
    RTC_SubsecondsMask14_0,
    RTC_SubsecondsMask15_0,
} RTC_SubSecondsMask_e;

typedef enum {
    RTC_WakeUpClockRTC16,
    RTC_WakeUpClockRTC8,
    RTC_WakeUpClockRTC4,
    RTC_WakeUpClockRTC2,
    RTC_WakeUpClockSPRE,                    // ck_spre (usually 1 Hz) clock is selected
    RTC_WakeUpClockSPREExtended,            // ck_spre (usually 1 Hz) clock is selected and 2^16 is added to the WUT counter value
} RTC_WakeUpClockSource_e;

typedef struct {
    uint8_t Year;       // Last two digits
    uint8_t Month;
    uint8_t Day;
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
    RTC_Notation_e Notation;
    RTC_Weekday_e Weekday;
} RTC_CalendarDate_t;

typedef struct {
    uint8_t AsyncPrescaler;
    uint16_t SyncPrescaler;
    RTC_ClkSource_e ClockSource;
    uint8_t HSEPrescaler;           // Valid only when HSE used as clock source
    RTC_HourFormat_e HourFormat;
    RTC_CalendarDate_t StartDate;
} RTC_Config_t;

typedef struct {
    RTC_Alarm_e AlarmX;
    RTC_Notation_e Notation;
    uint8_t SecondsDisabled;
    uint8_t Seconds;
    uint8_t MinutesDisabled;
    uint8_t Minutes;
    uint8_t HoursDisabled;
    uint8_t Hours;
    uint8_t DateDisabled;
    uint8_t DateIsWeekday;              // if Date represents the date units or the week day
    uint8_t Date;
    RTC_SubSecondsMask_e SubsSecondsMask;
    uint16_t SubSeconds;
} RTC_AlarmConfig_t;

typedef struct {
    RTC_WakeUpClockSource_e ClockSource;
    uint16_t AutoReloadValue;
} RTC_WakeUpTimerConfig_t;

typedef struct {
    RTC_Config_t Config;
} RTC_Handle_t;

/* ------------ METHODS ------------ */

/*
 * Write protection
 */
void RTC_DisableWriteProtection();

/*
 * Init and De-Init
 */
RTC_Error_e RTC_Init(RTC_Handle_t *rtcHandle);

/*
 * Reading Calendar
 */
RTC_Error_e RTC_ReadCalendar(RTC_CalendarDate_t *pCalendarDate);

/*
 * Alarm
 */
RTC_Error_e RTC_ConfigureAlarm(RTC_Handle_t *rtcHandle, RTC_AlarmConfig_t AlarmConfig);
void RTC_AlarmControl(RTC_Alarm_e Alarm, uint8_t Enabled);

/*
 * Wake-up timer
 */
RTC_Error_e RTC_ConfigureWakeUpTimer(RTC_WakeUpTimerConfig_t Config);
void RTC_WakeUpTimerControl(uint8_t Enabled);

void RTC_ConfigureDayLightSaving();

#endif //RTC_DRIVER_H
