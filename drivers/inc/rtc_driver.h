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
    RCC_ErrInvalidHSEPresc
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

typedef struct {
    uint8_t Year;       // Last two digits
    uint8_t Month;
    uint8_t Day;
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
    RTC_Notation_e Notation;
    RTC_Weekday_e Weekday;
} RTC_DateConfig_t;

typedef struct {
    uint8_t AsyncPrescaler;
    uint16_t SyncPrescaler;
    RTC_ClkSource_e ClockSource;
    uint8_t HSEPrescaler;           // Valid only when HSE used as clock source
    RTC_HourFormat_e HourFormat;
    RTC_DateConfig_t StartDate;
} RTC_Config_t;

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

#endif //RTC_DRIVER_H
