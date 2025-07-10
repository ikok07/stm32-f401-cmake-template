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

#define RTC_IT_ALARM_A                  0b00001
#define RTC_IT_ALARM_B                  0b00010
#define RTC_IT_WKUP                     0b00100
#define RTC_IT_TIMESTAMP                0b01000
#define RTC_IT_TAMPER                   0b10000

#define RTC_IT_ALARM_A_POS              0x00
#define RTC_IT_ALARM_B_POS              0x01
#define RTC_IT_WKUP_POS                 0x02
#define RTC_IT_TIMESTAMP_POS            0x03
#define RTC_IT_TAMPER_POS               0x04

#define RTC_IRQ_GROUP_ALARMS            0b001
#define RTC_IRQ_GROUP_WKUP              0b010
#define RTC_IRQ_GROUP_TS_TAMP           0b100

#define RTC_IRQ_GROUP_ALARMS_POS        0x00
#define RTC_IRQ_GROUP_WKUP_POS          0x01
#define RTC_IRQ_GROUP_TS_TAMP_POS            0x02

/* ------------ ERROR CODES ------------ */

typedef enum {
    RTC_ErrOK,
    RTC_ErrTimeout,
    RTC_ErrDisabled,
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
    RTC_ErrWKUPEnabled,
    RTC_ErrInvalidClocks,                // To read the RTC calendar properly, the APB1 clock frequency must be equal to or greater than seven times the RTCCLK,
    RTC_ErrTimestampNotAvailable,
    RTC_ErrAFPinInOutputMode
} RTC_Error_e;

/* ------------ CONFIG STRUCTURES ------------ */

typedef enum {
    RTC_ClkLSE = 1,
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

typedef enum {
    RTC_FlagAlarmA,
    RTC_FlagAlarmB,
    RTC_FlagWKUPTimer,
    RTC_FlagTimestamp,
    RTC_FlagTimestampOverflow,
    RTC_FlagTamper
} RTC_Flag_e;

typedef enum {
    RTC_EventAlarmA,
    RTC_EventAlarmB,
    RTC_EventWakeUp,
    RTC_EventTimestamp,
    RTC_EventTamperDetection
} RTC_Event_e;

typedef enum {
    RTC_AFTimestamp,
    RTC_AFTamper,
} RTC_AFMapping_e;

typedef enum {
    RTC_AlarmOutputDisabled,
    RTC_AlarmOutputA,
    RTC_AlarmOutputB,
    RTC_AlarmOutputWKUP,
} RTC_AlarmOutput_e;

typedef enum {
    RTC_TamperFilter0,
    RTC_TamperFilter2,      // Tamper is activated after 2 consecutive samples at the active level.
    RTC_TamperFilter4,      // Tamper is activated after 4 consecutive samples at the active level.
    RTC_TamperFilter8,      // Tamper is activated after 8 consecutive samples at the active level.
} RTC_TamperFilterCount_e;

typedef enum {
    RTC_TamperFreqMode0,    // RTCCLK / 32768 (1 Hz when RTCCLK = 32768 Hz)
    RTC_TamperFreqMode1,    // RTCCLK / 16384 (2 Hz when RTCCLK = 32768 Hz)
    RTC_TamperFreqMode2,    // RTCCLK / 8192 (4 Hz when RTCCLK = 32768 Hz)
    RTC_TamperFreqMode3,    // RTCCLK / 4096 (8 Hz when RTCCLK = 32768 Hz)
    RTC_TamperFreqMode4,    // RTCCLK / 2048 (16 Hz when RTCCLK = 32768 Hz)
    RTC_TamperFreqMode5,    // RTCCLK / 1024 (32 Hz when RTCCLK = 32768 Hz)
    RTC_TamperFreqMode6,    // RTCCLK / 512 (64 Hz when RTCCLK = 32768 Hz)
    RTC_TamperFreqMode7,    // RTCCLK / 256 (128 Hz when RTCCLK = 32768 Hz)
} RTC_TamperFrequency_e;

typedef enum {
    RTC_TamperPreCharge1,
    RTC_TamperPreCharge2,
    RTC_TamperPreCharge4,
    RTC_TamperPreCharge8,
} RTC_TamperPreChargeCycles_e;

typedef struct {
    uint8_t SummerTimeMonth;
    uint8_t SummerTimeWeek;
    uint8_t WinterTimeMonth;
    uint8_t WinterTimeWeek;
    uint8_t Hour;
} RTC_DaylightSavingConfig_t;

typedef struct {
    uint8_t Year;       // Last two digits
    uint8_t Month;
    uint8_t Day;
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
    uint16_t Subseconds;        // Read only!
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
    uint8_t TimestampOnDetectionEnabled;            // Save timestamp on tamper detection event
    RTC_TamperFilterCount_e FilterCount;
    RTC_TamperFrequency_e FrequencyMode;
    RTC_TamperPreChargeCycles_e PreChargeCycles;    // The duration of time during which the pull-up/is activated before each sample
    uint8_t TamperTriggerWhenLOW;
    uint8_t PullUpDisabled;
} RTC_TamperDetectionConfig_t;

typedef struct {
    RTC_AlarmOutput_e AlarmOutput;
    RTC_AFMapping_e AFMapping;
    RTC_Config_t Config;
} RTC_Handle_t;

/* ------------ CONSTANTS ------------ */

static const RTC_DaylightSavingConfig_t EUROPEAN_DAYLIGHT_CONFIG = {
    .SummerTimeMonth = 3,
    .SummerTimeWeek = 5,
    .WinterTimeMonth = 10,
    .WinterTimeWeek = 5,
    .Hour = 1,
};

/* ------------ METHODS ------------ */

/*
 * Write protection
 */
void RTC_EnableWriteProtection();
void RTC_DisableWriteProtection();

/*
 * RTC Toggle
 */
void RTC_ClockControl(uint8_t Enabled);

/*
 * Configuration and reset
 */
uint8_t RTC_IsCalendarInitialized();
RTC_Error_e RTC_Init(RTC_Handle_t *pRTCHandle);
RTC_Error_e RTC_ConfigureDayLightSaving(RTC_Handle_t *pRTCHandle, RTC_DaylightSavingConfig_t Config);

/*
 * Reading Calendar
 */
RTC_Error_e RTC_ReadCalendar(RTC_Handle_t *pRTCHandle, RTC_CalendarDate_t *pCalendarDate);

/*
 * Alarm
 */
RTC_Error_e RTC_ConfigureAlarm(RTC_Handle_t *pRTCHandle, RTC_AlarmConfig_t AlarmConfig);
void RTC_AlarmControl(RTC_Alarm_e Alarm, uint8_t Enabled);

/*
 * Wake-up timer
 */
RTC_Error_e RTC_ConfigureWakeUpTimer(RTC_WakeUpTimerConfig_t Config);
void RTC_WakeUpTimerControl(uint8_t Enabled);

/*
 * Alternate function
 */
void RTC_SelectAlarmOutput(RTC_Handle_t *pRTCHandle, RTC_AlarmOutput_e Output, uint8_t PinLowOnTrigger, uint8_t PinOutputPushPull);
RTC_Error_e RTC_MapAdditionalFunction(RTC_Handle_t *pRTCHandle, RTC_AFMapping_e Mapping);

/*
 * Timestamp
 */
RTC_Error_e RTC_ReadTimestamp(RTC_Handle_t *pRTCHandle, RTC_CalendarDate_t *pCalendarDate);
void RTC_TimestampControl(uint8_t Enabled);

/*
 * Tamper detection
 */
void RTC_ConfigureTamperDetection(RTC_TamperDetectionConfig_t Config);
void RTC_TamperDetectionControl(uint8_t Enabled);

/*
 * Flags
 */
void RTC_ClearFlag(RTC_Flag_e Flag);

/*
 * Interrupts
 */
void RTC_EnableInterrupts(uint8_t InterruptsMask);
void RTC_DisableInterrupts(uint8_t InterruptsMask);
void RTC_IQREnable(uint8_t IRQGroupsMask, uint8_t Priority);
void RTC_IQRDisable(uint8_t IRQGroupsMask);
void RTC_IRQHandling(RTC_Handle_t *pRTCHandle);

/*
 * Application callback
 */
void RTC_ApplicationEventCallback(RTC_Handle_t *pRTCHandle, RTC_Event_e AppEvent);


#endif //RTC_DRIVER_H
