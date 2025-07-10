//
// Created by Kok on 7/8/25.
//

#ifndef POWER_DRIVER_H
#define POWER_DRIVER_H

#include <stdint.h>

/* ------------ MACROS ------------ */

#define PWR_TIMEOUT_MS              3000

/* ------------ ERROR CODES ------------ */

typedef enum {
    PWR_ErrOK,
    PWR_ErrTimeout,
    PWR_ErrBackupRegLocked
} PWR_Error_e;

/* ------------ CONFIG STRUCTURES ------------ */

typedef enum {
    PWR_RegulatorScale3 = 1,
    PWR_RegulatorScale2,
} PWR_RegulatorScaling_e;

typedef enum {
    PWR_PVDThres2_2V,
    PWR_PVDThres2_3V,
    PWR_PVDThres2_4V,
    PWR_PVDThres2_5V,
    PWR_PVDThres2_6V,
    PWR_PVDThres2_7V,
    PWR_PVDThres2_8V,
    PWR_PVDThres2_9V
} PWR_PDVThreshold_e;

typedef enum {
    PWR_WakeUpSrcInterrupt,
    PWR_WakeUpSrcEXTIEvent,
} PWR_WakeUpSrc_e;

typedef struct {
    PWR_WakeUpSrc_e Source;
    uint8_t WKUPPinEnabled;                 // Port A, Pin 0
} PWR_WakeUpConfig_t;

typedef struct {
    PWR_RegulatorScaling_e RegulatorVoltageScaling;
    uint8_t MainRegulatorLowVoltageEnabled;             // Main regulator in Low Voltage and Flash memory in Deep Sleep mode when the device is in Stop mode.
    uint8_t LowPowerRegulatorLowVoltageEnabled;         // Low-power regulator in Low Voltage and Flash memory in Deep Sleep mode if LowPowerRegulatorInStopMode is set when the device is in Stop mode.
    uint8_t FlashPowerDownInStopMode;
    PWR_PDVThreshold_e PVDThreshold;
    uint8_t PVDEnabled;
    uint8_t StandbyModeInDeepSleepEnabled;
    uint8_t LowPowerRegulatorInStopMode;
    uint8_t BackupRegulatorEnabled;
    PWR_WakeUpConfig_t WakeUpConfig;
} PWR_Config_t;

typedef struct {
    PWR_Config_t Config;
} PWR_Handle_t;

/* ------------ METHODS ------------ */

/*
 * Peripheral controls
 */
void PWR_PeriClockControl(uint8_t Enabled);

/*
 * Init and De-Init
 */
PWR_Error_e PWR_Init(PWR_Handle_t *pwrHandle);
void PWR_DeInit();

/*
 * Low-Power modes
 */
void PWR_EnterSleepMode(PWR_Handle_t *pwrHandle);
void PWR_EnterDeepSleepMode(PWR_Handle_t *pwrHandle);

/*
 * User methods
 */
void PWR_UnlockBackupRegisters();
void PWR_LockBackupRegisters();
void PWR_ClearWakeUpFlags();


#endif //POWER_DRIVER_H
