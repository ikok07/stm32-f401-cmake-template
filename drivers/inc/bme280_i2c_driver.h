//
// Created by Kok on 6/22/25.
//

#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H

#include <stdint.h>

#include "gpio_driver.h"
#include "i2c_driver.h"

/* ------------ REGISTERS ------------ */

#define BME280_REG_HUM_LSB              0xFE
#define BME280_REG_HUM_MSB              0xFD

#define BME280_REG_TEMP_XLSB            0xFC
#define BME280_REG_TEMP_LSB             0xFB
#define BME280_REG_TEMP_MSB             0xFA

#define BME280_REG_PRESS_XLSB           0xF9
#define BME280_REG_PRESS_LSB            0xF8
#define BME280_REG_PRESS_MSB            0xF7

#define BME280_REG_CONFIG               0xF5
#define BME280_REG_CTRL_MEAS            0xF4
#define BME280_REG_CTRL_STATUS          0xF3
#define BME280_REG_CTRL_HUM             0xF2

#define BME280_REG_CALIB41              0xF0
#define BME280_REG_CALIB26              0xE1

#define BME280_REG_RESET                0xE0
#define BME280_REG_ID                   0xD0

#define BME280_REG_CALIB25              0xA1
#define BME280_REG_CALIB00              0x88

/* ------------ BIT_OFFSETS ------------ */

#define BME280_CONFIG_FILTER_POS                2U
#define BME280_CONFIG_TSB_POS                   5U

#define BME280_CTRL_HUM_OSRSH_POS               0U

#define BME280_CTRL_MEAS_OSRSP_POS              2U
#define BME280_CTRL_MEAS_OSRST_POS              5U
#define BME280_CTRL_MEAS_MODE_POS               0U

/* ------------ ERROR CODES ------------ */

typedef enum {
    BME280_ErrOK,
    BME280_ErrTimeout,
    BME280_ErrConnFail,
    BME280_WriteErr,
    BME280_ReadErr,
    BME280_PressErr
} BME280_Error_t;

/* ------------ CONFIG STRUCTURES ------------ */

typedef enum {
    BME280_AddrPinLOW,
    BME280_AddrPinHIGH,
} BME280_AddrPin_e;

typedef enum {
    BME280_ModeSleep,
    BME280_ModeForced,
    BME280_ModeNormal = 3,
} BME280_Mode_e;

typedef enum {
    BME280_OversamplingOFF,
    BME280_Oversampling1,
    BME280_Oversampling2,
    BME280_Oversampling4,
    BME280_Oversampling8,
    BME280_Oversampling16,
} BME280_Oversampling_e;

typedef enum {
    BME280_StandbyDuration05,            //     0.5ms
    BME280_StandbyDuration625,           //     62.5ms
    BME280_StandbyDuration1250,          //     125ms
    BME280_StandbyDuration2500,          //     250ms
    BME280_StandbyDuration5000,          //     500ms
    BME280_StandbyDuration10000,         //     1000ms
    BME280_StandbyDuration100,           //     10ms
    BME280_StandbyDuration200,           //     20ms
} BME280_StandbyDuration_e;

typedef enum {
    BME280_FilterCoeffOFF,
    BME280_FilterCoeff2,
    BME280_FilterCoeff4,
    BME280_FilterCoeff8,
    BME280_FilterCoeff16,
} BME280_FilterCoeff_e;

typedef struct {
    BME280_AddrPin_e AddrPin;
    BME280_FilterCoeff_e FilterCoeff;
    BME280_Oversampling_e PressureOversampling;
    BME280_Oversampling_e HumidityOversampling;
    BME280_Oversampling_e TemperatureOversampling;
    BME280_StandbyDuration_e NormalModeStandbyDuration;
} BME280_Config_t;

typedef struct {
    GPIO_Handle_t *GPIO_VDDHandle;
    GPIO_Handle_t *GPIO_VDDIOHandle;
} BME280_PowerCtrl_t;

typedef struct {
    I2C_Handle_t *pI2C_Handle;
    BME280_PowerCtrl_t *pBME280_PowerCtrl;
    BME280_Config_t BME280_Config;
} BME280_Handle_t;

typedef struct {
    uint8_t RegisterAddr;
    uint8_t Value;
} __attribute__((packed)) BME280_RegValuePair_t;

typedef struct {
    uint16_t Dig_T1;
    int16_t Dig_T2;
    int16_t Dig_T3;
    uint16_t Dig_P1;
    int16_t Dig_P2;
    int16_t Dig_P3;
    int16_t Dig_P4;
    int16_t Dig_P5;
    int16_t Dig_P6;
    int16_t Dig_P7;
    int16_t Dig_P8;
    int16_t Dig_P9;
    uint8_t Dig_H1;
    int16_t Dig_H2;
    uint8_t Dig_H3;
    int16_t Dig_H4;
    int16_t Dig_H5;
    int8_t Dig_H6;
} __attribute__((packed)) BME280_CompensationParameters_t;

typedef struct {
    int32_t UPressure;
    int32_t UTemperature;
    int32_t UHumidity;
} BME280_UncompensatedResult_t;

typedef struct {
    float Pressure;
    float Temperature;
    float Humidity;
} BME280_Result_t;

/* ------------ METHODS ------------ */

/*
 * Confugration
 */
BME280_Error_t BME280_Configure(BME280_Handle_t *pBME280Handle);

/*
 * Read data
 */
 BME280_Error_t BME280_GetSample(BME280_Handle_t *pBME280Handle, BME280_Result_t *pResult);

/*
 * Power controls
 */
void BME280_EnableVDD(BME280_Handle_t *pBME280Handle);
void BME280_DisableVDD(BME280_Handle_t *pBME280Handle);
void BME280_EnableVDDIO(BME280_Handle_t *pBME280Handle);
void BME280_DisableVDDIO(BME280_Handle_t *pBME280Handle);

/*
 * Other controls
 */
BME280_Error_t BME280_CheckDeviceID(BME280_Handle_t *pBME280Handle);
BME280_Error_t BME280_SetMode(BME280_Handle_t *pBME280Handle, BME280_Mode_e Mode);
void BME280_Reset(BME280_Handle_t *pBME280Handle);

#endif //BME280_DRIVER_H
