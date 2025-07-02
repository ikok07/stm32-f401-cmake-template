//
// Created by Kok on 6/21/25.
//

#include "bme280_i2c_driver.h"

// Used for calculation of pressure
int32_t tFine;

static BME280_Error_t write_to_register(BME280_Handle_t *pBME280Handle, BME280_RegValuePair_t *pRegValuePairs, uint8_t Len);
static BME280_Error_t read_from_register(BME280_Handle_t *pBME280Handle, uint8_t RegisterAddr, uint8_t *pRXBuffer, uint8_t Len);

static BME280_Error_t get_compensation_parameters(BME280_Handle_t *pBME280Handle, BME280_CompensationParameters_t *compensationParameters);
static void fpu_compensate_temperature(BME280_CompensationParameters_t compensationParameters, int32_t adc_temperature, float *final_temperature);
static BME280_Error_t fpu_compensate_pressure(BME280_CompensationParameters_t compensationParameters, int32_t adc_pressure, float *final_pressure);
static void fpu_compensate_humidity(BME280_CompensationParameters_t compensationParameters, int32_t adc_humidity, float *final_humidity);

/**
 * @brief Initializes the device
 * @warning VDDIO and I2C MUST be enabled before calling this method
 * @param pBME280Handle BME280 handle
 * @return OK - 0; ERROR > 0
 */
BME280_Error_t BME280_Configure(BME280_Handle_t *pBME280Handle) {
    uint8_t configReg = 0;
    uint8_t humReg = 0;
    uint8_t measReg = 0;

    // Humidity oversampling
    humReg |= (pBME280Handle->BME280_Config.HumidityOversampling << BME280_CTRL_HUM_OSRSH_POS);

    BME280_RegValuePair_t hum_reg_value_pair = {
        .RegisterAddr = BME280_REG_CTRL_HUM,
        .Value = humReg
   };

    BME280_Error_t err;

    // Apply config
    if ((err = write_to_register(pBME280Handle, &hum_reg_value_pair, 1)) != 0) {
        return err;
    }

    // Filter coefficient
    configReg |= (pBME280Handle->BME280_Config.FilterCoeff << BME280_CONFIG_FILTER_POS);

    // Standby duration
    configReg |= (pBME280Handle->BME280_Config.NormalModeStanbyDuration << BME280_CONFIG_TSB_POS);

    // Pressure oversampling
    measReg |= (pBME280Handle->BME280_Config.PressureOversampling << BME280_CTRL_MEAS_OSRSP_POS);

    // Temperature oversampling
    measReg |= (pBME280Handle->BME280_Config.TemperatureOversampling << BME280_CTRL_MEAS_OSRST_POS);

    BME280_RegValuePair_t reg_value_pairs[2] = {
        {
                .RegisterAddr = BME280_REG_CONFIG,
                .Value = configReg
           },
        {
                .RegisterAddr = BME280_REG_CTRL_MEAS,
                .Value = measReg
           },
    };

    // Apply config
    if ((err = write_to_register(pBME280Handle, reg_value_pairs, 2)) != 0) {
        return err;
    }

    return BME280_ErrOK;
}

/**
 * @brief Reads the current values measured by the sensor
 * @param pBME280Handle BME280 handle
 * @param pResult Result returned from the sensor
 * @return OK - 0; ERROR > 0
 */
BME280_Error_t BME280_GetSample(BME280_Handle_t *pBME280Handle, BME280_Result_t *pResult) {
    uint8_t rawData[8];
    BME280_UncompensatedResult_t uncompensatedResult;
    BME280_Error_t err;

    if ((err = read_from_register(pBME280Handle, BME280_REG_PRESS_MSB, rawData, sizeof(rawData))) != 0) {
        return err;
    }

    uncompensatedResult.UPressure = (rawData[0] << 12) | (rawData[1] << 4) | (rawData[2] >> 4);
    uncompensatedResult.UTemperature = (rawData[3] << 12) | (rawData[4] << 4) | (rawData[5] >> 4);
    uncompensatedResult.UHumidity = (rawData[6] << 8) | rawData[7];

    BME280_CompensationParameters_t compensationParameters;
    if ((err = get_compensation_parameters(pBME280Handle, &compensationParameters)) != 0) {
        return err;
    }

    fpu_compensate_temperature(compensationParameters, uncompensatedResult.UTemperature, &pResult->Temperature);
    if ((err = fpu_compensate_pressure(compensationParameters, uncompensatedResult.UPressure, &pResult->Pressure)) != 0) {
        return err;
    }
    fpu_compensate_humidity(compensationParameters, uncompensatedResult.UHumidity, &pResult->Humidity);

    return BME280_ErrOK;
}

/**
 * @brief Enables the power to the device's sensors
 * @param pBME280Handle BME280 handle
 */
void BME280_EnableVDD(BME280_Handle_t *pBME280Handle) {
    GPIO_WriteToOutputPin(
        pBME280Handle->pBME280_PowerCtrl->GPIO_VDDHandle->pGPIOx,
        pBME280Handle->pBME280_PowerCtrl->GPIO_VDDHandle->GPIO_PinConfig.GPIO_PinNumber,
        ENABLE
    );
}

/**
 * @brief Disables the power to the device's sensors
 * @param pBME280Handle BME280 handle
 */
void BME280_DisableVDD(BME280_Handle_t *pBME280Handle) {
    GPIO_WriteToOutputPin(
        pBME280Handle->pBME280_PowerCtrl->GPIO_VDDHandle->pGPIOx,
        pBME280Handle->pBME280_PowerCtrl->GPIO_VDDHandle->GPIO_PinConfig.GPIO_PinNumber,
        DISABLE
    );
}

/**
 * @brief Enables the power to the device's digital interface
 * @param pBME280Handle BME280 handle
 */
void BME280_EnableVDDIO(BME280_Handle_t *pBME280Handle) {
    GPIO_WriteToOutputPin(
        pBME280Handle->pBME280_PowerCtrl->GPIO_VDDIOHandle->pGPIOx,
        pBME280Handle->pBME280_PowerCtrl->GPIO_VDDIOHandle->GPIO_PinConfig.GPIO_PinNumber,
        ENABLE
    );
}

/**
 * @brief Disables the power to the device's digital interface
 * @param pBME280Handle BME280 handle
 */
void BME280_DisableVDDIO(BME280_Handle_t *pBME280Handle) {
    GPIO_WriteToOutputPin(
        pBME280Handle->pBME280_PowerCtrl->GPIO_VDDIOHandle->pGPIOx,
        pBME280Handle->pBME280_PowerCtrl->GPIO_VDDIOHandle->GPIO_PinConfig.GPIO_PinNumber,
        DISABLE
    );
}

BME280_Error_t BME280_CheckDeviceID(BME280_Handle_t *pBME280Handle) {
    uint8_t err;
    uint8_t regValue;
    if ((err = read_from_register(pBME280Handle, BME280_REG_ID, &regValue, 1))) {
        return err;
    }

    if (regValue != 0x60) return BME280_ErrConnFail;

    return BME280_ErrOK;
}

/**
 * @brief Used for setting the sensor in one of the three supported modes - Sleep, Forced or Normal
 * @param pBME280Handle BME280 handle
 * @param Mode The mode in which to set the device
 * @return OK - 1; ERROR > 0
 */
BME280_Error_t BME280_SetMode(BME280_Handle_t *pBME280Handle, BME280_Mode_e Mode) {
    BME280_Error_t err;

    // Fetch current register value
    uint8_t regValue;
    if ((err = read_from_register(pBME280Handle, BME280_REG_CTRL_MEAS, &regValue, 1))) {
        return err;
    }

    // Device mode
    regValue &=~ (0x03 << BME280_CTRL_MEAS_MODE_POS);
    regValue |= (Mode << BME280_CTRL_MEAS_MODE_POS);

    BME280_RegValuePair_t regValuePair = {
        .RegisterAddr = BME280_REG_CTRL_MEAS,
        .Value = regValue
    };

    if ((err = write_to_register(pBME280Handle, &regValuePair, 1))) {
        return err;
    }

    return BME280_ErrOK;
}

/**
 * @brief Resets the sensor
 * @param pBME280Handle BME280 handle
 */
void BME280_Reset(BME280_Handle_t *pBME280Handle) {
    BME280_DisableVDD(pBME280Handle);
    BME280_EnableVDD(pBME280Handle);
}

BME280_Error_t write_to_register(BME280_Handle_t *pBME280Handle, BME280_RegValuePair_t *pRegValuePairs, uint8_t Len) {
    uint8_t addr = pBME280Handle->BME280_Config.AddrPin == BME280_AddrPinLOW ? 0x76 : 0x77;

    I2C_Error_e err = I2C_MasterSendData(
        pBME280Handle->pI2C_Handle,
        (uint8_t*)pRegValuePairs,
        Len * 2,
        addr,
        I2C_StopEnabled
    );
    if (err != I2C_ErrOK) {
        if (err == I2C_ErrTimeout) return BME280_ErrTimeout;
        return BME280_WriteErr;
    }

    return BME280_ErrOK;
}

BME280_Error_t read_from_register(BME280_Handle_t *pBME280Handle, uint8_t RegisterAddr, uint8_t *pRXBuffer, uint8_t Len) {
    uint8_t addr = pBME280Handle->BME280_Config.AddrPin == BME280_AddrPinLOW ? 0x76 : 0x77;
    I2C_Error_e err = I2C_MasterSendData(
        pBME280Handle->pI2C_Handle,
        &RegisterAddr,
        sizeof(RegisterAddr),
        addr,
        I2C_StopDisabled
    );
    if (err != I2C_ErrOK) {
        if (err == I2C_ErrTimeout) return BME280_ErrTimeout;
        return BME280_WriteErr;
    }

    err = I2C_MasterReceiveData(
        pBME280Handle->pI2C_Handle,
        pRXBuffer,
        Len,
        addr,
        I2C_StopEnabled
    );
    if (err != I2C_ErrOK) {
        if (err == I2C_ErrTimeout) return BME280_ErrTimeout;
        return BME280_ReadErr;
    }

    return 0;
}

BME280_Error_t get_compensation_parameters(BME280_Handle_t *pBME280Handle, BME280_CompensationParameters_t *compensationParameters) {
    uint8_t calib_len = BME280_REG_CALIB25 - BME280_REG_CALIB00 + 1; // 26
    uint8_t humid_len = BME280_REG_CALIB41 - BME280_REG_CALIB26 + 1; // 16
    uint8_t calib_data[26];
    uint8_t humid_data[16];

    BME280_Error_t err;

    // Read up to Dig_H1
    if ((err = read_from_register(pBME280Handle, BME280_REG_CALIB00, calib_data, calib_len))) {
        return err;
    }

    // Read from Dig_H2 to the end
    if ((err = read_from_register(pBME280Handle, BME280_REG_CALIB26, humid_data, humid_len))) {
        return err;
    }

    // Extract T and P parameters
    compensationParameters->Dig_T1 = (uint16_t)((calib_data[1] << 8) | calib_data[0]);
    compensationParameters->Dig_T2 = (int16_t)((calib_data[3] << 8) | calib_data[2]);
    compensationParameters->Dig_T3 = (int16_t)((calib_data[5] << 8) | calib_data[4]);

    compensationParameters->Dig_P1 = (uint16_t)((calib_data[7] << 8) | calib_data[6]);
    compensationParameters->Dig_P2 = (int16_t)((calib_data[9] << 8) | calib_data[8]);
    compensationParameters->Dig_P3 = (int16_t)((calib_data[11] << 8) | calib_data[10]);
    compensationParameters->Dig_P4 = (int16_t)((calib_data[13] << 8) | calib_data[12]);
    compensationParameters->Dig_P5 = (int16_t)((calib_data[15] << 8) | calib_data[14]);
    compensationParameters->Dig_P6 = (int16_t)((calib_data[17] << 8) | calib_data[16]);
    compensationParameters->Dig_P7 = (int16_t)((calib_data[19] << 8) | calib_data[18]);
    compensationParameters->Dig_P8 = (int16_t)((calib_data[21] << 8) | calib_data[20]);
    compensationParameters->Dig_P9 = (int16_t)((calib_data[23] << 8) | calib_data[22]);

    // Extract H1
    compensationParameters->Dig_H1 = calib_data[24];

    // Extract humidity parameters
    compensationParameters->Dig_H2 = (int16_t)((humid_data[1] << 8) | humid_data[0]);
    compensationParameters->Dig_H3 = humid_data[2];

    /* dig_H4 and dig_H5 require special handling due to their bit packing */

    // dig_H4[11:4] from 0xE4, dig_H4[3:0] from 0xE5[3:0]
    compensationParameters->Dig_H4 = (int16_t)((humid_data[3] << 4) | (humid_data[4] & 0x0F));

    // dig_H5[11:4] from 0xE6, dig_H5[3:0] from 0xE5[7:4]
    compensationParameters->Dig_H5 = (int16_t)((humid_data[5] << 4) | (humid_data[4] >> 4));

    compensationParameters->Dig_H6 = (int8_t)humid_data[6];
    return BME280_ErrOK;
}

void fpu_compensate_temperature(BME280_CompensationParameters_t compensationParameters, int32_t adc_temperature, float *final_temperature) {
    float var1, var2, T;
    var1 = (((float)adc_temperature) / 16384.0f - ((float)compensationParameters.Dig_T1) / 1024.0f) * ((float)compensationParameters.Dig_T2);
    var2 = (((float)adc_temperature) / 131072.0f - ((float)compensationParameters.Dig_T1)/8192.0f) * (((float)adc_temperature / 131072.0 - ((float)compensationParameters.Dig_T1) / 8192.0f)) * ((float)compensationParameters.Dig_T3);

    tFine = (int32_t)(var1 + var2);

    // Final temperature calculation
    T = (var1 + var2) / 5120.0f;

    *final_temperature = T;
}

BME280_Error_t fpu_compensate_pressure(BME280_CompensationParameters_t compensationParameters, int32_t adc_pressure, float *final_pressure) {
    float var1, var2, p;

    // Note: tFine should be calculated from temperature compensation first

    var1 = ((float)tFine/2.0f) - 64000.0f;
    var2 = var1 * var1 * ((float)compensationParameters.Dig_P6) / 32768.0f;
    var2 = var2 + var1 * ((float)compensationParameters.Dig_P5) * 2.0f;
    var2 = (var2/4.0f) + (((float)compensationParameters.Dig_P4) * 65536.0f);

    var1 = (((float)compensationParameters.Dig_P3) * var1 * var1 / 524288.0f + ((float)compensationParameters.Dig_P2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)compensationParameters.Dig_P1);

    if (var1 == 0.0f) {
        *final_pressure = 0.0f;
        return BME280_PressErr;
    }

    p = 1048576.0f - (float)adc_pressure;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;

    var1 = ((float)compensationParameters.Dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)compensationParameters.Dig_P8) / 32768.0f;
    p = p + (var1 + var2 + ((float)compensationParameters.Dig_P7)) / 16.0f;

    *final_pressure = p;
    return BME280_ErrOK;
}

void fpu_compensate_humidity(BME280_CompensationParameters_t compensationParameters, int32_t adc_humidity, float *final_humidity) {
    float varH;

    // Note: tFine should be calculated from temperature compensation first

    varH = (((float)tFine) - 76800.0f);
    varH = (adc_humidity - (((float)compensationParameters.Dig_H4) * 64.0f + ((float)compensationParameters.Dig_H5) / 16384.0f * varH)) * (((float)compensationParameters.Dig_H2) / 65536.0f * (1.0f + ((float)compensationParameters.Dig_H6) / 67108864.0f * varH * (1.0f + ((float)compensationParameters.Dig_H3) / 67108864.0f * varH)));
    varH = varH * (1.0f - ((float)compensationParameters.Dig_H1) * varH / 524288.0f);

    if (varH > 100.0f) {
        varH = 100.0f;
    } else if (varH < 0.0f) {
        varH = 0.0f;
    }

    *final_humidity = varH;
}