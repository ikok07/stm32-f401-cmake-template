//
// Created by Kok on 6/21/25.
//

#include "bme280_i2c_driver.h"

// Used for calculation of pressure
int32_t tFine;

static uint8_t write_to_register(BME280_Handle_t *pBME280Handle, BME280_RegValuePair_t *pRegValuePairs, uint8_t Len);
static uint8_t read_from_register(BME280_Handle_t *pBME280Handle, uint8_t RegisterAddr, uint8_t *pRXBuffer, uint8_t Len);

static uint8_t get_compensation_parameters(BME280_Handle_t *pBME280Handle, BME280_CompensationParameters_t *compensationParameters);
static uint8_t fpu_compensate_temperature(BME280_CompensationParameters_t compensationParameters, int32_t adc_temperature, float *final_temperature);
static uint8_t fpu_compensate_pressure(BME280_CompensationParameters_t compensationParameters, int32_t adc_pressure, float *final_pressure);
static uint8_t fpu_compensate_humidity(BME280_CompensationParameters_t compensationParameters, int32_t adc_humidity, float *final_humidity)

/**
 * @brief Initializes the device
 * @warning VDDIO and I2C MUST be enabled before calling this method
 * @param pBME280Handle BME280 handle
 * @return OK - 0; ERROR > 0
 */
uint8_t BME280_Configure(BME280_Handle_t *pBME280Handle) {

    uint8_t configReg = 0;
    uint8_t humReg = 0;
    uint8_t measReg = 0;

    // Humidity oversampling
    humReg |= (pBME280Handle->BME280_Config.HumidityOversampling << BME280_CTRL_HUM_OSRSH_POS);

    BME280_RegValuePair_t hum_reg_value_pair = {
        .RegisterAddr = BME280_REG_CTRL_HUM,
        .Value = humReg
   };

    uint8_t err;

    // Apply config
    if ((err = write_to_register(pBME280Handle, &hum_reg_value_pair, 1)) != 0) {
        return err;
    }

    // Filter coefficient
    configReg |= (pBME280Handle->BME280_Config.FilterCoeff << BME280_CONFIG_FILTER_POS);

    // Standby duration
    configReg |= (pBME280Handle->BME280_Config.NormalModeStanbyDuration << BME280_CONFIG_TSB_POS);

    // Device mode
    measReg |= (pBME280Handle->BME280_Config.Mode << BME280_CTRL_MEAS_MODE_POS);

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

    return 0;
}

uint8_t BME280_GetSample(BME280_Handle_t *pBME280Handle, BME280_Result_t *pResult) {
    BME280_UncompensatedResult_t *uncompensatedResult[1];
    uint8_t err;
    if ((err = read_from_register(pBME280Handle, BME280_REG_PRESS_MSB, uncompensatedResult, sizeof(uncompensatedResult))) != 0) {
        return err;
    }


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

/**
 * @brief Resets the sensor
 * @param pBME280Handle BME280 handle
 */
void BME280_Reset(BME280_Handle_t *pBME280Handle) {
    BME280_DisableVDD(pBME280Handle);
    BME280_EnableVDD(pBME280Handle);
}

uint8_t write_to_register(BME280_Handle_t *pBME280Handle, BME280_RegValuePair_t *pRegValuePairs, uint8_t Len) {
    uint8_t addr = pBME280Handle->BME280_Config.AddrPin == BME280_AddrPinLOW ? 0x76 : 0x77;

    I2C_Error_e err = I2C_MasterSendData(
        pBME280Handle->pI2C_Handle,
        (uint8_t*)pRegValuePairs,
        Len * 2,
        addr,
        I2C_StopEnabled
    );
    if (err != I2C_ErrOK) {
        return 1;
    }

    return 0;
}

uint8_t read_from_register(BME280_Handle_t *pBME280Handle, uint8_t RegisterAddr, uint8_t *pRXBuffer, uint8_t Len) {
    uint8_t addr = pBME280Handle->BME280_Config.AddrPin == BME280_AddrPinLOW ? 0x76 : 0x77;
    I2C_Error_e err = I2C_MasterSendData(
        pBME280Handle->pI2C_Handle,
        &RegisterAddr,
        sizeof(RegisterAddr),
        addr,
        I2C_StopDisabled
    );
    if (err != I2C_ErrOK) {
        return 1;
    }

    err = I2C_MasterReceiveData(
        pBME280Handle->pI2C_Handle,
        pRXBuffer,
        Len,
        addr,
        I2C_StopEnabled
    );
    if (err != I2C_ErrOK) {
        return 2;
    }

    return 0;
}

uint8_t get_compensation_parameters(BME280_Handle_t *pBME280Handle, BME280_CompensationParameters_t *compensationParameters) {
    uint8_t calib_len = BME280_REG_CALIB25 - BME280_REG_CALIB00; // 25
    uint8_t humid_len = BME280_REG_CALIB41 - BME280_REG_CALIB26; // 7
    uint8_t calib_data[25];
    uint8_t humid_data[7];

    uint8_t err;

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
    return 0;
}

uint8_t fpu_compensate_temperature(BME280_CompensationParameters_t compensationParameters, int32_t adc_temperature, float *final_temperature) {
    float var1, var2, T;
    var1 = (((float)adc_temperature) / 16384.0f - ((float)compensationParameters.Dig_T1) / 1024.0f) * ((float)compensationParameters.Dig_T2);
    var2 = (((float)adc_temperature) / 131072.0f - ((float)compensationParameters.Dig_T1)/8192.0f) * (((float)adc_temperature / 131072.0 - ((float)compensationParameters.Dig_T1) / 8192.0f)) * ((float)compensationParameters.Dig_T3);

    tFine = (int32_t)(var1 + var2);

    // Final temperature calculation
    T = (var1 + var2) / 5120.0f;

    *final_temperature = T;
    return 0;
}

uint8_t fpu_compensate_pressure(BME280_CompensationParameters_t compensationParameters, int32_t adc_pressure, float *final_pressure) {
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
        return 0;
    }

    p = 1048576.0f - (float)adc_pressure;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;

    var1 = ((float)compensationParameters.Dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)compensationParameters.Dig_P8) / 32768.0f;
    p = p + (var1 + var2 + ((float)compensationParameters.Dig_P7)) / 16.0f;

    *final_pressure = p;
    return 0;
}

uint8_t fpu_compensate_humidity(BME280_CompensationParameters_t compensationParameters, int32_t adc_humidity, float *final_humidity) {
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

    return 0;
}