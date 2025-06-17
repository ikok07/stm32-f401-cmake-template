//
// Created by Kok on 6/16/25.
//

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "stm32f4xx.h"

// TODO: Support 10-bit register addresses

/**
 * @brief Enables the I2C peripheral clock
 * @param offset The offset of the enable register specific to each I2C
 */
#define I2C_PCLK_EN(offset) RCC->APB1ENR |= (1 << offset);

/**
 * @brief Disables the I2C peripheral clock
 * @param offset The offset of the enable register specific to each I2C
 */
#define I2C_PCLK_DI(offset) RCC->APB1ENR &=~ (1 << offset);

/**
 * @brief Converts I2C index to it's corresponding IRQ number
 * @param index The index of the I2C peripheral
 * @param eventInterrupt If the IRQ number should be for the event interrupt (otherwise it's error interrupt)
 */
#define I2C_INDEX_TO_IRQ_NUMBER(index, eventInterrupt) ((index == 1) ? eventInterrupt ? I2C1_EV_IRQn : I2C1_ER_IRQn :\
                                                        (index == 2) ? eventInterrupt ? I2C2_EV_IRQn : I2C2_ER_IRQn :\
                                                        (index == 3) ? eventInterrupt ? I2C3_EV_IRQn : I2C3_ER_IRQn :\
                                                        I2C1_EV_IRQn)

/**
 * @brief Returns the maximum rise time for both SDA and SCL signals
 * @param speed The speed of the I2C peripheral
 */
#define I2C_MAX_TRISE_NS_FOR_SPEED(speed)                  (speed <= I2C_SCL_SPEED_SM ? 1000 : 300)

/**
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM                100000
#define I2C_SCL_SPEED_FM2K              200000
#define I2C_SCL_SPEED_FM3K              300000
#define I2C_SCL_SPEED_FM4K              400000

/**
 * @I2C_DeviceAddressLen
 */
#define I2C_DEVICE_ADDR_7_BITS          0
#define I2C_DEVICE_ADDR_10_BITS         1

/**
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2                   0
#define I2C_FM_DUTY_16_9                1

/**
 * @I2C_ResetState
 */
#define I2C_RESET_DISABLED              0
#define I2C_RESET_ENABLED               1

/**
 * @I2C_DisableStart
 */
#define I2C_START_ENABLED               0
#define I2C_START_DISABLED              1
typedef struct {
    uint32_t I2C_SCLSpeed;                      /** Possible values from @I2C_SCLSpeed */
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_DeviceAddressLen;               /** Possible values from @I2C_DeviceAddressLen */
    uint16_t I2C_FMDutyCycle;                   /** Possible values from @I2C_FMDutyCycle */
} I2C_Config_t;

typedef struct {
    I2C_TypeDef *pI2Cx;
    I2C_Config_t I2C_Config;
    uint8_t I2C_ResetState;                     /** Possible values from @I2C_ResetState */
} I2C_Handle_t;

/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t Enable);

/*
 * Init and De-Init
 */
uint8_t I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_TypeDef *pI2Cx);

/*
 * Data send and receive
 */
uint8_t I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint8_t Len, uint16_t SlaveAddr, uint8_t DisableStop);
uint8_t I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, uint16_t SlaveAddr, uint8_t DisableStop);

/*
 * Other controls
 */
uint8_t I2C_PeripheralEnabled(I2C_TypeDef *pI2Cx);
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t Enable);
void I2C_SetResetState(I2C_Handle_t *pI2CHandle, uint8_t Enable);

/*
 * IRQ Configuration
 */
void I2C_IRQConfig(uint8_t PerIndex, uint8_t IRQPriority, uint8_t Enable);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

#endif //I2C_DRIVER_H
