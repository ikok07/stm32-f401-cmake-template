//
// Created by Kok on 6/16/25.
//

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "stm32f4xx.h"

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
#define I2C_MAX_TRISE_NS_FOR_SPEED(speed)                  (speed <= I2C_SclSpeedSM ? 1000 : 300)

#define TIMEOUT_MS              3000

/* ------------ ERROR CODES ------------ */

typedef enum {
    I2C_ErrOK,
    I2C_ErrTimeout,
    I2C_ErrResetEnabled,
    I2C_ErrLenZero,
    I2C_ErrPerNotReady
} I2C_Error_e;


/* ------------ CONFIG STRUCTURES ------------ */

typedef enum {
    I2C_Index_1 = 1,
    I2C_Index_2,
    I2C_Index_3
} I2C_PerIndex_e;

typedef enum {
    I2C_SclSpeedSM = 100000,
    I2C_SclSpeedFM2K = 200000,
    I2C_SclSpeedFM3K = 300000,
    I2C_SclSpeedFM4K = 400000,
} I2C_SclSpeed_e;

typedef enum {
    I2C_DeviceAddr7Bits,
    I2C_DeviceAddr10Bits,
} I2C_DeviceAddrLen_e;

typedef enum {
    I2C_FmDuty2,
    I2C_FmDuty16_9,
} I2C_FmDutyCycle_e;

typedef enum {
    I2C_ResetDisabled,
    I2C_ResetEnabled,
} I2C_ResetState_e;

typedef enum {
    I2C_StopEnabled,
    I2C_StopDisabled
} I2C_DisableStop_e;

typedef enum {
    I2C_Ready,
    I2C_RxBusy,
    I2C_TxBusy
} I2C_TxRxState_e;

typedef enum {
    I2C_EventTxComplete,
    I2C_EventRxComplete,
    I2C_EventDataRequest,                   // when the slave sends data to master
    I2C_EventDataReceive,                   // when the master sends data to the slave
    I2C_EventStop,
    I2C_EventBusError,
    I2C_EventArbLost,
    I2C_EventNACK,
    I2C_EventOvr,
    I2C_EventTimeout
} I2C_Event_e;

typedef struct {
    I2C_SclSpeed_e I2C_SCLSpeed;
    uint16_t I2C_DeviceAddress;
    I2C_DeviceAddrLen_e I2C_DeviceAddressLen;
    I2C_FmDutyCycle_e I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct {
    uint8_t *pTXBuffer;
    uint8_t *pRXBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint32_t RxLenOriginal;
    I2C_TxRxState_e TxRxState;
    uint8_t DevAddr;                            // Slave device address
    uint8_t WriteEnabled;                       // Indicates if the next operation should be WRITE or READ
} I2C_ITState_t;

typedef struct {
    I2C_TypeDef *pI2Cx;
    I2C_Config_t I2C_Config;
    I2C_ResetState_e I2C_ResetState;
    I2C_ITState_t I2C_ITState;                  // Used when using interrupts
} I2C_Handle_t;

/* ------------ METHODS ------------ */

/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t Enable);

/*
 * Init and De-Init
 */
I2C_Error_e I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_TypeDef *pI2Cx);

/*
 * Data send and receive
 */
I2C_Error_e I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint8_t Len, uint16_t SlaveAddr, I2C_DisableStop_e DisableStop);
I2C_Error_e I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, uint16_t SlaveAddr, I2C_DisableStop_e DisableStop);

I2C_Error_e I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint8_t Len, uint16_t SlaveAddr);
I2C_Error_e I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, uint16_t SlaveAddr);

void I2C_SlaveConfigure(I2C_PerIndex_e I2CIndex, I2C_Handle_t *pI2CHandle);
void I2C_SlaveSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t data);
void I2C_SlaveReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer);

/*
 * Other controls
 */
uint8_t I2C_PeripheralEnabled(I2C_TypeDef *pI2Cx);
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t Enable);
void I2C_AcknowledgeControl(I2C_TypeDef *pI2Cx, uint8_t Enabled);
void I2C_SetResetState(I2C_Handle_t *pI2CHandle, uint8_t Enable);

/*
 * IRQ Configuration
 */
void I2C_IRQConfig(I2C_PerIndex_e PerIndex, uint8_t IRQPriority, uint8_t Enable);
void I2C_IRQEventHandling(I2C_Handle_t *pI2CHandle);
void I2C_IRQErrorHandling(I2C_Handle_t *pI2CHandle);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, I2C_Event_e AppEvent);

#endif //I2C_DRIVER_H
