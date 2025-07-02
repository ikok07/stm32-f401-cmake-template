//
// Created by Kok on 6/25/25.
//

#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include "stm32f4xx.h"

#define TIMEOUT_MS                  3000

#define USART_INDEX_TO_IRQ_NUMBER(perIndex)             perIndex == 1 ? USART1_IRQn : perIndex == 2 ? USART2_IRQn : perIndex == 6 ? USART6_IRQn : USART1_IRQn

/* ------------ ERROR CODES ------------ */

typedef enum {
    USART_ErrOK,
    USART_ErrTimeout,
    USART_ErrArgumentNULL,
    USART_ErrInvalidBaudRate,
    USART_PerBusy,
    USART_FPUNotEnabled
} USART_Error_e;

/* ------------ CONFIG STRUCTURES ------------ */

typedef enum {
    USART_EventTxComplete,
    USART_EventRxComplete,
    USART_EventCTS,
    USART_EventOverrunDetected,
    USART_EventIdleDetected,
    USART_EventParityError,
    USART_EventBreakFlag,
    USART_EventNoiseFlag,
    USART_EventFramingError,
} USART_Event_e;

typedef enum {
    USART_BaudRate_9600     = 9600,
    USAR_BaudRage_19200     = 19200,
    USAR_BaudRage_38400     = 38400,
    USAR_BaudRage_115200    = 115200,
} USART_BaudRate_e;

typedef enum {
    USART_OversamplingBy16,
    USART_OversamplingBy8,
} USART_Oversampling_e;

typedef enum {
    USART_WordLength8Bits,
    USART_WordLength9Bits,
} USART_WordLength_e;

typedef enum {
    USART_ParityControlDisabled,
    USART_ParityControlEnabled
} USART_ParityControl_e;

typedef enum {
    USART_EvenParity,
    USART_OddParity
} USART_ParitySelection_e;

typedef enum {
    USART_StopBits1,        // 1 bit
    USART_StopBits05,       // 0.5 bits
    USART_StopBits2,        // 2 bits
    USART_StopBits15,       // 1.5 bits
} USART_StopBits_e;

typedef enum {
    USART_ClockDisabled,
    USART_ClockEnabled
} USART_ClkControl_e;

typedef enum {
    USART_PolarityLOW,
    USART_PolarityHIGH,
} USART_CPOL_e;

typedef enum {
    USART_PhaseFirstEdge,
    USART_PhaseSecondEdge,
} USART_CPHA_e;

typedef enum {
    USART_LastBitLOW,
    USART_LastBitHIGH,
} USART_LastBitClockPulse_e;

typedef enum {
    USART_ThreeSampleBitMethod,             // Used in noisy environments
    USART_OneSampleBitMethod,               // Used in less noisy environments
} USART_BitMethod_e;

typedef enum {
    USART_CTSDisabled,
    USART_CTSEnabled
} USART_CTSControl_e;

typedef enum {
    USART_RTSDisabled,
    USART_RTSEnabled
} USART_RTSControl_e;

typedef enum {
    USART_Disabled,
    USART_Enabled
} USART_PwrState;

typedef enum {
    USART_InterruptTXEmpty,
    USART_InterruptTXComplete,
    USART_InterruptRXNotEmpty,              // Includes RXNE and ORE flags
    USART_InterruptCTS,
    USART_InterruptIdleLine,
    USART_InterruptParityErr,
    USART_InterruptBreakFlag,
    USART_InterruptGenericErrors            // Includes Noise, Overrun and Framing flags
} USART_Interrupt_e;

typedef enum {
    USART_InterruptStateReady,
    USART_InterruptStateTXBusy,
    USART_InterruptStateRXBusy,
} USART_InterruptState_e;

// WARNING: The order of these flag SHOULD be EXACTLY like that due to the placement in status register
typedef enum {
    USART_FlagPE,
    USART_FlagFE,
    USART_FlagNF,
    USART_FlagORE,
    USART_FlagIDLE,
    USART_FlagRXNE,
    USART_FlagTC,
    USART_FlagTXE,
    USART_FlagLBD,
    USART_FlagCTS,
} USART_Flag_e;

typedef enum {
    USART_FlagDisabled,
    USART_FlagEnabled,
} USART_FlagStatus_e;

typedef struct {
    USART_ClkControl_e ClockControl;
    USART_CPOL_e CPOL;
    USART_CPHA_e CPHA;
    USART_LastBitClockPulse_e LastBitClockPulse;
} USART_ClkConfig_t;

typedef struct {
    USART_BaudRate_e BaudRate;
    USART_Oversampling_e Oversampling;
    USART_WordLength_e WordLength;
    USART_ParityControl_e ParityControl;                    // Enabled or disabled parity bit
    USART_ParitySelection_e ParitySelection;                // Even or Odd
    USART_StopBits_e StopBits;
    USART_BitMethod_e BitMethod;
    USART_CTSControl_e CTSControl;
    USART_RTSControl_e RTSControl;
    USART_ClkConfig_t ClockConfig;
} USART_Config_t;

typedef struct {
    uint8_t *pTXBuffer;
    uint8_t *pRXBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint32_t RxLenOriginal;
    USART_InterruptState_e InterruptState;
} USART_ITState;

typedef struct {
    USART_TypeDef *pUSARTx;
    USART_Config_t USART_Config;
    USART_ITState USART_ITState;
} USART_Handle_t;

typedef struct {
    uint16_t Mantissa;
    uint8_t Fraction;
} USART_BaudRateResult_t;

/* ------------ METHODS ------------ */

/*
 * Peripheral control
 */
void USART_PeriClockControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled);
void USART_PeripheralControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled);
void USART_PeripheralTXControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled);
void USART_PeripheralRXControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled);
USART_PwrState USART_PeripheralEnabled(USART_Handle_t *pUSARTHandle);

/*
 * Init and De-Init
 */
USART_Error_e USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_Handle_t *pUSARTHandle);

/*
 * Data send and receive
 */
USART_Error_e USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint32_t Len);
void USART_SendBreak(USART_Handle_t *pUSARTHandle);
USART_Error_e USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Len);

USART_Error_e USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint32_t Len);
USART_Error_e USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Len);

/*
 * Interrupt config
 */

USART_Error_e USART_IRQEnable(USART_Handle_t *pUSARTHandle, USART_Interrupt_e *EnabledInterrupts, uint32_t Len);
void USART_IRQDisable(USART_Handle_t *pUSARTHandle);
void USART_IRQHandler(USART_Handle_t *pUSARTHandle);

/*
 * Hardware flow control
 */
void USART_CTSControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled);
void USART_RSSControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled);

/*
 * Other methods
 */
USART_Error_e USART_SetBaudRate(USART_Handle_t *pUSARTHandle, USART_BaudRate_e BaudRate);

USART_FlagStatus_e USART_GetFlag(USART_Handle_t *pUSARTHandle, USART_Flag_e Flag);
void USART_ClearFlag(USART_Handle_t *pUSARTHandle, USART_Flag_e Flag);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, USART_Event_e AppEvent);

#endif //USART_DRIVER_H
