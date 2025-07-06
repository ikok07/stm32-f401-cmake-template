//
// Created by Kok on 7/6/25.
//

#ifndef DMA_DRIVER_H
#define DMA_DRIVER_H

/* ------------ MACROS ------------ */

#define DMA_STREAM_TO_IRQ_NUMBER(stream)            stream == DMA1_Stream0 ? DMA1_Stream0_IRQn : \
                                                    stream == DMA1_Stream1 ? DMA1_Stream1_IRQn : \
                                                    stream == DMA1_Stream2 ? DMA1_Stream2_IRQn : \
                                                    stream == DMA1_Stream3 ? DMA1_Stream3_IRQn : \
                                                    stream == DMA1_Stream4 ? DMA1_Stream4_IRQn : \
                                                    stream == DMA1_Stream5 ? DMA1_Stream5_IRQn : \
                                                    stream == DMA1_Stream6 ? DMA1_Stream6_IRQn : \
                                                    stream == DMA1_Stream7 ? DMA1_Stream7_IRQn : \
                                                    stream == DMA2_Stream0 ? DMA2_Stream0_IRQn : \
                                                    stream == DMA2_Stream1 ? DMA2_Stream1_IRQn : \
                                                   stream == DMA2_Stream2 ? DMA2_Stream2_IRQn : \
                                                    stream == DMA2_Stream3 ? DMA2_Stream3_IRQn : \
                                                    stream == DMA2_Stream4 ? DMA2_Stream4_IRQn : \
                                                    stream == DMA2_Stream5 ? DMA2_Stream5_IRQn : \
                                                    stream == DMA2_Stream6 ? DMA2_Stream6_IRQn : \
                                                    stream == DMA2_Stream7 ? DMA2_Stream7_IRQn : 0; \

#define DMA_MBURST_CONFIG_VAL_TO_BEATS(configVal)           configVal == DMA_Burst1 ? 1 : \
                                                            configVal == DMA_Burst4 ? 4 : \
                                                            configVal == DMA_Burst8 ? 8 : \
                                                            configVal == DMA_Burst16 ? 16 : 1

/* ------------ ERROR CODES ------------ */

typedef enum {
    DMA_ErrOK,
    DMA_ErrPerDisabled,
    DMA_ErrIncompatibleBufferMode,
    DMA_ErrInvalidDataLength
} DMA_Error_e;

/* ------------ CONFIG STRUCTURES ------------ */

#include "stm32f4xx.h"

typedef enum {
    DMA_Channel0,
    DMA_Channel1,
    DMA_Channel2,
    DMA_Channel3,
    DMA_Channel4,
    DMA_Channel5,
    DMA_Channel6,
    DMA_Channel7,
} DMA_Channel_e;

typedef enum {
    DMA_Burst1,
    DMA_Burst4,
    DMA_Burst8,
    DMA_Burst16,
} DMA_BurstConfig_e;

typedef enum {
    DMA_PriorityLow,
    DMA_PriorityMedium,
    DMA_PriorityHigh,
    DMA_PriorityVeryHigh,
} DMA_StreamPriority_e;

typedef enum {
    DMA_PINCOSPSIZE,
    DMA_PINCOS4Bytes,
} DMA_PINCOS_e;

typedef enum {
    DMA_DataSizeByte,
    DMA_DataSizeHalfWord,
    DMA_DataSizeWord,
} DMA_DataSize_e;

typedef enum {
    DMA_DirPerToMem,
    DMA_DirMemToPer,
    DMA_DirMemToMem,
} DMA_Direction_e;

typedef enum {
    DMA_FlowController,
    DMA_PeripheralFlowController
} DMA_FlowController_e;

typedef enum {
    DMA_FIFO1_4,
    DMA_FIFO1_2,
    DMA_FIFO3_4,
    DMA_FIFOFull,
} DMA_FIFOThres_e;

typedef enum {
    DMA_InterruptDirectModeError,
    DMA_InterruptTransferError,
    DMA_InterruptHalfTransfer,
    DMA_InterruptTransferComplete,
    DMA_InterruptFIFOOverrun,
} DMA_Interrupt_e;

typedef enum {
    DMA_FlagHalfTransferComplete,
    DMA_FlagTransferComplete,
    DMA_FlagTransferError,
    DMA_FlagFIFOOverrun,
    DMA_FlagDirectModeError,
} DMA_Flag_e;

typedef struct {
    DMA_Channel_e Channel;
    DMA_BurstConfig_e MemoryBurstConfig;
    DMA_BurstConfig_e PeripheralBurstConfig;
    uint8_t DoubleBufferMode;
    DMA_StreamPriority_e Priority;
    DMA_PINCOS_e PeripheralIncrementOffsetSize;
    DMA_DataSize_e MemoryDataSize;
    DMA_DataSize_e PeripheralDataSize;
    uint8_t MemoryIncrementMode;
    uint8_t PeripheralIncrementMode;
    uint8_t CircularMode;
    uint8_t DirectModeDisabled;
    DMA_FIFOThres_e FIFOThreshold;
    DMA_Direction_e Direction;
    DMA_FlowController_e FlowController;
} DMA_Config_t;

typedef struct {
    DMA_Stream_TypeDef *pDMAxStream;
    DMA_Config_t Config;
} DMA_Handle_t;

/* ------------ METHODS ------------ */

/*
 * Peripheral control
 */
void DMA_PeriClockControl(DMA_Handle_t *pDMAHandle, uint8_t Enable);
void DMA_PeripheralControl(DMA_Handle_t *pDMAHandle, uint8_t Enable);

/*
 * Init & De-Init
 */
void DMA_Init(DMA_Handle_t *pDMAHandle);
void DMA_DeInit(DMA_Handle_t *pDMAHandle);

/*
 * Status information
 */
uint16_t DMA_GetRemainingTransferCount(DMA_Handle_t *pDMAHandle);
uint8_t DMA_IsTransferActive(DMA_Handle_t *pDMAHandle);
uint32_t *DMA_GetCurrentBuffer(DMA_Handle_t *pDMAHandle);

/*
 * Transfers
 */
DMA_Error_e DMA_ConfigureSingleBufferTransfer(DMA_Handle_t *pDMAHandle, uint32_t PerAddr, uint32_t Mem0Addr, uint16_t Len);
DMA_Error_e DMA_ConfigureDoubleBufferTransfer(DMA_Handle_t *pDMAHandle, uint32_t PerAddr, uint32_t Mem0Addr, uint32_t Mem1Addr, uint16_t Len);

void DMA_StartTransaction(DMA_Handle_t *pDMAHandle);
void DMA_StopTransaction(DMA_Handle_t *pDMAHandle);

/*
 * Interrupts
 */
void DMA_EnableInterrupts(DMA_Handle_t *pDMAHandle, DMA_Interrupt_e *EnabledInterrupts, uint8_t Len);
void DMA_DisableInterrupts(DMA_Handle_t *pDMAHandle, DMA_Interrupt_e *DisabledInterrupts, uint8_t Len);

void DMA_IRQEnable(DMA_Handle_t *pDMAHandle, uint8_t IRQPriority);
void DMA_IRQDisable(DMA_Handle_t *pDMAHandle);

void DMA_IRQHanding(DMA_Handle_t *pDMAHandle);

/*
 * Other methods
 */
void DMA_ClearInterruptFlags(DMA_Handle_t *pDMAHandle);
void DMA_ApplicationCallback(DMA_Handle_t *pDMAHandle, DMA_Flag_e Flag);

#endif //DMA_DRIVER_H
