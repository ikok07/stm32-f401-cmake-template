//
// Created by Kok on 6/28/25.
//

#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include "stm32f4xx.h"

#define FLASH_CR_UNLOCK_KEY1            0x45670123
#define FLASH_CR_UNLOCK_KEY2            0xCDEF89AB

#define FLASH_OPTCR_UNLOCK_KEY1         0x08192A3B
#define FLASH_OPTCR_UNLOCK_KEY2         0x4C5D6E7F

typedef enum {
    FLASH_WaitState0,
    FLASH_WaitState1,
    FLASH_WaitState2,
    FLASH_WaitState3,
    FLASH_WaitState4,
    FLASH_WaitState5,
} FLASH_WaitState_e;

typedef enum {
    FLASH_MemoryReady,
    FLASH_MemoryBusy,
} FLASH_MemoryState;

typedef enum {
    FLASH_ParallelismX8,
    FLASH_ParallelismX16,
    FLASH_ParallelismX32,
    FLASH_ParallelismX64,
} FLASH_Parallelism_e;

/*
 * Acceleration features
 */
void FLASH_SetLatency(FLASH_WaitState_e WaitState);

/*
 * Controls
 */
void FLASH_SetParallelism(FLASH_Parallelism_e Parallelism);

/*
 * State
 */
FLASH_MemoryState FLASH_GetCurrentState();

/*
 * Lock / unlock registers
 */
void FLASH_UnlockCR();
void FLASH_LockCR();
void FLASH_UnlockOPTCR();
void FLASH_LockOPTCR();


#endif //FLASH_DRIVER_H
