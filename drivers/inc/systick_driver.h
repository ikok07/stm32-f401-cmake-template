//
// Created by Kok on 6/30/25.
//

#ifndef SYSTICK_DRIVER_H
#define SYSTICK_DRIVER_H

#include <stdint.h>

/* ------------ ERROR FLAGS ------------ */

typedef enum {
    SYSTICK_ErrOK,
    SYSTICK_ErrInvalidReloadValue,
    SYSTICK_ErrCounterDisabled
} SYSTICK_Error_e;

/* ------------ CONFIG STRUCTURES ------------ */

typedef enum {
    SYSTICK_SrcAHB8,                // AHB / 8
    SYSTICK_SrcAHB
} SYSTICK_Src_e;

typedef enum {
    SYSTICK_InterruptsDisabled,
    SYSTICK_InterruptsEnabled,
} SYSTICK_InterruptsStatus_e;

typedef struct {
    SYSTICK_Src_e ClockSource;
    SYSTICK_InterruptsStatus_e InterruptsStatus;
    uint32_t TimerPeriodMS;
    uint8_t TickCounterEnabled;
} SYSTICK_Config_t;

/* ------------ METHODS ------------ */

/*
 * Initialization
 */
SYSTICK_Error_e SYSTICK_Init(SYSTICK_Config_t Config);

/*
 * Ticks controls
 */
SYSTICK_Error_e SYSTICK_GetCurrTicks(uint32_t *ticks);
void SYSTICK_ClearTickCounter();

/*
 * Counter
 */
void SYSTICK_CounterControl(uint8_t Enabled);
uint8_t SYSTICK_CheckCounterEnabled();
uint8_t SYSTICK_CheckCountFlag();

/*
 * Other methods
 */
SYSTICK_Error_e SYSTICK_SetReloadValue(uint32_t Value);
void SYSTICK_ClearCounterValue();
uint32_t SYSTICK_GetCounterValue();
SYSTICK_Error_e SYSTICK_ConfigureForMs(uint32_t MS);


#endif //SYSTICK_DRIVER_H
