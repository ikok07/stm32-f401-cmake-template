//
// Created by Kok on 6/30/25.
//

#include "generic_methods.h"

#include <commons.h>
#include <systick_driver.h>


void Generic_InitSysTick() {
    SYSTICK_Config_t systickConfig = {
        .ClockSource = SYSTICK_SrcAHB8,
        .InterruptsStatus = SYSTICK_InterruptsEnabled,
        .TimerPeriodMS = 1,
        .TickCounterEnabled = ENABLE
    };

    SYSTICK_Init(systickConfig);
}

/**
 * @note The maximum allowed delay is 1 minute
 * @param ms The desired milliseconds
 */
SYSTICK_Error_e Generic_Delay(uint32_t ms) {
    // Max 1 minute delay
    if (ms > 60000) ms = 60000;

    uint32_t startTicks, currTicks;

    SYSTICK_Error_e err = SYSTICK_GetCurrTicks(&startTicks);
    if (err != SYSTICK_ErrOK) return err;
    if ((err = SYSTICK_GetCurrTicks(&currTicks)) != SYSTICK_ErrOK) return err;

    while (currTicks - startTicks < ms) {
        if ((err = SYSTICK_GetCurrTicks(&currTicks)) != SYSTICK_ErrOK) return err;
    };

    return SYSTICK_ErrOK;
}

