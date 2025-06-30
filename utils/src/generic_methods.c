//
// Created by Kok on 6/30/25.
//

#include "generic_methods.h"

#include <commons.h>
#include <systick_driver.h>

void Generic_InitTimer() {
    SYSTICK_Config_t systickConfig = {
        .ClockSource = SYSTICK_SrcAHB8,
        .InterruptsStatus = SYSTICK_InterruptsDisabled,
        .TimerPeriodMS = 1000
    };

    SYSTICK_Init(systickConfig);
}

void Generic_Delay(uint32_t ms) {
    while (ms > 0) {
        uint32_t delayChunk = (ms > 1000) ? 1000 : ms;
        SYSTICK_ConfigureForMs(delayChunk);
        SYSTICK_ClearCounterValue();
        SYSTICK_CounterControl(ENABLE);
        while (!SYSTICK_CheckCountFlag());
        SYSTICK_CounterControl(DISABLE);
        ms -= delayChunk;
    }
}
