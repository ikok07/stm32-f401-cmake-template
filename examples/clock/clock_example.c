//
// Created by Kok on 6/27/25.
//

#include "clock_driver.h"

void SystemInit() {
    // Setup for 84MHz

    CLOCK_SelectSysClock(CLOCK_SysClockHSI);
    CLOCK_Disable(CLOCK_SrcPLL);
    CLOCK_SetPLLFactors(16, 336, CLOCK_PLLSysClkPresc4);
    CLOCK_Enable(CLOCK_SrcPLL);
    CLOCK_SelectSysClock(CLOCK_SysClockPLL);
    CLOCK_SetAHBBusPrescaler(CLOCK_AHBPresc1);
    CLOCK_SetAPBBusPrescaler(CLOCK_BusAPB1, CLOCK_APBPresc2);
    CLOCK_SetAPBBusPrescaler(CLOCK_BusAPB2, CLOCK_APBPresc1);

    uint32_t hclk = CLOCK_GetHclkHz();
    uint32_t apb1 = CLOCK_GetApb1Hz();
    uint32_t apb1Timer = CLOCK_GetApb1TimerHz();
    uint32_t apb2 = CLOCK_GetApb2Hz();
    uint32_t apb2Timer = CLOCK_GetApb2TimerHz();
    (void)hclk;
    (void)apb1;
    (void)apb1Timer;
    (void)apb2;
    (void)apb2Timer;
}
