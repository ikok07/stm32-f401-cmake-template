//
// Created by Kok on 6/30/25.
//

#include "systick_driver.h"

#include <clock_driver.h>

#include "stm32f4xx.h"
#include "core_cm4.h"

volatile uint32_t tick_count = 0;

SYSTICK_Error_e SYSTICK_Init(SYSTICK_Config_t Config) {
    // Reload value
    uint32_t clk = Config.ClockSource == SYSTICK_SrcAHB ? CLOCK_GetHclkHz() : CLOCK_GetHclkHz() / 8;
    SYSTICK_Error_e err = SYSTICK_SetReloadValue((clk / 1000) * Config.TimerPeriodMS - 1);
    if (err != SYSTICK_ErrOK) return err;

    // Clear the counter and control register
    SYSTICK_ClearCounterValue();
    SysTick->CTRL = 0x00;

    // Clock source
    SysTick->CTRL |= (Config.ClockSource << SysTick_CTRL_CLKSOURCE_Pos);

    // Interrupt state
    SysTick->CTRL |= (Config.InterruptsStatus << SysTick_CTRL_TICKINT_Pos);

    if (Config.TickCounterEnabled) SYSTICK_CounterControl(ENABLE);;

    return SYSTICK_ErrOK;
}

SYSTICK_Error_e SYSTICK_GetCurrTicks(uint32_t *ticks) {
    if (!SYSTICK_CheckCounterEnabled()) return SYSTICK_ErrCounterDisabled;
    *ticks = tick_count;
    return SYSTICK_ErrOK;;
}

void SYSTICK_ClearTickCounter() {
    tick_count = 0;
}

/**
 * @brief Controls if the systick counter should be enabled or disabled
 * @param Enabled If the counter should be enabled
 */
void SYSTICK_CounterControl(uint8_t Enabled) {
    if (Enabled) {
        SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos);
    } else {
        SysTick->CTRL &=~ (1 << SysTick_CTRL_ENABLE_Pos);
    }
}

uint8_t SYSTICK_CheckCounterEnabled() {
    return SysTick->CTRL & (1 << SysTick_CTRL_ENABLE_Pos);
}

/**
 * @return Returns 1 if timer counted to 0 since last time this flag was checked
 */
uint8_t SYSTICK_CheckCountFlag() {
    return (SysTick->CTRL & (1 << SysTick_CTRL_COUNTFLAG_Pos)) ? 1 : 0;
}

/**
 * @brief Sets the systick reload value
 * @param Value The value to be set in the reload register
 */
SYSTICK_Error_e SYSTICK_SetReloadValue(uint32_t Value) {
    if (Value > 0xFFFFFF) {
        return SYSTICK_ErrInvalidReloadValue;
    }

    SysTick->LOAD = Value & 0xFFFFFF;

    return SYSTICK_ErrOK;
}


/**
 * Clears the current systick counter value and the COUNTFLAG
 */
void SYSTICK_ClearCounterValue() {
    SysTick->VAL = 0x01;
}

/**
 * @brief Gets the current systick counter value
 * @return The current systick counter value
 */
uint32_t SYSTICK_GetCounterValue() {
    return SysTick->VAL;
}

/**
 * @brief Automatically calculates the reload value for the desired clock period
 * @param MS The desired clock period in milliseconds
 * @param CpuClkHz The CPU frequency
 */
SYSTICK_Error_e SYSTICK_ConfigureForMs(uint32_t MS) {
    uint32_t clk = ((SysTick->CTRL >> SysTick_CTRL_CLKSOURCE_Pos) & 0x01) == SYSTICK_SrcAHB ? CLOCK_GetHclkHz() : CLOCK_GetHclkHz() / 8;
    return SYSTICK_SetReloadValue((clk / 1000) * MS - 1);
}

/**
 * Interrupt handler
 */
void SysTick_Handler() {
    tick_count++;
}