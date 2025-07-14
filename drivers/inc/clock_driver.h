//
// Created by Kok on 6/27/25.
//

#ifndef CLOCK_DRIVER_H
#define CLOCK_DRIVER_H

#include <stdint.h>

/* ------------ MACROS ------------ */

#define HSE_VALUE                           0           // Set according to the frequency of the used HSE source

#define CLOCK_PLL_MIN_MULT_FACTOR           50
#define CLOCK_PLL_MAX_MULT_FACTOR           432

#define CLOCK_PLL_MIN_IN_DIV_FACTOR         2
#define CLOCK_PLL_MAX_IN_DIV_FACTOR         63

#define CLOCK_AHB_PRESC_VALUE_TO_DIVISOR(val)   (val == CLOCK_AHBPresc1     ? 1 :\
                                                 val == CLOCK_AHBPresc2     ? 2 :\
                                                 val == CLOCK_AHBPresc4     ? 4 :\
                                                 val == CLOCK_AHBPresc8     ? 8 :\
                                                 val == CLOCK_AHBPresc16    ? 16 :\
                                                 val == CLOCK_AHBPresc64    ? 64 :\
                                                 val == CLOCK_AHBPresc128   ? 128 :\
                                                 val == CLOCK_AHBPresc256   ? 256 :\
                                                 val == CLOCK_AHBPresc512   ? 512 :\
                                                 1)

#define CLOCK_APB_PRESC_VALUE_TO_DIVISOR(val)   (val == CLOCK_APBPresc1     ? 1 :\
                                                 val == CLOCK_APBPresc2     ? 2 :\
                                                 val == CLOCK_APBPresc4     ? 4 :\
                                                 val == CLOCK_APBPresc8     ? 8 :\
                                                 val == CLOCK_APBPresc16    ? 16 :\
                                                 1)

#define CLOCK_PLL_SYSCLK_PRESC_VALUE_TO_DIVISOR(val)        (val == CLOCK_PLLSysClkPresc2       ? 2 :\
                                                             val == CLOCK_PLLSysClkPresc4       ? 4 :\
                                                             val == CLOCK_PLLSysClkPresc6       ? 6 :\
                                                             val == CLOCK_PLLSysClkPresc8       ? 8 :\
                                                             2)

/* ------------ ERROR CODES ------------ */

typedef enum {
    CLOCK_ErrOK,
    CLOCK_ErrInvalidMultFactor,
    CLOCK_ErrInvalidInDivFactor,
    CLOCK_ErrInvalidSysClkDivFactor
} CLOCK_Error_e;

/* ------------ CONFIG STRUCTURES ------------ */

typedef enum {
    CLOCK_SrcHSI,
    CLOCK_SrcHSE,
    CLOCK_SrcHSEBypass,
    CLOCK_SrcPLL,
    CLOCK_SrcLSE,
    CLOCK_SrcLSEBypass,
} CLOCK_Src_e;

typedef enum {
    CLOCK_SysClockHSI,
    CLOCK_SysClockHSE,
    CLOCK_SysClockPLL,
} CLOCK_SysClockSrc_e;

typedef enum {
    CLOCK_PLLClockHSI,
    CLOCK_PLLClockHSE,
} CLOCK_PLLSrc_e;

typedef enum {
    CLOCK_PLLSysClkPresc2,
    CLOCK_PLLSysClkPresc4,
    CLOCK_PLLSysClkPresc6,
    CLOCK_PLLSysClkPresc8,
} CLOCK_PLLSysClkPrescaler_e;

typedef enum {
    CLOCK_BusAPB1,
    CLOCK_BusAPB2,
} CLOCK_APBBus_e;

typedef enum {
    CLOCK_APBPresc1 = 0b000,
    CLOCK_APBPresc2 = 0b100,
    CLOCK_APBPresc4 = 0b101,
    CLOCK_APBPresc8 = 0b110,
    CLOCK_APBPresc16 = 0b111,
} CLOCK_APBPrescaler_e;

typedef enum {
    CLOCK_AHBPresc1         = 0b0000,
    CLOCK_AHBPresc2         = 0b1000,
    CLOCK_AHBPresc4         = 0b1001,
    CLOCK_AHBPresc8         = 0b1010,
    CLOCK_AHBPresc16        = 0b1011,
    CLOCK_AHBPresc64        = 0b1100,
    CLOCK_AHBPresc128       = 0b1101,
    CLOCK_AHBPresc256       = 0b1110,
    CLOCK_AHBPresc512       = 0b1111,
} CLOCK_AHBPrescaler_e;

typedef enum {
    CLOCK_TimClkPrescOption1,           // If the APB prescaler is configured to a division factor of 1, TIMxCLK = HCKL . Otherwise, the timer clock frequencies are set to twice to the frequency of the APB domain to which the timers are connected: TIMxCLK = 2xPCLKx.
    CLOCK_TimClkPrescOption2,           // If the APB prescaler is configured to a division factor of 1 or 2, TIMxCLK = HCKL. Otherwise, the timer clock frequencies are set to four times to the frequency of the APB domain to which the timers are connected: TIMxCLK = 4xPCLKx.
} CLOCK_TimClkPrescOption_e;

/* ------------ METHODS ------------ */

/*
 * Clock information
 */
uint32_t CLOCK_GetSysClockHz();
uint32_t CLOCK_GetPLLSysClockHz();
uint32_t CLOCK_GetHclkHz();
uint32_t CLOCK_GetApb1Hz();
uint32_t CLOCK_GetApb1TimerHz();
uint32_t CLOCK_GetApb2Hz();
uint32_t CLOCK_GetApb2TimerHz();

/*
 * Clock enable/disable
 */
void CLOCK_Enable(CLOCK_Src_e CLOCK_Src);
void CLOCK_Disable(CLOCK_Src_e CLOCK_Src);

/*
 * Clock selection and bus config
 */
void CLOCK_SelectSysClock(CLOCK_SysClockSrc_e CLOCK_Src);
void CLOCK_SetAHBBusPrescaler(CLOCK_AHBPrescaler_e Prescaler);
void CLOCK_SetAPBBusPrescaler(CLOCK_APBBus_e CLOCK_APBBus, CLOCK_APBPrescaler_e Prescaler);

/*
 * PLL
 */
void CLOCK_SelectPLLClock(CLOCK_PLLSrc_e CLOCK_Src);
CLOCK_Error_e CLOCK_SetPLLFactors(uint32_t InDivFactor, uint32_t MultFactor, CLOCK_PLLSysClkPrescaler_e SysClkDivFactor);

/*
 * Reset
 */
void CLOCK_ResetSystem();
void CLOCK_ResetBackupDomain();

/*
 * Other methods
 */
void CLOCK_SetSecuritySystem(uint8_t Enabled);
void CLOCK_SetTimersClockPrescalers(CLOCK_TimClkPrescOption_e Option);

#endif //CLOCK_DRIVER_H
