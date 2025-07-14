//
// Created by Kok on 6/27/25.
//

#include "clock_driver.h"

#include "stm32f4xx.h"

uint32_t CLOCK_GetSysClockHz() {
    CLOCK_SysClockSrc_e src = (RCC->CFGR >> RCC_CFGR_SWS_Pos) & 0x03;
    if (src == CLOCK_SysClockHSI) return 16000000;
    else if (src == CLOCK_SysClockHSE) return HSE_VALUE;
    else if (src == CLOCK_SysClockPLL) return CLOCK_GetPLLSysClockHz();

    return 0;
}

uint32_t CLOCK_GetPLLSysClockHz() {
    CLOCK_PLLSrc_e src = (RCC->PLLCFGR >> RCC_PLLCFGR_PLLSRC_Pos) & 0x01;
    uint32_t inputRaw = src == CLOCK_PLLClockHSI ? 16000000 : HSE_VALUE;

    uint32_t inputDiv = (RCC->PLLCFGR >> RCC_PLLCFGR_PLLM_Pos) & 0x3F;
    uint32_t multFactor = (RCC->PLLCFGR >> RCC_PLLCFGR_PLLN_Pos) & 0x1FF;
    uint32_t sysClkDiv = CLOCK_PLL_SYSCLK_PRESC_VALUE_TO_DIVISOR(((RCC->PLLCFGR >> RCC_PLLCFGR_PLLP_Pos) & 0x03));

    return ((inputRaw / inputDiv) * multFactor) / sysClkDiv;
}

uint32_t CLOCK_GetHclkHz() {
    uint32_t sysClk = CLOCK_GetSysClockHz();
    uint32_t ahbPrescaler = CLOCK_AHB_PRESC_VALUE_TO_DIVISOR(((RCC->CFGR >> RCC_CFGR_HPRE_Pos) & 0xF));
    return sysClk / ahbPrescaler;
}

uint32_t CLOCK_GetApb1Hz() {
    uint32_t hclk = CLOCK_GetHclkHz();
    uint32_t apb1Prescaler = CLOCK_APB_PRESC_VALUE_TO_DIVISOR(((RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x07));
    return hclk / apb1Prescaler;
}

uint32_t CLOCK_GetApb1TimerHz() {
    uint32_t apb1 = CLOCK_GetApb1Hz();
    uint32_t apb1Prescaler = CLOCK_APB_PRESC_VALUE_TO_DIVISOR(((RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x07));
    return apb1Prescaler == 1 ? apb1 : apb1 * 2;
}

uint32_t CLOCK_GetApb2Hz() {
    uint32_t hclk = CLOCK_GetHclkHz();
    uint32_t apb2Prescaler = CLOCK_APB_PRESC_VALUE_TO_DIVISOR(((RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x07));
    return hclk / apb2Prescaler;
}

uint32_t CLOCK_GetApb2TimerHz() {
    uint32_t apb2 = CLOCK_GetApb2Hz();
    uint32_t apb2Prescaler = CLOCK_APB_PRESC_VALUE_TO_DIVISOR(((RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x07));
    return apb2Prescaler == 1 ? apb2 : apb2 * 2;
}

/**
 * @brief Enabled the desired clock source
 * @param CLOCK_Src Desired source
 */
void CLOCK_Enable(CLOCK_Src_e CLOCK_Src) {
    if (CLOCK_Src == CLOCK_SrcHSI) {
        RCC->CR |= (1 << RCC_CR_HSION_Pos);
        // Wait for the source to stabilize
        while (!(RCC->CR & (1 << RCC_CR_HSIRDY_Pos)));
    } else if (CLOCK_Src == CLOCK_SrcHSE || CLOCK_Src == CLOCK_SrcHSEBypass) {
        RCC->CR &=~ (1 << RCC_CR_HSEBYP_Pos);
        if (CLOCK_Src == CLOCK_SrcHSEBypass) RCC->CR |= (1 << RCC_CR_HSEBYP_Pos);
        RCC->CR |= (1 << RCC_CR_HSEON_Pos);
        // Wait for the source to stabilize
        while (!(RCC->CR & (1 << RCC_CR_HSERDY_Pos)));
    } else if (CLOCK_Src == CLOCK_SrcPLL) {
        RCC->CR |= (1 << RCC_CR_PLLON_Pos);
        // Wait for the source to stabilize
        while (!(RCC->CR & (1 << RCC_CR_PLLRDY_Pos)));
    } else if (CLOCK_Src == CLOCK_SrcLSE || CLOCK_Src == CLOCK_SrcLSEBypass) {
        RCC->BDCR &=~ (1 << RCC_BDCR_LSEBYP_Pos);
        if (CLOCK_Src == CLOCK_SrcLSEBypass) RCC->BDCR |= (1 << RCC_BDCR_LSEBYP_Pos);
        RCC->BDCR |= (1 << RCC_BDCR_LSEON_Pos);
        // Wait for the source to stabilize
        while (!(RCC->BDCR & (1 << RCC_BDCR_LSERDY_Pos)));
    }
}

/**
 * @brief Disables the desired clock source
 * @param CLOCK_Src The desired clock source
 */
void CLOCK_Disable(CLOCK_Src_e CLOCK_Src) {
    if (CLOCK_Src == CLOCK_SrcHSI) RCC->CR &=~ (1 << RCC_CR_HSION_Pos);
    else if (CLOCK_Src == CLOCK_SrcHSE || CLOCK_Src == CLOCK_SrcHSEBypass) RCC->CR &=~ (1 << RCC_CR_HSEON_Pos);
    else if (CLOCK_Src == CLOCK_SrcPLL) RCC->CR &=~ (1 << RCC_CR_PLLON_Pos);
    else if (CLOCK_Src == CLOCK_SrcLSE || CLOCK_Src == CLOCK_SrcLSEBypass) RCC->BDCR &=~ (1 << RCC_BDCR_LSEON_Pos);
}


/**
 * @brief Selects which source to be used for system clock
 * @param CLOCK_Src The desired SysClk source
 */
void CLOCK_SelectSysClock(CLOCK_SysClockSrc_e CLOCK_Src) {
    RCC->CFGR &=~ (0x03 << RCC_CFGR_SW_Pos);
    RCC->CFGR |= (CLOCK_Src << RCC_CFGR_SW_Pos);
    while ((((RCC->CFGR >> RCC_CFGR_SWS_Pos) & 0x3) != CLOCK_Src));
}

/**
 * @brief Configures the AHB1 Bus prescaler
 * @param Prescaler The desired prescaler
 */
void CLOCK_SetAHBBusPrescaler(CLOCK_AHBPrescaler_e Prescaler) {
    RCC->CFGR |= (Prescaler << RCC_CFGR_HPRE_Pos);
}

/**
 * @brief Configures the APB Bus prescaler
 * @param CLOCK_APBBus APB1 or APB2
 * @param Prescaler The desired prescaler
 */
void CLOCK_SetAPBBusPrescaler(CLOCK_APBBus_e CLOCK_APBBus, CLOCK_APBPrescaler_e Prescaler) {
    if (CLOCK_APBBus == CLOCK_BusAPB1) {
        RCC->CFGR |= (Prescaler << RCC_CFGR_PPRE1_Pos);
    } else {
        RCC->CFGR |= (Prescaler << RCC_CFGR_PPRE2_Pos);
    }
}

/**
 * @brief Sets the PLL input clock
 * @param CLOCK_Src THe desired clock source
 */
void CLOCK_SelectPLLClock(CLOCK_PLLSrc_e CLOCK_Src) {
    RCC->CR |= (CLOCK_Src << RCC_PLLCFGR_PLLSRC_Pos);
}

/**
 * @brief Configures the PLL with the desired factors
 * @param MultFactor Main PLL multiplication factor
 * @param InDivFactor Division factor for the main PLL and audio PLL input clock
 * @param SysClkDivFactor Main PLL division factor for main system clock
 * @return CLOCK_ErrOK on success
 */
CLOCK_Error_e CLOCK_SetPLLFactors(uint32_t InDivFactor, uint32_t MultFactor, CLOCK_PLLSysClkPrescaler_e SysClkDivFactor) {
    if (InDivFactor < CLOCK_PLL_MIN_IN_DIV_FACTOR || InDivFactor > CLOCK_PLL_MAX_IN_DIV_FACTOR) return CLOCK_ErrInvalidInDivFactor;
    if (MultFactor < CLOCK_PLL_MIN_MULT_FACTOR || MultFactor > CLOCK_PLL_MAX_MULT_FACTOR) return CLOCK_ErrInvalidMultFactor;

    // Clear old values first
    RCC->PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR &= ~(0x3F  << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR &= ~(0x3   << RCC_PLLCFGR_PLLP_Pos);

    RCC->PLLCFGR |= (MultFactor << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (InDivFactor << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (SysClkDivFactor << RCC_PLLCFGR_PLLP_Pos);
    
    return CLOCK_ErrOK;
}

/**
 * @brief Resets the CPU and the Peripherals
 */
void CLOCK_ResetSystem() {
    NVIC_SystemReset();
}

/**
 * @brief Resets RTC registers, LSE oscillator configuration and the RCC_BDCR
 */
void CLOCK_ResetBackupDomain() {
    RCC->BDCR |= (1 << RCC_BDCR_BDRST_Pos);
    RCC->BDCR &=~ (1 << RCC_BDCR_BDRST_Pos);
}

/**
 * @brief Enables / Disables the clock security system (CSS)
 * @param Enabled If it should be enabled
 */
void CLOCK_SetSecuritySystem(uint8_t Enabled) {
    if (Enabled) {
        RCC->CR |= (1 << RCC_CR_CSSON_Pos);
    } else {
        RCC->CR &=~ (1 << RCC_CR_CSSON_Pos);
    }
}

/**
* @brief Selects how the clock frequency of all the timers connected to APB1
         and APB2 domain is prescaled.
 * @param Option Prescaler option
 */
void CLOCK_SetTimersClockPrescalers(CLOCK_TimClkPrescOption_e Option) {
    if (Option == CLOCK_TimClkPrescOption1) {
        RCC->DCKCFGR &=~ (1 << RCC_DCKCFGR_TIMPRE_Pos);
    } else {
        RCC->DCKCFGR |= (1 << RCC_DCKCFGR_TIMPRE_Pos);
    }
}




