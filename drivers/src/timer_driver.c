//
// Created by Kok on 6/29/25.
//

#include "timer_driver.h"

#include <clock_driver.h>

static uint8_t calc_dtg_value(TIM_DeadTimeClkDivision_e clkDivision, uint32_t dead_time);

/**
 * @brief Enables or disables peripheral clock for the given timer
 * @param pTIMx Base address of the timer
 * @param Enabled If the peripheral is enabled or disabled (1 or 0)
 */
void TIM_PeriClockControl(TIM_TypeDef *pTIMx, uint8_t Enabled) {
    if (Enabled) {
        if (pTIMx == TIM1) TIM_PCLK_EN(1, RCC_APB2ENR_TIM1EN_Pos);
        else if (pTIMx == TIM2) TIM_PCLK_EN(2, RCC_APB1ENR_TIM2EN_Pos);
        else if (pTIMx == TIM3) TIM_PCLK_EN(3, RCC_APB1ENR_TIM3EN_Pos);
        else if (pTIMx == TIM4) TIM_PCLK_EN(4, RCC_APB1ENR_TIM4EN_Pos);
        else if (pTIMx == TIM5) TIM_PCLK_EN(5, RCC_APB1ENR_TIM5EN_Pos);
        else if (pTIMx == TIM9) TIM_PCLK_EN(9, RCC_APB2ENR_TIM9EN_Pos);
        else if (pTIMx == TIM10) TIM_PCLK_EN(10, RCC_APB2ENR_TIM10EN_Pos);
        else if (pTIMx == TIM11) TIM_PCLK_EN(11, RCC_APB2ENR_TIM11EN_Pos);
    } else {
        if (pTIMx == TIM1) TIM_PCLK_DI(1, RCC_APB2ENR_TIM1EN_Pos);
        else if (pTIMx == TIM2) TIM_PCLK_DI(2, RCC_APB1ENR_TIM2EN_Pos);
        else if (pTIMx == TIM3) TIM_PCLK_DI(3, RCC_APB1ENR_TIM3EN_Pos);
        else if (pTIMx == TIM4) TIM_PCLK_DI(4, RCC_APB1ENR_TIM4EN_Pos);
        else if (pTIMx == TIM5) TIM_PCLK_DI(5, RCC_APB1ENR_TIM5EN_Pos);
        else if (pTIMx == TIM9) TIM_PCLK_DI(9, RCC_APB2ENR_TIM9EN_Pos);
        else if (pTIMx == TIM10) TIM_PCLK_DI(10, RCC_APB2ENR_TIM10EN_Pos);
        else if (pTIMx == TIM11) TIM_PCLK_DI(11, RCC_APB2ENR_TIM11EN_Pos);
    }
}

/**
 * @brief Makes basic configuration of the desired timer
 * @param pTIMHandle timer handle
 */
void TIM_Init(TIM_Handle_t *pTIMHandle) {

    // Clear the timer registers before write
    pTIMHandle->pTIMx->CR1 &=~ (TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_OPM | TIM_CR1_URS | TIM_CR1_UDIS);

    // Auto-Reload Preload
    pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIM_Config.PreloadEnabled << TIM_CR1_ARPE_Pos);

    // Counter direction
    pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIM_Config.Direction << TIM_CR1_DIR_Pos);

    // Center-aligned mode selection
    pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIM_Config.CenterModeSelection << TIM_CR1_CMS_Pos);

    // One pulse mode
    pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIM_Config.OnePulseMode << TIM_CR1_OPM_Pos);

    // Update request source
    pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIM_Config.UpdateRequestSourceMode << TIM_CR1_URS_Pos);

    // Update event toggle
    pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIM_Config.UpdateEventToggle << TIM_CR1_UDIS_Pos);

}

/**
 * @brief Resets the timer configuration
 * @param pTIMHandle timer handle
 */
void TIM_DeInit(TIM_Handle_t *pTIMHandle) {

    // Stop the timer
    TIM_Stop(pTIMHandle);

    // Reset all timer registers
    if (pTIMHandle->pTIMx == TIM1) TIM_RESET(1, RCC_APB2RSTR_TIM1RST_Pos);
    else if (pTIMHandle->pTIMx == TIM2) TIM_RESET(2, RCC_APB1RSTR_TIM2RST_Pos);
    else if (pTIMHandle->pTIMx == TIM3) TIM_RESET(3, RCC_APB1RSTR_TIM3RST_Pos);
    else if (pTIMHandle->pTIMx == TIM4) TIM_RESET(4, RCC_APB1RSTR_TIM4RST_Pos);
    else if (pTIMHandle->pTIMx == TIM5) TIM_RESET(5, RCC_APB1RSTR_TIM5RST_Pos);
    else if (pTIMHandle->pTIMx == TIM9) TIM_RESET(9, RCC_APB2RSTR_TIM9RST_Pos);
    else if (pTIMHandle->pTIMx == TIM10) TIM_RESET(10, RCC_APB2RSTR_TIM10RST_Pos);
    else if (pTIMHandle->pTIMx == TIM11) TIM_RESET(11, RCC_APB2RSTR_TIM11RST_Pos);

    // Disable the timer clock
    TIM_PeriClockControl(pTIMHandle->pTIMx, DISABLE);
}

void TIM_Start(TIM_Handle_t *pTIMHandle) {
    pTIMHandle->pTIMx->CR1 |= (1 << TIM_CR1_CEN_Pos);
}

void TIM_Stop(TIM_Handle_t *pTIMHandle) {
    pTIMHandle->pTIMx->CR1 &=~ (1 << TIM_CR1_CEN_Pos);
}

/**
 * @brief Sets the UG bit in the EGR in order to manually trigger update event
 * @param pTIMHandle timer handle
 */
void TIM_GenerateUpdateEvent(TIM_Handle_t *pTIMHandle) {
    pTIMHandle->pTIMx->EGR |= (1 << TIM_EGR_UG_Pos);
}

/**
 * @brief Sets the CCxG bit in the EGR in order to manually trigger capture compare event for the desired channel
 * @param pTIMHandle timer handle
 */
void TIM_GenerateCaptureCompareEvent(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel) {
    if (Channel == TIM_CaptureCompareChan1) {
        pTIMHandle->pTIMx->EGR |= (1 << TIM_EGR_CC1G_Pos);
    } else if (Channel == TIM_CaptureCompareChan2) {
        pTIMHandle->pTIMx->EGR |= (1 << TIM_EGR_CC2G_Pos);
    } else if (Channel == TIM_CaptureCompareChan3) {
        pTIMHandle->pTIMx->EGR |= (1 << TIM_EGR_CC3G_Pos);
    } else if (Channel == TIM_CaptureCompareChan4) {
        pTIMHandle->pTIMx->EGR |= (1 << TIM_EGR_CC4G_Pos);
    }
}

/**
 * @brief Configures the desired timer channel in output compare mode
 * @param pTIMHandle timer handle
 * @param Channel The desired timer channel
 * @param Config Output compare configuration
 */
void TIM_ConfigureOutputCompare(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel,
                                TIM_OutputCompareConfig_t Config) {
    if (Channel <= TIM_CaptureCompareChan2) {
        if (Channel == TIM_CaptureCompareChan1) {
            // Output mode
            pTIMHandle->pTIMx->CCMR1 &=~ TIM_CCMR1_CC1S_Msk;

            // Output compare mode
            pTIMHandle->pTIMx->CCMR1 &=~ TIM_CCMR1_OC1M_Msk;
            pTIMHandle->pTIMx->CCMR1 |= (Config.Mode << TIM_CCMR1_OC1M_Pos);

            // Preload
            pTIMHandle->pTIMx->CCMR1 &=~ TIM_CCMR1_OC1PE_Msk;
            pTIMHandle->pTIMx->CCMR1 |= (Config.Preload << TIM_CCMR1_OC1PE_Pos);
        } else {
            // Output mode
            pTIMHandle->pTIMx->CCMR1 &=~ TIM_CCMR1_CC2S_Msk;

            // Output compare mode
            pTIMHandle->pTIMx->CCMR1 &=~ TIM_CCMR1_OC2M_Msk;
            pTIMHandle->pTIMx->CCMR1 |= (Config.Mode << TIM_CCMR1_OC2M_Pos);

            // Preload
            pTIMHandle->pTIMx->CCMR1 &=~ TIM_CCMR1_OC2PE_Msk;
            pTIMHandle->pTIMx->CCMR1 |= (Config.Preload << TIM_CCMR1_OC2PE_Pos);
        }
    } else {
        if (Channel == TIM_CaptureCompareChan3) {
            // Output mode
            pTIMHandle->pTIMx->CCMR2 &=~ TIM_CCMR2_CC3S_Msk;

            // Output compare mode
            pTIMHandle->pTIMx->CCMR2 &=~ TIM_CCMR2_OC3M_Msk;
            pTIMHandle->pTIMx->CCMR2 |= (Config.Mode << TIM_CCMR2_OC3M_Pos);

            // Preload
            pTIMHandle->pTIMx->CCMR2 &=~ TIM_CCMR2_OC3PE_Msk;
            pTIMHandle->pTIMx->CCMR2 |= (Config.Preload << TIM_CCMR2_OC3PE_Pos);
        } else {
            // Output mode
            pTIMHandle->pTIMx->CCMR2 &=~ TIM_CCMR2_CC4S_Msk;

            // Output compare mode
            pTIMHandle->pTIMx->CCMR2 &=~ TIM_CCMR2_OC4M_Msk;
            pTIMHandle->pTIMx->CCMR2 |= (Config.Mode << TIM_CCMR2_OC4M_Pos);

            // Preload
            pTIMHandle->pTIMx->CCMR2 &=~ TIM_CCMR2_OC4PE_Msk;
            pTIMHandle->pTIMx->CCMR2 |= (Config.Preload << TIM_CCMR2_OC4PE_Pos);
        }
    }
}

/**
 * @brief Enables or disables the desired output / compare channel
 * @param pTIMHandle timer handle
 * @param Channel the desired channel
 * @param Enabled If the channel should be enabled or disabled
 */
void TIM_CaptureCompareChannelControl(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel, uint8_t Enabled) {
    if (Enabled) TIM_CC_EN(pTIMHandle->pTIMx->CCER, Channel);
    else TIM_CC_DI(pTIMHandle->pTIMx->CCER, Channel);
}

/**
 * @brief Selects the active state's level
 * @param pTIMHandle timer handle
 * @param Channel the desired channel
 * @param Polarity The desired channel polarity
 */
void TIM_CaptureCompareChannelPolarity(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel, TIM_CaptureComparePolarity_e Polarity) {
    TIM_CC_POL_SET(pTIMHandle->pTIMx->CCER, Channel, Polarity);
}

void TIM_SetPrescaler(TIM_Handle_t *pTIMHandle, uint16_t Prescaler) {
    pTIMHandle->pTIMx->PSC = Prescaler;
}

void TIM_SetAutoReload(TIM_Handle_t *pTIMHandle, uint16_t AutoReloadValue) {
    pTIMHandle->pTIMx->ARR = AutoReloadValue;
}

void TIM_SetCounter(TIM_Handle_t *pTIMHandle, uint16_t Value) {
    pTIMHandle->pTIMx->CNT = Value;
}

uint32_t TIM_GetCounter(TIM_Handle_t *pTIMHandle) {
    return pTIMHandle->pTIMx->CNT;
}

void TIM_SetCompareValue(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel, uint16_t CompareValue) {
    if (Channel == TIM_CaptureCompareChan1) pTIMHandle->pTIMx->CCR1 = CompareValue;
    if (Channel == TIM_CaptureCompareChan2) pTIMHandle->pTIMx->CCR2 = CompareValue;
    if (Channel == TIM_CaptureCompareChan3) pTIMHandle->pTIMx->CCR3 = CompareValue;
    if (Channel == TIM_CaptureCompareChan4) pTIMHandle->pTIMx->CCR4 = CompareValue;
}

uint32_t TIM_GetCompareValue(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel) {
    if (Channel == TIM_CaptureCompareChan1) return pTIMHandle->pTIMx->CCR1;
    if (Channel == TIM_CaptureCompareChan2) return pTIMHandle->pTIMx->CCR2;
    if (Channel == TIM_CaptureCompareChan3) return pTIMHandle->pTIMx->CCR3;
    if (Channel == TIM_CaptureCompareChan4) return pTIMHandle->pTIMx->CCR4;

    return 0;
}

void TIM_SetRepetitionCounter(TIM_Handle_t *pTIMHandle, uint8_t Value) {
    pTIMHandle->pTIMx->RCR = Value;
}

/**
 * @brief Controls the ON / OFF state of the timer output
 * @param pTIMHandle timer handle
 * @param OutputEnabled If the timer output should be enabled or disabled
 */
void TIM_MasterMainOutputControl(TIM_Handle_t *pTIMHandle, uint8_t OutputEnabled) {
    if (OutputEnabled) {
        pTIMHandle->pTIMx->BDTR |= (1 << TIM_BDTR_MOE_Pos);
    } else {
        pTIMHandle->pTIMx->BDTR &=~ (1 << TIM_BDTR_MOE_Pos);
    }
}

/**
 * @brief Controls if the hardware can automatically activate the timer output at the next update event
 * @param pTIMHandle timer handle
 * @param Enabled If the automatic output enable should be enabled
 */
void TIM_AutomaticOutputEnable(TIM_Handle_t *pTIMHandle, uint16_t Enabled) {
    if (Enabled) {
        pTIMHandle->pTIMx->BDTR |= (1 << TIM_BDTR_AOE_Pos);
    } else {
        pTIMHandle->pTIMx->BDTR &=~ (1 << TIM_BDTR_AOE_Pos);
    }
}

/**
 * @brief Set the active level of the break input
 * @param pTIMHandle timer handle
 * @param Polarity If the break input is active LOW or HIGH
 */
void TIM_SetBreakPolarity(TIM_Handle_t *pTIMHandle, TIM_BreakPolarity_e Polarity) {
    if (Polarity == TIM_BreakPolarityLOW) {
        pTIMHandle->pTIMx->BDTR &=~ (1 << TIM_BDTR_BKP_Pos);
    } else {
        pTIMHandle->pTIMx->BDTR |= (1 << TIM_BDTR_BKP_Pos);
    }
}

/**
 * @brief Enables or disables the break input
 * @param pTIMHandle timer handle
 * @param Enabled If the break input should be enabled
 */
void TIM_BreakControl(TIM_Handle_t *pTIMHandle, uint8_t Enabled) {
    if (Enabled) {
        pTIMHandle->pTIMx->BDTR |= (1 << TIM_BDTR_BKE_Pos);
    } else {
        pTIMHandle->pTIMx->BDTR &=~ (1 << TIM_BDTR_BKE_Pos);
    }
}

/**
 * @brief This bit is used when master main output is enabled on channels having a complementary output which are
 *        configured as outputs. This option is not implemented if no complementary output is implemented
 *        in the timer.
 * @param pTIMHandle timer handle
 * @param Mode The desired off state mode (disabled or inactive)
 */
void TIM_OffStateForRunModeControl(TIM_Handle_t *pTIMHandle, TIM_OffStateMode_e Mode) {
    if (Mode == TIM_OffStateModeOutputDisabled) {
        pTIMHandle->pTIMx->BDTR &=~ (1 << TIM_BDTR_OSSR_Pos);
    } else {
        pTIMHandle->pTIMx->BDTR |= (1 << TIM_BDTR_OSSR_Pos);
    }
}

/**
 * @brief This bit is used when master main output is disabled on channels configured as outputs.
 * @param pTIMHandle timer handle
 * @param Mode The desired off state mode (disabled or inactive)
 */
void TIM_OffStateForIdleModeControl(TIM_Handle_t *pTIMHandle, TIM_OffStateMode_e Mode) {
    if (Mode == TIM_OffStateModeOutputDisabled) {
        pTIMHandle->pTIMx->BDTR &=~ (1 << TIM_BDTR_OSSI_Pos);
    } else {
        pTIMHandle->pTIMx->BDTR |= (1 << TIM_BDTR_OSSI_Pos);
    }
}

/**
 * @brief Configures the dead time generator responsible for inserting a delay between turning off one output and turning on its complementary output
 * @param pTIMHandle timer handle
 * @param Config
 */
void TIM_ConfigureDeadTimeGenerator(TIM_Handle_t *pTIMHandle, TIM_DeadTimeGeneratorConfig_t Config) {
    // Clock division
    pTIMHandle->pTIMx->CR1 &=~ TIM_CR1_CKD_Msk;
    pTIMHandle->pTIMx->CR1 |= (Config.ClockDivision << TIM_CR1_CKD_Pos);

    // Duration
    pTIMHandle->pTIMx->BDTR &=~ TIM_BDTR_DTG_Msk;
    pTIMHandle->pTIMx->BDTR |= calc_dtg_value(Config.ClockDivision, Config.Duration) << TIM_BDTR_DTG_Pos;
}

/**
 * @brief This option offers a write protection against software errors.
 * @param pTIMHandle timer handle
 * @param LockLevel the desired lock level
 */
void TIM_LockConfiguration(TIM_Handle_t *pTIMHandle, TIM_LockLevel_e LockLevel) {
    pTIMHandle->pTIMx->BDTR &=~ TIM_BDTR_LOCK_Msk;
    pTIMHandle->pTIMx->BDTR |= (LockLevel << TIM_BDTR_LOCK_Pos);
}

/**
 * @brief Enables the desired timer interrupts
 * @param pTIMHandle timer handle
 * @param InterruptMask All interrupts which need to be enabled. Available options from @TimerInterrupts
 */
void TIM_EnableInterrupts(TIM_Handle_t *pTIMHandle, uint32_t InterruptMask) {
    pTIMHandle->pTIMx->DIER |= InterruptMask;
}

/**
 * @brief Disables the desired timer interrupts
 * @param pTIMHandle timer handle
 * @param InterruptMask All interrupts which need to be disabled. Available options from @TimerInterrupts
 */
void TIM_DisableInterrupts(TIM_Handle_t *pTIMHandle, uint32_t InterruptMask) {
    pTIMHandle->pTIMx->DIER &=~ InterruptMask;
}

/**
 * @brief Enables the IRQ lines for the general purpose timers
 * @note You should use TIM_IRQTIM1Enable() when you use TIM1, not this one!
 * @param pTIMHandle timer handle
 * @param Priority The desired interrupt priority
 */
void TIM_IRQEnable(TIM_Handle_t *pTIMHandle, uint8_t Priority) {
    if (pTIMHandle->pTIMx == TIM1) return;

    uint8_t irqNumber = TIM2_IRQn;
    if (pTIMHandle->pTIMx == TIM3) irqNumber = TIM3_IRQn;
    if (pTIMHandle->pTIMx == TIM4) irqNumber = TIM4_IRQn;
    if (pTIMHandle->pTIMx == TIM5) irqNumber = TIM5_IRQn;
    if (pTIMHandle->pTIMx == TIM9) irqNumber = TIM1_BRK_TIM9_IRQn;
    if (pTIMHandle->pTIMx == TIM10) irqNumber = TIM1_UP_TIM10_IRQn;
    if (pTIMHandle->pTIMx == TIM11) irqNumber = TIM1_TRG_COM_TIM11_IRQn;

    NVIC_EnableIRQ(irqNumber);
    NVIC_SetPriority(irqNumber, Priority);
}

/**
 * @brief Disables the IRQ lines for the general purpose timers
 * @note You should use TIM_IRQTIM1Disable() when you use TIM1, not this one!
 * @warning Some timers share the same IRQ line with the TIM1! You should disable those lines only if you are sure that this will not cause problems!
 * @param pTIMHandle timer handle
 */
void TIM_IRQDisable(TIM_Handle_t *pTIMHandle) {
    uint8_t irqNumber = TIM2_IRQn;
    if (pTIMHandle->pTIMx == TIM3) irqNumber = TIM3_IRQn;
    if (pTIMHandle->pTIMx == TIM4) irqNumber = TIM4_IRQn;
    if (pTIMHandle->pTIMx == TIM5) irqNumber = TIM5_IRQn;
    if (pTIMHandle->pTIMx == TIM9) irqNumber = TIM1_BRK_TIM9_IRQn;
    if (pTIMHandle->pTIMx == TIM10) irqNumber = TIM1_UP_TIM10_IRQn;
    if (pTIMHandle->pTIMx == TIM11) irqNumber = TIM1_TRG_COM_TIM11_IRQn;

    NVIC_DisableIRQ(irqNumber);
}

/**
 * @brief Enables the desired IRQ lines for TIM1
 * @param IRQPairs Structure containing the IRQ number and the priority for the desired interrupt
 * @param Len The length of the IRQPairs array
 */
void TIM_IRQTIM1Enable(TIM_AdvancedTimerIRQPair_t *IRQPairs, uint8_t Len) {
    while (Len > 0) {
        uint8_t number = IRQPairs[Len - 1].IRQNumber;
        uint8_t priority = IRQPairs[Len - 1].Priority;
        NVIC_EnableIRQ(number);
        NVIC_SetPriority(number, priority);
        Len--;
    }
}

/**
 * @brief Disables the desired IRQ lines for TIM1
 * @param IRQNumbers The IRQ numbers which need to be disabled
 * @param Len The length of the IRQNumbers array
 */
void TIM_IRQTIM1Disable(uint8_t *IRQNumbers, uint8_t Len) {
    while (Len > 0) {
        NVIC_DisableIRQ(IRQNumbers[Len - 1]);
        Len--;
    }
}

uint8_t TIM_GetStatusFlag(TIM_Handle_t *pTIMHandle, TIM_Flag_e Flag) {
    return (pTIMHandle->pTIMx->SR & (1 << Flag)) > 0 ? 1 : 0;
}

void TIM_ClearStatusFlag(TIM_Handle_t *pTIMHandle, TIM_Flag_e Flag) {
    pTIMHandle->pTIMx->SR &=~ (1 << Flag);
}

uint8_t calc_dtg_value(TIM_DeadTimeClkDivision_e clkDivision, uint32_t dead_time) {
    // Only TIM1 supports dead time
    uint32_t timerClk = CLOCK_GetApb2Hz();

    // Get clock division for dead-time
    uint8_t divisor = 1;

    switch (clkDivision) {
        case 0: break;
        case 1: divisor = 2; break;
        case 2: divisor = 4; break;
        default: break;
    }

    // Calculate tDTS period in nanoseconds
    uint32_t tdtsNS = (1000000000UL * divisor) / timerClk;

    uint32_t counts = (dead_time + tdtsNS / 2) / tdtsNS;        // Add rounding

    uint8_t dtg = 0xFF;
    if (counts < 128) {
        dtg = counts;
    } else if (counts < 255) {
        dtg = 0x80 | ((counts / 2) - 64);
    } else if (counts < 505) {
        dtg = 0xC0  | ((counts / 8) - 32);
    } else if (counts < 1009) {
        dtg = 0xE0 | ((counts / 16) - 32);
    }

    return dtg;
}
