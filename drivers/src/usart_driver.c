//
// Created by Kok on 6/25/25.
//

#include "usart_driver.h"

#include <commons.h>

static uint32_t get_pclk1_clock();
static USART_BaudRateResult_t calc_baud_rate(USART_Handle_t *pUSARTHandle);

/**
 * @brief Controls the peripheral's clock
 * @param pUSARTHandle USART handle
 * @param Enabled If the peripheral's clock should be enabled
 */
void USART_PeriClockControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled) {
    if (Enabled) {
        if (pUSARTHandle->pUSARTx == USART1) {
            RCC->APB2ENR |= (1 << RCC_APB2ENR_USART1EN_Pos);
        } else if (pUSARTHandle->pUSARTx == USART6) {
            RCC->APB2ENR |= (1 << RCC_APB2ENR_USART6EN_Pos);
        } else {
            RCC->APB2ENR |= (1 << RCC_APB1ENR_USART2EN_Pos);
        }
    } else {
        if (pUSARTHandle->pUSARTx == USART1) {
            RCC->APB2ENR &=~ (1 << RCC_APB2ENR_USART1EN_Pos);
        } else if (pUSARTHandle->pUSARTx == USART6) {
            RCC->APB2ENR &=~ (1 << RCC_APB2ENR_USART6EN_Pos);
        } else {
            RCC->APB2ENR &=~ (1 << RCC_APB1ENR_USART2EN_Pos);
        }
    }
}

/**
 * @brief Controls the power state of the peripheral
 * @param pUSARTHandle USART Handle
 * @param Enabled If the peripheral should be enabled or disabled
 */
void USART_PeripheralControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled) {
    if (Enabled) {
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_UE_Pos);
    } else {
        pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_UE_Pos);
    }
}

/**
 * @brief Check if the peripheral is enabled or disabled state
 * @param pUSARTHandle USART Handle
 * @return Disabled or enabled peripheral
 */
USART_PwrState USART_PeripheralEnabled(USART_Handle_t *pUSARTHandle) {
    return pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_UE_Pos);
}

/**
 * @brief Inititializes the USART peripheral
 * @param pUSARTHandle USART Handle
 * @return USART_ErrOK on success
 */
USART_Error_e USART_Init(USART_Handle_t *pUSARTHandle) {

    // Oversampling
    pUSARTHandle->pUSARTx->CR1 |= (pUSARTHandle->USART_Config.Oversampling << USART_CR1_OVER8_Pos);

    // Word length
    pUSARTHandle->pUSARTx->CR1 |= (pUSARTHandle->USART_Config.WordLength << USART_CR1_M_Pos);

    // Parity
    pUSARTHandle->pUSARTx->CR1 |= (pUSARTHandle->USART_Config.ParityControl << USART_CR1_PCE_Pos);
    pUSARTHandle->pUSARTx->CR1 |= (pUSARTHandle->USART_Config.ParitySelection << USART_CR1_PS_Pos);

    // Stop bits
    pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.StopBits << USART_CR2_STOP_Pos);

    // Bit method
    pUSARTHandle->pUSARTx->CR3 |= (pUSARTHandle->USART_Config.BitMethod << USART_CR3_ONEBIT_Pos);

    // CTS Control
    pUSARTHandle->pUSARTx->CR3 |= (pUSARTHandle->USART_Config.CTSControl << USART_CR3_CTSE_Pos);

    // RTS Control
    pUSARTHandle->pUSARTx->CR3 |= (pUSARTHandle->USART_Config.RTSControl << USART_CR3_RTSE_Pos);

    // Clock configure
    if (pUSARTHandle->USART_Config.ClockConfig.ClockControl == USART_ClockEnabled) {
        // Clock enable
        pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.ClockConfig.ClockControl << USART_CR2_CLKEN_Pos);

        // Clock polarity
        pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.ClockConfig.CPOL << USART_CR2_CPOL_Pos);

        // Clock phase
        pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.ClockConfig.CPHA << USART_CR2_CPHA_Pos);

        // Last bit clock pulse
        pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.ClockConfig.LastBitClockPulse << USART_CR2_LBCL_Pos);
    }

    // Baud rate
    USART_BaudRateResult_t brrResult = calc_baud_rate(pUSARTHandle);
    if (brrResult.Mantissa == 0) {
        return USART_ErrInvalidBaudRate;
    }
    pUSARTHandle->pUSARTx->BRR = (brrResult.Mantissa << 4) | brrResult.Fraction;

    return USART_ErrOK;
}

/**
 * @brief De-Initializes the USART peripheral and disables it's clock
 * @param pUSARTHandle USART Handle
 */
void USART_DeInit(USART_Handle_t *pUSARTHandle) {
    pUSARTHandle->pUSARTx->CR1 = 0;
    pUSARTHandle->pUSARTx->CR2 = 0;
    pUSARTHandle->pUSARTx->CR3 = 0;
    pUSARTHandle->pUSARTx->BRR = 0;
    USART_PeriClockControl(pUSARTHandle, DISABLE);
}

uint32_t get_pclk1_clock() {
    uint32_t srcFreq = 0;
    uint8_t swsValue = (RCC->CFGR >> RCC_CFGR_SWS_Pos) & RCC_CFGR_SWS_Msk;
    if (swsValue == 0) srcFreq = 16000000; // HSI
    else if (swsValue == 1) srcFreq = HSE_VALUE; // HSE
    else if (swsValue == 2) {/* To be implemented... */} // PLL
    else return -1;

    uint8_t ahbPresc = 1;
    uint8_t hpreValue = (RCC->CFGR >> RCC_CFGR_HPRE_Pos) & RCC_CFGR_HPRE_Msk;
    if (hpreValue >= 8 && hpreValue <= 15) ahbPresc = 1 << (hpreValue - 7);

    uint8_t apb1Presc = 1;
    uint8_t ppre1Value = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & RCC_CFGR_PPRE1_Msk;
    if (ppre1Value >= 4 && ppre1Value <= 7) apb1Presc = 1 << (ppre1Value - 3);


    return srcFreq / (ahbPresc * apb1Presc);
}

USART_BaudRateResult_t calc_baud_rate(USART_Handle_t *pUSARTHandle) {
    uint32_t pclk = get_pclk1_clock();
    USART_BaudRate_e baudRate = pUSARTHandle->USART_Config.BaudRate;
    USART_Oversampling_e oversampling = pUSARTHandle->USART_Config.Oversampling;

    uint32_t usartdiv = (pclk / (8 * (2 - oversampling) * baudRate)) * 100;
    uint32_t fraction = usartdiv % 100;
    uint32_t mantissa = usartdiv / 100;

    USART_BaudRateResult_t result = {
        .Mantissa = mantissa & 0xFFF
    };

    if (pUSARTHandle->USART_Config.Oversampling == USAR_OversamplingBy8) {
        result.Fraction = ((fraction * 8) / 100) & 0x7;       // mask with 0b111
    } else {
        result.Fraction = ((fraction * 16) / 100) & 0xF;       // mask with 0b1111
    }

    return result;
}


