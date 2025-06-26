//
// Created by Kok on 6/25/25.
//

#include "usart_driver.h"

#include <commons.h>
#include <stddef.h>

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
 * @brief Controls the power state of the USART transmitter
 * @param pUSARTHandle USART Handle
 * @param Enabled If the peripheral should be enabled or disabled
 */
void USART_PeripheralTXControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled) {
    if (Enabled) {
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TE_Pos);
    } else {
        pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_TE_Pos);
    }
}

/**
 * @brief Controls the power state of the USART receiver
 * @param pUSARTHandle USART Handle
 * @param Enabled If the peripheral should be enabled or disabled
 */
void USART_PeripheralRXControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled) {
    if (Enabled) {
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RE_Pos);
    } else {
        pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_RE_Pos);
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

/**
 * @brief Sends data via USART peripheral
 * @note This is a blocking method
 * @param pUSARTHandle USART Handle
 * @param pTXBuffer The buffer which needs to be sent
 * @param Len The length of the buffer
 * @return USART_ErrOK if successful
 */
USART_Error_e USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint8_t Len) {
    while (Len > 0) {
        // Wait for empty tx buffer
        while (!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE_Pos))) {}

        pUSARTHandle->pUSARTx->DR = *pTXBuffer++;
    }

    // Wait for transfer complete
    while (!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC_Pos)));

    return USART_ErrOK;
}

/**
 * @brief Sends USART break signal
 * @note This is a blocking method
 * @param pUSARTHandle USART Handle
 */
void USART_SendBreak(USART_Handle_t *pUSARTHandle) {
    pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_SBK_Pos);
}

/**
 * @brief Receives data via USART peripheral
 * @note This is a blocking method
 * @param pUSARTHandle USART Handle
 * @param pRXBuffer The buffer where the received data is stored
 * @param Len The length of the buffer
 * @return USART_ErrOK if successful
 */
USART_Error_e USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint8_t Len) {
    uint32_t originalLen = Len;
    while (Len > 0) {
        // Wait for data reception
        while (!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE_Pos)));

        pRXBuffer[originalLen - Len] = pUSARTHandle->pUSARTx->DR;
        Len--;
    }

    return USART_ErrOK;
}

/**
 * @brief Configures the USASRT interrupt state for transmission
 * @note The peripheral must be correctly setup for interrupt transmission before calling this method
 * @param pUSARTHandle USART handle
 * @param pTXBuffer The buffer which needs to be sent
 * @param Len The length of the buffer
 */
USART_Error_e USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint8_t Len) {
    if (Len == 0) return;
    if (pUSARTHandle->USART_ITState.InterruptState != USART_InterruptReady) return USART_PerBusy;

    pUSARTHandle->USART_ITState.pTXBuffer = pTXBuffer;
    pUSARTHandle->USART_ITState.TxLen = Len;
    pUSARTHandle->USART_ITState.InterruptState = USART_InterruptTXBusy;

    // Enable interrupt
    pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE_Pos);

    return USART_ErrOK;
}

/**
 * @brief Configures the USASRT interrupt state for reception
 * @note The peripheral must be correctly setup for interrupt reception before calling this method
 * @param pUSARTHandle USART handle
 * @param pRXBuffer The buffer where to store the data
 * @param Len The length of the buffer
 */
USART_Error_e USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint8_t Len) {
    if (Len == 0) return;
    if (pUSARTHandle->USART_ITState.InterruptState != USART_InterruptReady) return USART_PerBusy;

    pUSARTHandle->USART_ITState.pRXBuffer = pRXBuffer;
    pUSARTHandle->USART_ITState.RxLen = Len;
    pUSARTHandle->USART_ITState.InterruptState = USART_InterruptRXBusy;

    // Enable interrupt
    pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE_Pos);

    return USART_ErrOK;
}

/**
 *
 * @param pUSARTHandle USART handle
 * @param EnabledInterrupts An array of interrupts which should be enabled
 * @param Len The length of the enabledInterrupts array
 * @return USART_ErrOK - success. USART_ErrArgumentNULL - EnabledInterrupts is NULL.
 */
USART_Error_e USART_IRQEnable(USART_Handle_t *pUSARTHandle, USART_Interrupt_e *EnabledInterrupts, uint8_t Len) {
    if (EnabledInterrupts == NULL) return USART_ErrArgumentNULL;
    if (Len == 0) return USART_ErrOK;

    uint8_t perIndex = 1;
    if (pUSARTHandle->pUSARTx == USART2) perIndex = 2;
    else if (pUSARTHandle->pUSARTx == USART6) perIndex = 6;

    uint8_t irqNumber = USART_INDEX_TO_IRQ_NUMBER(perIndex);
    NVIC_EnableIRQ(irqNumber);

    while (Len > 0) {
        switch (*EnabledInterrupts++) {
            case USART_InterruptTXEmpty:
                pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE_Pos);
            break;
            case USART_InterruptTXComplete:
                pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE_Pos);
            break;
            case USART_InterruptRXNotEmpty:
                pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE_Pos);
            break;
            case USART_InterruptCTS:
                pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_CTSIE_Pos);
            break;
            case USART_InterruptIdleLine:
                pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_IDLEIE_Pos);
            break;
            case USART_InterruptParityErr:
                pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PEIE_Pos);
            break;
            case USART_InterruptBreakFlag:
                pUSARTHandle->pUSARTx->CR2 |= (1 << USART_CR2_LBDIE_Pos);
            break;
            case USART_InterruptGenericErrors:
                pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_EIE_Pos);
            break;
        }
        Len--;
    }

    return USART_ErrOK;
}

/**
 * @brief Disables all USART interrupts
 * @param pUSARTHandle USART handle
 */
void USART_IRQDisable(USART_Handle_t *pUSARTHandle) {
    uint8_t perIndex = 1;
    if (pUSARTHandle->pUSARTx == USART2) perIndex = 2;
    else if (pUSARTHandle->pUSARTx == USART6) perIndex = 6;

    uint8_t irqNumber = USART_INDEX_TO_IRQ_NUMBER(perIndex);
    NVIC_DisableIRQ(irqNumber);
}

void USART_IRQHandler(USART_Handle_t *pUSARTHandle) {

    // TX empty flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagTXE) && pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE_Pos)) {
        if (pUSARTHandle->USART_ITState.TxLen > 0) {
            pUSARTHandle->pUSARTx->DR = *pUSARTHandle->USART_ITState.pTXBuffer++;
            pUSARTHandle->USART_ITState.TxLen--;

            if (pUSARTHandle->USART_ITState.TxLen == 0) {
                pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_TXEIE_Pos);
                pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE_Pos);
            }
        }
    }

    // CTS flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagCTS) && pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE_Pos)) {
        pUSARTHandle->pUSARTx->SR &=~ (1 << USART_SR_CTS_Pos);
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventCTS);
    }

    // Tx complete flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagTC) && pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE_Pos)) {
        if (pUSARTHandle->USART_ITState.TxLen == 0) {
            pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_TCIE_Pos);
            pUSARTHandle->pUSARTx->SR &=~ (1 << USART_SR_TC_Pos);
            pUSARTHandle->USART_ITState.InterruptState = USART_InterruptReady;
            USART_ApplicationEventCallback(pUSARTHandle, USART_EventTxComplete);
        }
    }

    // RX not empty flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagRXNE) && pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE_Pos)) {
        if (pUSARTHandle->USART_ITState.RxLen > 0) {
            uint32_t originalLen = pUSARTHandle->USART_ITState.RxLenOriginal;
            uint32_t len = pUSARTHandle->USART_ITState.RxLen;
            pUSARTHandle->USART_ITState.pRXBuffer[originalLen - len] = pUSARTHandle->pUSARTx->DR;
            pUSARTHandle->USART_ITState.RxLen--;
        } else {
            pUSARTHandle->USART_ITState.InterruptState = USART_InterruptReady;
            pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_RXNEIE_Pos);
            USART_ApplicationEventCallback(pUSARTHandle, USART_EventRxComplete);
        }
    }

    // Overrun flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagORE) && (pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE_Pos) || pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE_Pos))) {
        pUSARTHandle->pUSARTx->SR &=~ (1 << USART_SR_ORE_Pos);
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventOverrunDetected);
    }

    // IDLE flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagIDLE) && pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE_Pos)) {
        pUSARTHandle->pUSARTx->SR &=~ (1 << USART_SR_IDLE_Pos);
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventIdleDetected);
    }

    // Parity error flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagPE) && pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_PEIE_Pos)) {
        pUSARTHandle->pUSARTx->SR &=~ (1 << USART_SR_PE_Pos);
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventParityError);
    }

    // Break flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagLBD) && pUSARTHandle->pUSARTx->CR2 & (1 << USART_CR2_LBDIE_Pos)) {
        pUSARTHandle->pUSARTx->SR &=~ (1 << USART_SR_LBD_Pos);
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventBreakFlag);
    }

    // Noise flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagNF) && pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE_Pos)) {
        pUSARTHandle->pUSARTx->SR &=~ (1 << USART_SR_NE_Pos);
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventNoiseFlag);
    }

    // Framing error flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagFE) && pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE_Pos)) {
        pUSARTHandle->pUSARTx->SR &=~ (1 << USART_SR_FE_Pos);
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventFramingError);
    }
}

void USART_CTSControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled) {
    if (Enabled) {
        pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_CTSE_Pos);
    } else {
        pUSARTHandle->pUSARTx->CR3 &=~ (1 << USART_CR3_CTSE_Pos);
    }
}

void USART_RSSControl(USART_Handle_t *pUSARTHandle, uint8_t Enabled) {
    if (Enabled) {
        pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_RTSE_Pos);
    } else {
        pUSARTHandle->pUSARTx->CR3 &=~ (1 << USART_CR3_RTSE_Pos);
    }
}

/**
 * @brief Gets the current state of the flag in the status register
 * @param pUSARTHandle USART handle
 * @param Flag Flag to read
 * @return Enabled or disabled flag
 */
USART_FlagStatus_e USART_GetFlag(USART_Handle_t *pUSARTHandle, USART_Flag_e Flag) {
    return pUSARTHandle->pUSARTx->SR & (1 << Flag) ? 1 : 0;
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

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, USART_Event_e AppEvent) {}
