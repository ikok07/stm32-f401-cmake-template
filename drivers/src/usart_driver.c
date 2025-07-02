//
// Created by Kok on 6/25/25.
//

#include "usart_driver.h"

#include <clock_driver.h>
#include <commons.h>
#include <math.h>
#include <stddef.h>
#include <systick_driver.h>

static USART_BaudRateResult_t calc_baud_rate(USART_Handle_t *pUSARTHandle, USART_BaudRate_e BaudRate);

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
            RCC->APB1ENR |= (1 << RCC_APB1ENR_USART2EN_Pos);
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
    USART_BaudRateResult_t brrResult = calc_baud_rate(pUSARTHandle, pUSARTHandle->USART_Config.BaudRate);
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
USART_Error_e USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint32_t Len) {
    while (Len > 0) {
        // Wait for empty tx buffer
        WAIT_WITH_TIMEOUT(!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE_Pos)), USART_ErrTimeout, TIMEOUT_MS);

        if (pUSARTHandle->USART_Config.WordLength == USART_WordLength9Bits) {
            if (pUSARTHandle->USART_Config.ParityControl == USART_ParityControlEnabled) {
                pUSARTHandle->pUSARTx->DR = *pTXBuffer++;
                Len--;
            } else {
                pUSARTHandle->pUSARTx->DR = *(uint16_t*)pTXBuffer & 0x01FF;
                pTXBuffer += 2;
                Len -= 2;
            }
        } else {
            if (pUSARTHandle->USART_Config.ParityControl == USART_ParityControlEnabled) {
                pUSARTHandle->pUSARTx->DR = *pTXBuffer++ & 0x7F;
                Len--;
            } else {
                pUSARTHandle->pUSARTx->DR = *pTXBuffer++;
                Len--;
            }
        }
    }

    // Wait for transfer complete
    WAIT_WITH_TIMEOUT(!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC_Pos)), USART_ErrTimeout, TIMEOUT_MS);

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
USART_Error_e USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Len) {
    uint32_t originalLen = Len;
    while (Len > 0) {
        // Wait for data reception
        WAIT_WITH_TIMEOUT(!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE_Pos)), USART_ErrTimeout, TIMEOUT_MS);

        if (pUSARTHandle->USART_Config.WordLength == USART_WordLength9Bits) {
            if (pUSARTHandle->USART_Config.ParityControl == USART_ParityControlEnabled) {
                pRXBuffer[originalLen - Len] = pUSARTHandle->pUSARTx->DR & 0xFF;
                Len--;
            } else {
                uint16_t data = pUSARTHandle->pUSARTx->DR;
                pRXBuffer[originalLen - Len] = (data >> 8) & 0x01;
                Len--;
                pRXBuffer[originalLen - Len] = data & 0xFF;
                Len--;
            }
        } else {
            if (pUSARTHandle->USART_Config.ParityControl == USART_ParityControlEnabled) {
                pRXBuffer[originalLen - Len] = pUSARTHandle->pUSARTx->DR & 0x7F;
                Len--;
            } else {
                pRXBuffer[originalLen - Len] = pUSARTHandle->pUSARTx->DR;
                Len--;
            }
        }
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
USART_Error_e USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint32_t Len) {
    if (Len == 0) return USART_ErrOK;
    if (pUSARTHandle->USART_ITState.InterruptState != USART_InterruptStateReady) return USART_PerBusy;

    pUSARTHandle->USART_ITState.pTXBuffer = pTXBuffer;
    pUSARTHandle->USART_ITState.TxLen = Len;
    pUSARTHandle->USART_ITState.InterruptState = USART_InterruptStateTXBusy;

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
USART_Error_e USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Len) {
    if (Len == 0) return USART_ErrOK;
    if (pUSARTHandle->USART_ITState.InterruptState != USART_InterruptStateReady) return USART_PerBusy;

    pUSARTHandle->USART_ITState.pRXBuffer = pRXBuffer;
    pUSARTHandle->USART_ITState.RxLenOriginal = Len;
    pUSARTHandle->USART_ITState.RxLen = Len;
    pUSARTHandle->USART_ITState.InterruptState = USART_InterruptStateRXBusy;

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
USART_Error_e USART_IRQEnable(USART_Handle_t *pUSARTHandle, USART_Interrupt_e *EnabledInterrupts, uint32_t Len) {
    uint8_t perIndex = 1;
    if (pUSARTHandle->pUSARTx == USART2) perIndex = 2;
    else if (pUSARTHandle->pUSARTx == USART6) perIndex = 6;

    uint8_t irqNumber = USART_INDEX_TO_IRQ_NUMBER(perIndex);
    NVIC_EnableIRQ(irqNumber);

    if (Len > 0 && EnabledInterrupts != NULL) {
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
            if (pUSARTHandle->USART_Config.WordLength == USART_WordLength9Bits) {
                if (pUSARTHandle->USART_Config.ParityControl == USART_ParityControlEnabled) {
                    pUSARTHandle->pUSARTx->DR = *pUSARTHandle->USART_ITState.pTXBuffer++;
                    pUSARTHandle->USART_ITState.TxLen--;
                } else {
                    pUSARTHandle->pUSARTx->DR = *(uint16_t*)pUSARTHandle->USART_ITState.pTXBuffer & 0x01FF;
                    pUSARTHandle->USART_ITState.pTXBuffer += 2;
                    pUSARTHandle->USART_ITState.TxLen -= 2;
                }
            } else {
                if (pUSARTHandle->USART_Config.ParityControl == USART_ParityControlEnabled) {
                    pUSARTHandle->pUSARTx->DR = *pUSARTHandle->USART_ITState.pTXBuffer++ & 0x7F;
                    pUSARTHandle->USART_ITState.TxLen--;
                } else {
                    pUSARTHandle->pUSARTx->DR = *pUSARTHandle->USART_ITState.pTXBuffer++;
                    pUSARTHandle->USART_ITState.TxLen--;
                }
            }

            if (pUSARTHandle->USART_ITState.TxLen == 0) {
                pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_TXEIE_Pos);
                pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE_Pos);
            }
        }
    }

    // CTS flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagCTS) && pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE_Pos)) {
        USART_ClearFlag(pUSARTHandle, USART_FlagCTS);
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventCTS);
    }

    // Tx complete flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagTC) && pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE_Pos)) {
        if (pUSARTHandle->USART_ITState.TxLen == 0) {
            pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_TCIE_Pos);
            pUSARTHandle->pUSARTx->SR &=~ (1 << USART_SR_TC_Pos);
            pUSARTHandle->USART_ITState.InterruptState = USART_InterruptStateReady;
            USART_ApplicationEventCallback(pUSARTHandle, USART_EventTxComplete);
        }
    }

    // RX not empty flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagRXNE) && pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE_Pos)) {
        if (pUSARTHandle->USART_ITState.RxLen > 0) {
            uint32_t originalLen = pUSARTHandle->USART_ITState.RxLenOriginal;
            uint32_t len = pUSARTHandle->USART_ITState.RxLen;


            if (pUSARTHandle->USART_Config.WordLength == USART_WordLength9Bits) {
                if (pUSARTHandle->USART_Config.ParityControl == USART_ParityControlEnabled) {
                    pUSARTHandle->USART_ITState.pRXBuffer[originalLen - len] = pUSARTHandle->pUSARTx->DR & 0xFF;
                    pUSARTHandle->USART_ITState.RxLen--;
                } else {
                    uint16_t data = pUSARTHandle->pUSARTx->DR;
                    pUSARTHandle->USART_ITState.pRXBuffer[originalLen - len] = (data >> 8) & 0x01;
                    pUSARTHandle->USART_ITState.RxLen--;
                    pUSARTHandle->USART_ITState.pRXBuffer[originalLen - len] = data & 0xFF;
                    pUSARTHandle->USART_ITState.RxLen--;
                }
            } else {
                if (pUSARTHandle->USART_Config.ParityControl == USART_ParityControlEnabled) {
                    pUSARTHandle->USART_ITState.pRXBuffer[originalLen - len] = pUSARTHandle->pUSARTx->DR & 0x7F;
                    pUSARTHandle->USART_ITState.RxLen--;
                } else {
                    pUSARTHandle->USART_ITState.pRXBuffer[originalLen - len] = pUSARTHandle->pUSARTx->DR;
                    pUSARTHandle->USART_ITState.RxLen--;
                }
            }

            if (len - 1 == 0) {
                pUSARTHandle->USART_ITState.InterruptState = USART_InterruptStateReady;
                pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_RXNEIE_Pos);
                USART_ApplicationEventCallback(pUSARTHandle, USART_EventRxComplete);
            }
        }
    }

    // Overrun flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagORE) && (pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE_Pos) || pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE_Pos))) {
        // Clear flags
        uint32_t originalLen = pUSARTHandle->USART_ITState.RxLenOriginal;
        uint32_t len = pUSARTHandle->USART_ITState.RxLen;
        pUSARTHandle->USART_ITState.pRXBuffer[originalLen - len] = pUSARTHandle->pUSARTx->DR;

        pUSARTHandle->USART_ITState.InterruptState = USART_InterruptStateReady;
        pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_RXNEIE_Pos);

        USART_ApplicationEventCallback(pUSARTHandle, USART_EventOverrunDetected);
    }

    // IDLE flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagIDLE) && pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE_Pos)) {
        // Clear flags
        (void)pUSARTHandle->pUSARTx->DR;

        pUSARTHandle->USART_ITState.InterruptState = USART_InterruptStateReady;
        pUSARTHandle->pUSARTx->CR1 &=~ (1 << USART_CR1_RXNEIE_Pos);

        USART_ApplicationEventCallback(pUSARTHandle, USART_EventIdleDetected);
    }

    // Parity error flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagPE) && pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_PEIE_Pos)) {
        pUSARTHandle->USART_ITState.InterruptState = USART_InterruptStateReady;
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventParityError);
    }

    // Break flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagLBD) && pUSARTHandle->pUSARTx->CR2 & (1 << USART_CR2_LBDIE_Pos)) {
        pUSARTHandle->USART_ITState.InterruptState = USART_InterruptStateReady;
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventBreakFlag);
    }

    // Noise flag
    if (USART_GetFlag(pUSARTHandle, USART_FlagNF) && pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE_Pos)) {
        USART_ApplicationEventCallback(pUSARTHandle, USART_EventNoiseFlag);
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
 * @brief Sets the desired USART baud rate
 * @param pUSARTHandle USART Handle
 * @param BaudRate The desired baud rate
 * @return USART_ErrOK if success
 */
USART_Error_e USART_SetBaudRate(USART_Handle_t *pUSARTHandle, USART_BaudRate_e BaudRate) {
    USART_BaudRateResult_t brrResult = calc_baud_rate(pUSARTHandle, BaudRate);
    if (brrResult.Mantissa == 0) {
        return USART_ErrInvalidBaudRate;
    }
    pUSARTHandle->pUSARTx->BRR = (brrResult.Mantissa << 4) | brrResult.Fraction;
    return USART_ErrOK;
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

/**
 * @brief Clears the desired flag in the status register
 * @note This only works with flags which can be disabled by writing directly to the SR
 * @param pUSARTHandle USART handle
 * @param Flag Flag to read
 * @return Enabled or disabled flag
 */
void USART_ClearFlag(USART_Handle_t *pUSARTHandle, USART_Flag_e Flag) {
    pUSARTHandle->pUSARTx->SR &=~ (1 << Flag);
}

USART_BaudRateResult_t calc_baud_rate(USART_Handle_t *pUSARTHandle, USART_BaudRate_e BaudRate) {
    uint32_t pclk = pUSARTHandle->pUSARTx == USART2 ? CLOCK_GetApb1Hz() : CLOCK_GetApb2Hz();
    USART_Oversampling_e oversampling = pUSARTHandle->USART_Config.Oversampling;

    uint32_t usartdiv = (pclk * 100) / (8 * (2 - oversampling) * BaudRate);
    uint16_t mantissa = usartdiv / 100 & 0xFFF;
    uint8_t fraction = (usartdiv % 100 * (16 - 8 * oversampling) + 50) / 100;        // 50 is used for rounding

    USART_BaudRateResult_t result = {
        .Mantissa = mantissa & 0xFFF,
        .Fraction = fraction
    };

    return result;
}

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, USART_Event_e AppEvent) {}
