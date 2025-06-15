//
// Created by Kok on 6/12/25.
//

#include "spi_driver.h"
#include "commons.h"

#include <string.h>

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIHandle);

/**
 * @brief Enables or disables peripheral clock for the given SPI peripheral
 * @param pSPIx Base address of the SPI peripheral
 * @param Enabled If the peripheral is enabled or disabled (1 or 0)
 */
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t Enabled) {
    if (Enabled) {
        if (pSPIx == SPI1) SPI_PCLK_EN(1, RCC_APB2ENR_SPI1EN_Pos);
        else if (pSPIx == SPI2) SPI_PCLK_EN(2, RCC_APB1ENR_SPI2EN_Pos);
        else if (pSPIx == SPI3) SPI_PCLK_EN(3, RCC_APB1ENR_SPI3EN_Pos);
        else if (pSPIx == SPI4) SPI_PCLK_EN(4, RCC_APB2ENR_SPI4EN_Pos);
    } else {
        if (pSPIx == SPI1) SPI_PCLK_DI(1, RCC_APB2ENR_SPI1EN_Pos);
        else if (pSPIx == SPI2) SPI_PCLK_DI(2, RCC_APB1ENR_SPI2EN_Pos);
        else if (pSPIx == SPI3) SPI_PCLK_DI(3, RCC_APB1ENR_SPI3EN_Pos);
        else if (pSPIx == SPI4) SPI_PCLK_DI(4, RCC_APB2ENR_SPI4EN_Pos);
    }
}

void SPI_Init(SPI_Handle_t *pSPIHandle) {

    // Make sure SPI is disabled
    pSPIHandle->pSPIx->CR1 &=~ (1 << SPI_CR1_SPE_Pos);

    // Set device mode
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_Pos;

    // Set bus configuration
    if (pSPIHandle->SPIConfig.SPI_BusConfig < SPI_BUS_CFG_SIMPLEX_TXONLY) {
        if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FULL_DUPLEX) {
            pSPIHandle->pSPIx->CR1 &=~ (1 << SPI_CR1_BIDIMODE_Pos);
        } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FULL_DUPLEX_RXONLY) {
            pSPIHandle->pSPIx->CR1 &=~ (1 << SPI_CR1_BIDIMODE_Pos);
            pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_RXONLY_Pos);
        }
        /*
         * In half duplex mode the direction of the data
         * is controlled manually with SPI_BidirectionalModeDirection()
         */
        else {
            pSPIHandle->pSPIx->CR1 |= 1 << SPI_CR1_BIDIMODE_Pos;
        }
    } else {
        pSPIHandle->pSPIx->CR1 |= 1 << SPI_CR1_BIDIMODE_Pos;
        if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_SIMPLEX_RXONLY) {
            SPI_BidirectionalModeDirection(pSPIHandle->pSPIx, DISABLE);
        } else {
            SPI_BidirectionalModeDirection(pSPIHandle->pSPIx, ENABLE);
        }
    }

    // Set peripheral clock speed
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR_Pos;

    // Set data frame format (8 or 16 bits)
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DF << SPI_CR1_DFF_Pos;

    if (pSPIHandle->SPIConfig.SPI_FrameFormat == SPI_FF_MOTOROLA) {
        // Set clock polarity
        pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_Pos;

        // Set clock phase
        pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_Pos;

        // Set bits order (MSB or LSB first)
        pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_BitsOrder << SPI_CR1_LSBFIRST_Pos;

        // Set hardware or software mode
        pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM_Pos;

        if (pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_SW) {
            // If in software MASTER mode, the internal NSS pin state should be high
            if (pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) {
                pSPIHandle->pSPIx->CR1 |= 1 << SPI_CR1_SSI_Pos;
            }
            // If in software SLAVE mode, the internal NSS pin state should be low
            else {
                pSPIHandle->pSPIx->CR1 &=~ (1 << SPI_CR1_SSI_Pos);
            }
        }

        // Set SS output enable bit
        pSPIHandle->pSPIx->CR2 |= pSPIHandle->SPIConfig.SPI_SS_OutputEnabled << SPI_CR2_SSOE_Pos;
    }

    // Set frame format (Motorola or TI)
    pSPIHandle->pSPIx->CR2 |= pSPIHandle->SPIConfig.SPI_FrameFormat << SPI_CR2_FRF_Pos;
}

/**
 * @brief De-Initializes the SPI peripheral
 * @param pSPIx Base address of the SPI peripheral
 */
void SPI_DeInit(SPI_TypeDef *pSPIx) {
    pSPIx->CR1 = 0;
    pSPIx->CR2 = 0;
    pSPIx->SR = 0x0002;
    SPI_PeriClockControl(pSPIx, DISABLE);
}

/**
 * @brief Sends data to the selected SPI device
 * @note This mode should only be used if the peripheral
 *       is not set tot RX Only mode and you don't want to read
 *       the data from the other device
 * @param pSPIHandle SPI peripheral handle
 * @param pTXBuffer Transfer buffer
 * @param len The length of the transfer buffer
 * @warning This is a blocking call
 * @return OK - 0. ERROR > 0
 */
uint8_t SPI_SendData(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t len) {
    if (len == 0) return 0;

    // Check if the peripheral is configured for tx communication
    if (
        pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FULL_DUPLEX_RXONLY ||
        pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_SIMPLEX_RXONLY
    ) {
        return 1;
    }

    // Check if peripheral is enabled
    if (!SPI_PeripheralEnabled(pSPIHandle->pSPIx)) {
        return 2;
    }

    while (len > 0) {
        // Wait until Tx buffer is empty
        while (!(pSPIHandle->pSPIx->SR & 1 << SPI_SR_TXE_Pos));

        if (pSPIHandle->SPIConfig.SPI_DF == SPI_DF_16BITS && len > 1) {
            pSPIHandle->pSPIx->DR = *((uint16_t*)pTXBuffer);
            (uint16_t*)pTXBuffer++;
            len -= 2;
        } else {
            pSPIHandle->pSPIx->DR = *pTXBuffer;
            pTXBuffer++;
            len -= 1;
        }
    }

    // Wait for transfer to complete
    while (!(pSPIHandle->pSPIx->SR & 1 << SPI_SR_TXE_Pos));
    while (pSPIHandle->pSPIx->SR & 1 << SPI_SR_BSY_Pos);

    return 0;
}

/**
 * @brief Receives data from the selected SPI device
 * @note This mode should only be used if the peripheral
 *       is set to RX Only mode
 * @param pSPIHandle SPI peripheral handle
 * @param pRXBuffer Receive buffer
 * @param len The length of the receive buffer
 * @warning This is a blocking call
 * @return OK - 0. ERROR > 0
 */
uint8_t SPI_ReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t len) {
    if (len == 0) return 0;

    // Check if the peripheral is configured only for rx communication
    if (
        pSPIHandle->SPIConfig.SPI_BusConfig != SPI_BUS_CFG_FULL_DUPLEX_RXONLY &&
        pSPIHandle->SPIConfig.SPI_BusConfig != SPI_BUS_CFG_SIMPLEX_RXONLY
    ) {
        return 1;
    }

    // Check if peripheral is enabled
    if (!SPI_PeripheralEnabled(pSPIHandle->pSPIx)) {
        return 2;
    }

    uint32_t rxIndex = 0;

    while (len > 0) {
        // Wait for data
        while (!(pSPIHandle->pSPIx->SR & 1 << SPI_SR_RXNE_Pos));

        if (pSPIHandle->SPIConfig.SPI_DF == SPI_DF_16BITS) {
            *(uint16_t*)&pRXBuffer[rxIndex] = pSPIHandle->pSPIx->DR;
            rxIndex += 2;
            len -= 2;
        } else {
            pRXBuffer[rxIndex] = pSPIHandle->pSPIx->DR;
            rxIndex += 1;
            len -= 1;
        }
    }

    // Wait for transfer to complete
    while (pSPIHandle->pSPIx->SR & 1 << SPI_SR_BSY_Pos);

    return 0;
}

/**
 * @brief Send and simultaneously receives data from the selected device
 * @param pSPIHandle SPI peripheral handle
 * @param pTXBuffer Transfer buffer
 * @param pRXBuffer Receive buffer
 * @param len Length of the transfer and receive buffer
 * @warning This is a blocking call
 * @return OK - 0. ERROR > 0
 */
uint8_t SPI_SendReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint8_t *pRXBuffer, uint32_t len) {
    if (len == 0) return 0;

    // Check if peripheral is enabled
    if (!SPI_PeripheralEnabled(pSPIHandle->pSPIx)) {
        return 2;
    }

    // Write the first data item
    if (pSPIHandle->SPIConfig.SPI_DF == SPI_DF_16BITS && len > 1) {
        pSPIHandle->pSPIx->DR = *(uint16_t*)pTXBuffer++;
        len -= 2;
    } else {
        pSPIHandle->pSPIx->DR = *pTXBuffer++;
        len -= 1;
    }

    uint32_t rxIndex = 0;
    uint32_t totalLen = len;

    while (len > 0) {
        // Wait until Tx buffer is empty
        while (!(pSPIHandle->pSPIx->SR & 1 << SPI_SR_TXE_Pos));

        if (pSPIHandle->SPIConfig.SPI_DF == SPI_DF_16BITS && len > 1) {
            pSPIHandle->pSPIx->DR = *(uint16_t*)pTXBuffer++;

            // Wait rx data
            while (!(pSPIHandle->pSPIx->SR & 1 << SPI_SR_RXNE_Pos));

            *(uint16_t*)&pRXBuffer[rxIndex] = pSPIHandle->pSPIx->DR;
            rxIndex += 2;
            len -= 2;
        } else {
            pSPIHandle->pSPIx->DR = *pTXBuffer++;

            // Wait rx data
            while (!(pSPIHandle->pSPIx->SR & 1 << SPI_SR_RXNE_Pos));

            pRXBuffer[rxIndex] = pSPIHandle->pSPIx->DR;
            rxIndex += 1;
            len -= 1;
        }
    }

    // Wait for the last rx data
    while (!(pSPIHandle->pSPIx->SR & 1 << SPI_SR_RXNE_Pos));
    if (pSPIHandle->SPIConfig.SPI_DF == SPI_DF_16BITS && rxIndex + 1 < totalLen) {
        *(uint16_t*)&pRXBuffer[rxIndex] = pSPIHandle->pSPIx->DR;
    } else {
        pRXBuffer[rxIndex] = pSPIHandle->pSPIx->DR;
    }

    // Wait for all transfers to complete
    while (!(pSPIHandle->pSPIx->SR & 1 << SPI_SR_TXE_Pos));
    while (pSPIHandle->pSPIx->SR & 1 << SPI_SR_BSY_Pos);

    return 0;
}

/**
 * @brief Sends data to the selected SPI device using interrupts
 * @note This mode should only be used if the peripheral
 *       is not set tot RX Only mode and you don't want to read
 *       the data from the other device
 * @param pSPIHandle SPI peripheral handle
 * @param pTXBuffer Transfer buffer
 * @param len The length of the transfer buffer
 * @return OK - 0. ERROR > 0
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t len) {
    if (len == 0) return 0;

    // Check if the peripheral is configured for tx communication
    if (
        pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FULL_DUPLEX_RXONLY ||
        pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_SIMPLEX_RXONLY
    ) {
        return 1;
    }

    // Check if peripheral is enabled
    if (!SPI_PeripheralEnabled(pSPIHandle->pSPIx)) {
        return 2;
    }

    if (pSPIHandle->TXState == SPI_STATE_READY) {
        pSPIHandle->pTXBuffer = pTXBuffer;
        pSPIHandle->TXLen = len;
        pSPIHandle->TXState = SPI_STATE_BUSY_TX;

        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE_Pos);
    }

    return 0;
}

/**
 * @brief Receives data from the selected SPI device using interrupts
 * @note This mode should only be used if the peripheral
 *       is set to RX Only mode
 * @param pSPIHandle SPI peripheral handle
 * @param pRXBuffer Receive buffer
 * @param len The length of the receive buffer
 * @return OK - 0. ERROR > 0
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t len) {
    if (len == 0) return 0;

    // Check if the peripheral is configured only for rx communication
    if (
        pSPIHandle->SPIConfig.SPI_BusConfig != SPI_BUS_CFG_FULL_DUPLEX_RXONLY &&
        pSPIHandle->SPIConfig.SPI_BusConfig != SPI_BUS_CFG_SIMPLEX_RXONLY
    ) {
        return 1;
    }

    // Check if peripheral is enabled
    if (!SPI_PeripheralEnabled(pSPIHandle->pSPIx)) {
        return 2;
    }

    if (pSPIHandle->RXState == SPI_STATE_READY) {
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE_Pos);
        pSPIHandle->pRXBuffer = pRXBuffer;
        pSPIHandle->RXLen = len;
        pSPIHandle->RXState = SPI_STATE_BUSY_RX;
    }

    return 0;
}

/**
 *
 * @param pSPIx Base address of the SPI peripheral
 * @return If the peripheral is enabled
 */
uint8_t SPI_PeripheralEnabled(SPI_TypeDef *pSPIx) {
    return ((pSPIx->CR1 & (1 << SPI_CR1_SPE_Pos)) > 0) ? 1 : 0;
}

/**
 * @brief Sets the SPE bit of the SPI peripheral
 * @param pSPIx Base address of the SPI peripheral
 * @param enabled If the peripheral is enabled
 */
void SPI_PeripheralControl(SPI_Handle_t *pSPIHandle, uint8_t enabled) {
    if (enabled) {
        pSPIHandle->pSPIx->CR1 |= 1 << SPI_CR1_SPE_Pos;
    } else {
        // Master mode
        if (pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) {
            // Full duplex mode
            if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FULL_DUPLEX) {
                while (!(pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE_Pos)));
                while (!(pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE_Pos)));
                while (pSPIHandle->pSPIx->SR & (1 << SPI_SR_BSY_Pos));
            }
            // Simplex TX only mode
            else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_SIMPLEX_TXONLY) {
                while (!(pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE_Pos)));
                while (pSPIHandle->pSPIx->SR & (1 << SPI_SR_BSY_Pos));
            }
            // Full duplex RX only mode / Half duplex mode / Simplex RX only mode
            else if (
                pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FULL_DUPLEX_RXONLY ||
                pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_HALF_DUPLEX ||
                pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_SIMPLEX_RXONLY
            ) {
                // TODO: To be implemented

                // Handle Motorola format
                if (pSPIHandle->SPIConfig.SPI_FrameFormat == SPI_FF_MOTOROLA) {

                }
                // Handle TI format
                else {

                }
            }
        }
        pSPIHandle->pSPIx->CR1 &=~ (1 << SPI_CR1_SPE_Pos);
    }
}

/**
 * @brief Disables the SPI peripheral without waiting for it to be not busy
 * @param pSPIx Base address of the SPI peripheral
 */
void SPI_PeripheralForceDisable(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->CR1 &=~ (1 << SPI_CR1_SPE_Pos);
}

/**
 * @brief Controls in which direction the data is flowing in half duplex or simplex mode
 * @param pSPIx Base address of the SPI peripheral
 * @param enableTx If set transfer is allowed, otherwise receive only
 */
void SPI_BidirectionalModeDirection(SPI_TypeDef *pSPIx, uint8_t enableTx) {
    if (enableTx) {
        pSPIx->CR1 |= 1 << SPI_CR1_BIDIOE_Pos;
    } else {
        pSPIx->CR1 &=~ (1 << SPI_CR1_BIDIOE_Pos);
    }
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->CR2 &=~ (1 << SPI_CR2_TXEIE_Pos);
    pSPIHandle->pTXBuffer = NULL;
    pSPIHandle->TXState = SPI_STATE_READY;
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->CR2 &=~ (1 << SPI_CR2_RXNEIE_Pos);
    pSPIHandle->pRXBuffer = NULL;
    pSPIHandle->RXState = SPI_STATE_READY;
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
}

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->DR;
    pSPIHandle->pSPIx->SR;
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR_COMPLETE);
}

/**
 *
 * @param PerIndex The index of the SPI peripheral (1, 2, 3 or 4)
 * @param IRQPriority IRQ Priority (0 - 15)
 * @param Enabled If the IRQ is enabled
 */
void SPI_IRQConfig(uint8_t PerIndex, uint8_t IRQPriority, uint8_t Enabled) {
    uint8_t IRQNumber = SPI_INDEX_TO_IRQ_NUMBER(PerIndex);
    if (Enabled) {
        NVIC_EnableIRQ(IRQNumber);
        NVIC_SetPriority(IRQNumber, IRQPriority);
    } else {
        NVIC_DisableIRQ(IRQNumber);
    }
}

/**
 * @brief The function which will be executed whenever an interrupt occurs
 * @param pSPIHandle SPI peripheral handle
 */
void IRQ_Handling(SPI_Handle_t *pSPIHandle) {

    // Check for TXE flag
    if (
        pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE_Pos) &&
        pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE_Pos)
    ) {
        spi_txe_interrupt_handler(pSPIHandle);
    }

    // Check for RXNE flag
    if (
        pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE_Pos) &&
        pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE_Pos)
    ) {
        spi_rxne_interrupt_handler(pSPIHandle);
    }

    // Check for OVR flag
    if (
        pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR_Pos) &&
        pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE_Pos)
    ) {
        spi_ovr_err_interrupt_handler(pSPIHandle);
    }
}

/**
 * @brief This method will be called when application event occurs.
 * @param pSPIHandle SPI peripheral handle
 * @param AppEvent The SPI event that occured. Options from @SPI_EVENT
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent) {}

void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle) {
    if (pSPIHandle->SPIConfig.SPI_DF == SPI_DF_16BITS && pSPIHandle->TXLen > 1) {
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTXBuffer);
        pSPIHandle->pTXBuffer += 2;
        pSPIHandle->TXLen -= 2;
    } else {
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTXBuffer;
        pSPIHandle->pTXBuffer++;
        pSPIHandle->TXLen -= 1;
    }

    // Prevent further interrupts
    if (!pSPIHandle->TXLen) SPI_CloseTransmission(pSPIHandle);
}

void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle) {
    if (pSPIHandle->SPIConfig.SPI_DF == SPI_DF_16BITS) {
        *(uint16_t*)&pSPIHandle->pRXBuffer = pSPIHandle->pSPIx->DR;
        pSPIHandle->pRXBuffer += 2;
        pSPIHandle->RXLen -= 2;
    } else {
        *pSPIHandle->pRXBuffer = pSPIHandle->pSPIx->DR;
        pSPIHandle->pRXBuffer++;
        pSPIHandle->RXLen -= 1;
    }

    if (!pSPIHandle->RXLen) SPI_CloseReception(pSPIHandle);
}

void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIHandle) {
    if (pSPIHandle->TXState != SPI_STATE_BUSY_TX) SPI_ClearOVRFlag(pSPIHandle);
}