//
// Created by Kok on 6/12/25.
//

#include "spi_driver.h";

#include <string.h>

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
    pSPIHandle->pSPIx->CR1 &=~ 1 << SPI_CR1_SPE_Pos;

    // Set device mode
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_Pos;

    // Set bus configuration
    if (pSPIHandle->SPIConfig.SPI_BusConfig < SPI_BUS_CFG_SIMPLEX_TXONLY) {
        if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FULL_DUPLEX) {
            pSPIHandle->pSPIx->CR1 &=~ 1 << SPI_CR1_BIDIMODE_Pos;
        } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FULL_DUPLEX_RXONLY) {
            pSPIHandle->pSPIx->CR1 &=~ 1 << SPI_CR1_BIDIMODE_Pos;
            pSPIHandle->pSPIx->CR1 |= 1 << SPI_CR1_RXONLY_Pos;
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
            // If in software MASTER mode, the NSS pin should be pulled high
            if (pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) {
                pSPIHandle->pSPIx->CR1 |= 1 << SPI_CR1_SSI_Pos;
            }
            // If in software SLAVE mode, the NSS pin should be pulled low
            else {
                pSPIHandle->pSPIx->CR1 &=~ 1 << SPI_CR1_SSI_Pos;
            }
        }

        // Set SS output enable bit
        pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SS_OutputEnabled << SPI_CR2_SSOE_Pos;
    }

    // Set frame format (Motorola or TI)
    pSPIHandle->pSPIx->CR2 |= pSPIHandle->SPIConfig.SPI_FrameFormat << SPI_CR2_FRF_Pos;
}

/**
 * @brief Sends data to the selected SPI device
 * @note This mode should only be used if the peripheral
 *       is not set tot RX Only mode and you don't want to read
 *       the data from the other device
 * @param pSPIHandle SPI peripheral handle
 * @param pTXBuffer Transfer buffer
 * @param len The length of the transfer buffer
 * @return OK - 0. ERROR > 0
 */
int SPI_SendData(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t len) {

    // Check if the peripheral is configured for tx communication
    if (
        pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FULL_DUPLEX_RXONLY ||
        pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_SIMPLEX_RXONLY
    ) {
        return 1;
    }

    // Enable peripheral
    SPI_PeripheralControl(pSPIHandle->pSPIx, ENABLE);

    while (len > 0) {
        // Wait until Tx buffer is empty
        while (!(pSPIHandle->pSPIx->SR & 1 << SPI_SR_TXE_Pos));

        if (pSPIHandle->SPIConfig.SPI_DF == SPI_DF_16BITS && len > 1) {
            pSPIHandle->pSPIx->DR = *(uint16_t*)pTXBuffer++;
            len -= 2;
        } else {
            pSPIHandle->pSPIx->DR = *pTXBuffer++;
            len -= 1;
        }
    }

    // Wait for transfer to complete
    while (pSPIHandle->pSPIx->SR & 1 << SPI_SR_BSY_Pos);

    // Disable peripheral
    SPI_PeripheralControl(pSPIHandle->pSPIx, DISABLE);

    return 0;
}

/**
 * @brief Receives data from the selected SPI device
 * @note This mode should only be used if the peripheral
 *       is set to RX Only mode
 * @param pSPIHandle SPI peripheral handle
 * @param pRXBuffer Receive buffer
 * @param len The length of the receive buffer
 * @return OK - 0. ERROR > 0
 */
int SPI_ReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t len) {
    if (len == 0) return 0;

    // Check if the peripheral is configured only for rx communication
    if (
        pSPIHandle->SPIConfig.SPI_BusConfig != SPI_BUS_CFG_FULL_DUPLEX_RXONLY &&
        pSPIHandle->SPIConfig.SPI_BusConfig != SPI_BUS_CFG_SIMPLEX_RXONLY
    ) {
        return 1;
    }

    // Enable peripheral
    SPI_PeripheralControl(pSPIHandle->pSPIx, ENABLE);

    uint32_t rxIndex = 0;
    uint8_t buffer[len];

    while (len > 0) {
        // Wait for data
        while (!(pSPIHandle->pSPIx->SR & 1 << SPI_SR_RXNE_Pos));

        if (pSPIHandle->SPIConfig.SPI_DF == SPI_DF_16BITS) {
            *(uint16_t*)&buffer[rxIndex] = pSPIHandle->pSPIx->DR;
            rxIndex += 2;
            len -= 2;
        } else {
            buffer[rxIndex] = pSPIHandle->pSPIx->DR;
            rxIndex += 1;
            len -= 1;
        }
    }

    // Wait for transfer to complete
    while (pSPIHandle->pSPIx->SR & 1 << SPI_SR_BSY_Pos);

    // Disable peripheral
    SPI_PeripheralControl(pSPIHandle->pSPIx, DISABLE);

    memcpy(pRXBuffer, buffer, sizeof(buffer));

    return 0;
}

/**
 * @brief Sets the SPE bit of the SPI peripheral
 * @param pSPIx Base address of the SPI peripheral
 * @param enabled If the peripheral is enabled
 */
void SPI_PeripheralControl(SPI_TypeDef *pSPIx, uint8_t enabled) {
    if (enabled) {
        pSPIx->CR1 |= 1 << SPI_CR1_SPE_Pos;
    } else {
        pSPIx->CR1 &=~ 1 << SPI_CR1_SPE_Pos;
    }
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
        pSPIx->CR1 &=~ 1 << SPI_CR1_BIDIOE_Pos;
    }
}
