//
// Created by Kok on 6/16/25.
//

#include "i2c_driver.h"

#include <commons.h>

static uint32_t get_pclk1_clock();
static void gen_start_condition(I2C_TypeDef *pI2Cx);
static void gen_stop_condition(I2C_TypeDef *pI2Cx);
static void exec_addr_phase(I2C_TypeDef *pI2Cx, uint8_t Address, uint8_t Write);
static void clear_addr_flag(I2C_TypeDef *pI2Cx);
static void set_ack_flag(I2C_TypeDef *pI2Cx, uint8_t Enabled);

static uint8_t read_single_byte(I2C_TypeDef *pI2Cx, uint8_t *pRXBuffer, uint8_t DisableStop);
static uint8_t read_multiple_bytes(I2C_TypeDef *pI2Cx, uint8_t *pRXBuffer, uint8_t Len, uint8_t DisableStop);

/**
 * @brief Enables or disables peripheral clock for the given I2C peripheral
 * @param pI2Cx Base address of the I2C peripheral
 * @param Enable If the peripheral is enabled or disabled (1 or 0)
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t Enable) {
    if (Enable) {
        if (pI2Cx == I2C1)        I2C_PCLK_EN(RCC_APB1ENR_I2C1EN_Pos)
        else if (pI2Cx == I2C2)   I2C_PCLK_EN(RCC_APB1ENR_I2C2EN_Pos)
        else if (pI2Cx == I2C3)   I2C_PCLK_EN(RCC_APB1ENR_I2C3EN_Pos)
    } else {
        if (pI2Cx == I2C1)        I2C_PCLK_DI(RCC_APB1ENR_I2C1EN_Pos)
        else if (pI2Cx == I2C2)   I2C_PCLK_DI(RCC_APB1ENR_I2C2EN_Pos)
        else if (pI2Cx == I2C3)   I2C_PCLK_DI(RCC_APB1ENR_I2C3EN_Pos)
    }
}

/**
 * @brief Initializes the I2C peripheral. Must be called first!
 * @param pI2CHandle I2C peripheral handle
 * @return OK == 0; ERROR > 0
 */
uint8_t I2C_Init(I2C_Handle_t *pI2CHandle) {
    if (pI2CHandle->I2C_ResetState == I2C_RESET_ENABLED) return 1;

    // Ensure that the peripheral is disabled
    pI2CHandle->pI2Cx->CR1 &=~ (1 << I2C_CR1_PE_Pos);

    // Device address
    pI2CHandle->pI2Cx->OAR1 |= (1 << 14); // According to the reference manual bit 14 should be kept 1
    if (pI2CHandle->I2C_Config.I2C_DeviceAddressLen == I2C_DEVICE_ADDR_10_BITS) {
        pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD0_Pos);
        pI2CHandle->pI2Cx->OAR1 |= (1 << I2C_OAR1_ADDMODE_Pos);
    } else {
        pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD1_Pos);
        pI2CHandle->pI2Cx->OAR1 &=~ (1 << I2C_OAR1_ADDMODE_Pos);
    }

    // I2C speed
    uint32_t pclk1FreqHz = get_pclk1_clock();
    pI2CHandle->pI2Cx->CR2 |= ((pclk1FreqHz / 1000000) << I2C_CR2_FREQ_Pos);

    uint16_t clockPeriodNs = (1000000000 / pclk1FreqHz);
    uint16_t ccrValue = 0;
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
        // Standard Mode: 100KHz, CCR = T_high / T_PCLK = T_SCL / (2 × T_PCLK)
        ccrValue = 5000 / clockPeriodNs; // 5000ns = half period for 100KHz
    } else {
        uint32_t periodNs = (1000000000 / pI2CHandle->I2C_Config.I2C_SCLSpeed);
        uint8_t divideFactor = pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2 ? 3 : 25;
        ccrValue = periodNs / (divideFactor * clockPeriodNs);
    }

    pI2CHandle->pI2Cx->CCR |= ((ccrValue & 0xFFF) << I2C_CCR_CCR_Pos);

    // Set speed and duty mode
    pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY_Pos);
    pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM ? 0 : 1 << I2C_CCR_FS_Pos);

    // Rise time (TRISE)
    uint32_t numerator = I2C_MAX_TRISE_NS_FOR_SPEED(pI2CHandle->I2C_Config.I2C_SCLSpeed);
    uint32_t denominator = clockPeriodNs;
    // Round the value correctly
    pI2CHandle->pI2Cx->TRISE = ((numerator + (denominator / 2)) / denominator) + 1;

    return 0;
}

void I2C_DeInit(I2C_TypeDef *pI2Cx) {
    pI2Cx->CR1 = 0;
    pI2Cx->CR2 = 0;
    pI2Cx->OAR1 = 0;
    pI2Cx->OAR2 = 0;
    pI2Cx->CCR = 0;
    pI2Cx->TRISE = 0x0002;
    pI2Cx->FLTR = 0;
    I2C_PeriClockControl(pI2Cx, DISABLE);
}

/**
 * @brief Sends data to a slave device
 * @param pI2CHandle I2C peripheral handle
 * @param pTXBuffer The buffer which needs to be send
 * @param Len The length of the TX buffer
 * @param SlaveAddr The address of the slave
 * @return OK == 0; ERROR > 0
 */
uint8_t I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint8_t Len, uint16_t SlaveAddr, uint8_t DisableStop) {
    // Set ACK
    set_ack_flag(pI2CHandle->pI2Cx, ENABLE);

    // Generate START condition
    gen_start_condition(pI2CHandle->pI2Cx);

    // Confirm START condition
    while (!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_SB_Pos));

    // Send slave address
    exec_addr_phase(pI2CHandle->pI2Cx, SlaveAddr, ENABLE);

    // Confirm slave device available on the bus
    while (!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_ADDR_Pos));

    // Clear ADDR flag
    clear_addr_flag(pI2CHandle->pI2Cx);

    // Send the data
    while (Len > 0) {
        // Wait for data register to be empty
        while (!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_TXE_Pos));

        pI2CHandle->pI2Cx->DR = *pTXBuffer++;
        Len--;
    }

    // Wait for transmission completion
    while (!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_TXE_Pos));
    while (!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_BTF_Pos));

    if (!DisableStop) {
        // Generate STOP condition
        gen_stop_condition(pI2CHandle->pI2Cx);
    }

    return 0;
}

/**
 * @brief Receives data from a slave device
 * @param pI2CHandle I2C peripheral handle
 * @param pRXBuffer The buffer where the read data will be placed
 * @param Len The length of the RX buffer
 * @param SlaveAddr The address of the slave
 * @return OK == 0; ERROR > 0
 */
uint8_t I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, uint16_t SlaveAddr, uint8_t DisableStop) {
    // Set ACK
    set_ack_flag(pI2CHandle->pI2Cx, ENABLE);

    // Generate START condition
    gen_start_condition(pI2CHandle->pI2Cx);

    // Confirm START condition
    while (!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_SB_Pos));

    // Send slave address
    exec_addr_phase(pI2CHandle->pI2Cx, SlaveAddr, DISABLE);

    // Confirm slave device available on the bus
    while (!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_ADDR_Pos));

    // Read the data depending on the length
    if (Len == 1) {
        read_single_byte(pI2CHandle->pI2Cx, pRXBuffer, DisableStop);
    } else {
        read_multiple_bytes(pI2CHandle->pI2Cx, pRXBuffer, Len, DisableStop);
    }

    // Re-enable ACK
    set_ack_flag(pI2CHandle->pI2Cx, ENABLE);

    return 0;
}

/**
 *
 * @param pI2Cx Base address of the I2C peripheral
 * @return If the peripheral is enabled
 */
uint8_t I2C_PeripheralEnabled(I2C_TypeDef *pI2Cx) {
    return (pI2Cx->CR1 & (1 << I2C_CR1_PE)) > 0;
}

/**
 * @brief Sets the PE bit of the I2C peripheral
 * @param pI2Cx Base address of the I2C peripheral
 * @param Enable If the peripheral is enabled
 */
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t Enable) {
    if (Enable) {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE_Pos);
    } else {
        pI2Cx->CR1 &=~ (1 << I2C_CR1_PE_Pos);
    }
}

void I2C_SetResetState(I2C_Handle_t *pI2CHandle, uint8_t Enable) {
    if (Enable) {
        pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_SWRST_Pos);
        pI2CHandle->I2C_ResetState = I2C_RESET_ENABLED;
    } else {
        pI2CHandle->pI2Cx->CR1 &=~ (1 << I2C_CR1_SWRST_Pos);
        pI2CHandle->I2C_ResetState = I2C_RESET_DISABLED;
    }
}

/**
 * @param PerIndex The index of the I2C peripheral (1, 2, pr 3)
 * @param IRQPriority IRQ Priority (0 - 15)
 * @param Enable If the IRQ is enabled
 */
void I2C_IRQConfig(uint8_t PerIndex, uint8_t IRQPriority, uint8_t Enable) {
    uint8_t eventIRQ = I2C_INDEX_TO_IRQ_NUMBER(PerIndex, 1);
    uint8_t errorIRQ = I2C_INDEX_TO_IRQ_NUMBER(PerIndex, 0);

    if (Enable) {
        NVIC_EnableIRQ(eventIRQ);
        NVIC_EnableIRQ(errorIRQ);

        NVIC_SetPriority(eventIRQ, IRQPriority);
        NVIC_SetPriority(errorIRQ, IRQPriority);
    } else {
        NVIC_DisableIRQ(eventIRQ);
        NVIC_DisableIRQ(errorIRQ);
    }
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

void gen_start_condition(I2C_TypeDef *pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_START_Pos);
}

void gen_stop_condition(I2C_TypeDef *pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP_Pos);
}

/**
 *
 * @param pI2Cx Base address of the I2C peripheral
 * @param Address The address of the target device
 * @param Write If the controller device wants to write or read
 */
void exec_addr_phase(I2C_TypeDef *pI2Cx, uint8_t Address, uint8_t Write) {
    if (Write) {
        pI2Cx->DR = (Address << 1) & ~(1);
    } else {
        pI2Cx->DR = (Address << 1) | (1);
    }
}

void clear_addr_flag(I2C_TypeDef *pI2Cx) {
    pI2Cx->SR1;
    pI2Cx->SR2;
}

void set_ack_flag(I2C_TypeDef *pI2Cx, uint8_t Enabled) {
    if (Enabled) {
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK_Pos);
    } else {
        pI2Cx->CR1 &=~ (1 << I2C_CR1_ACK_Pos);
    }
}

uint8_t read_single_byte(I2C_TypeDef *pI2Cx, uint8_t *pRXBuffer, uint8_t DisableStop) {

    // Disable ACK
    set_ack_flag(pI2Cx, DISABLE);

    // Clear ADDR flag
    clear_addr_flag(pI2Cx);

    // Wait until data available
    while (!(pI2Cx->SR1 & 1 << I2C_SR1_RXNE_Pos));

    if (!DisableStop) {
        // Generate STOP condition
        gen_stop_condition(pI2Cx);
    }

    *pRXBuffer = pI2Cx->DR;

    return 0;
}

uint8_t read_multiple_bytes(I2C_TypeDef *pI2Cx, uint8_t *pRXBuffer, uint8_t Len, uint8_t DisableStop) {
    // Clear ADDR flag
    clear_addr_flag(pI2Cx);

    // Read data
    for (int i = Len; i > 0; i--) {
        // Wait for data register to have data
        while (!(pI2Cx->SR1 & 1 << I2C_SR1_RXNE_Pos));

        if (i == 2) {
            // Disable ACK
            set_ack_flag(pI2Cx, DISABLE);
            if (!DisableStop) {
                // Generate STOP condition
                gen_stop_condition(pI2Cx);
            }
        }

        pRXBuffer[Len - i] = pI2Cx->DR;
    }

    return 0;
}
