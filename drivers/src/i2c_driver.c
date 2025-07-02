//
// Created by Kok on 6/16/25.
//

#include "i2c_driver.h"

#include <clock_driver.h>
#include <commons.h>
#include <systick_driver.h>

static void gen_start_condition(I2C_TypeDef *pI2Cx);
void clear_addr_flag(I2C_Handle_t *pI2CHandle);
static void gen_stop_condition(I2C_TypeDef *pI2Cx);
static void exec_addr_phase(I2C_Handle_t *pI2CHandle, uint8_t Address, uint8_t Write);
static void exec_first_10bit_addr_phase(I2C_Handle_t *pI2CHandle, uint16_t Address, uint8_t Write);
static void exec_second_10bit_addr_phase(I2C_Handle_t *pI2CHandle, uint16_t Address);

static I2C_Error_e read_single_byte(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, I2C_DisableStop_e DisableStop);
static I2C_Error_e read_two_bytes(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, I2C_DisableStop_e DisableStop);
static I2C_Error_e read_multiple_bytes(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, I2C_DisableStop_e DisableStop);

// Interrupt handlers
static void it_start_handler(I2C_Handle_t *pI2CHandle);
static void it_addr_handler(I2C_Handle_t *pI2CHandle);
static void it_btf_handler(I2C_Handle_t *pI2CHandle);
static void it_stopf_handler(I2C_Handle_t *pI2CHandle);
static void it_txe_handler(I2C_Handle_t *pI2CHandle);
static void it_rnxe_handler(I2C_Handle_t *pI2CHandle);
static void it_close_data_flow(I2C_Handle_t *pI2CHandle);

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
I2C_Error_e I2C_Init(I2C_Handle_t *pI2CHandle) {
    if (pI2CHandle->I2C_ResetState == I2C_ResetEnabled) return I2C_ErrResetEnabled;

    // Ensure that the peripheral is disabled
    pI2CHandle->pI2Cx->CR1 &=~ (1 << I2C_CR1_PE_Pos);

    // Device address
    pI2CHandle->pI2Cx->OAR1 |= (1 << 14); // According to the reference manual bit 14 should be kept 1
    if (pI2CHandle->I2C_Config.I2C_DeviceAddressLen == I2C_DeviceAddr10Bits) {
        pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD0_Pos);
        pI2CHandle->pI2Cx->OAR1 |= (1 << I2C_OAR1_ADDMODE_Pos);
    } else {
        pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD1_Pos);
        pI2CHandle->pI2Cx->OAR1 &=~ (1 << I2C_OAR1_ADDMODE_Pos);
    }

    // I2C speed
    uint32_t pclk1FreqHz = CLOCK_GetApb1Hz();
    pI2CHandle->pI2Cx->CR2 |= ((pclk1FreqHz / 1000000) << I2C_CR2_FREQ_Pos);

    uint16_t clockPeriodNs = (1000000000 / pclk1FreqHz);
    uint16_t ccrValue = 0;
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SclSpeedSM) {
        // Standard Mode: 100KHz, CCR = T_high / T_PCLK = T_SCL / (2 × T_PCLK)
        ccrValue = 5000 / clockPeriodNs; // 5000ns = half period for 100KHz
    } else {
        uint32_t periodNs = (1000000000 / pI2CHandle->I2C_Config.I2C_SCLSpeed);
        uint8_t divideFactor = pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FmDuty2 ? 3 : 25;
        ccrValue = periodNs / (divideFactor * clockPeriodNs);
    }

    pI2CHandle->pI2Cx->CCR |= ((ccrValue & 0xFFF) << I2C_CCR_CCR_Pos);

    // Set speed and duty mode
    pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY_Pos);
    pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SclSpeedSM ? 0 : 1 << I2C_CCR_FS_Pos);

    // Rise time (TRISE)
    uint32_t numerator = I2C_MAX_TRISE_NS_FOR_SPEED(pI2CHandle->I2C_Config.I2C_SCLSpeed);
    uint32_t denominator = clockPeriodNs;
    // Round the value correctly
    pI2CHandle->pI2Cx->TRISE = ((numerator + (denominator / 2)) / denominator) + 1;

    return I2C_ErrOK;
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
 * @param DisableStop Control if a repeated start should be generated
 * @return OK == 0; ERROR > 0
 */
I2C_Error_e I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint8_t Len, uint16_t SlaveAddr, I2C_DisableStop_e DisableStop) {
    if (Len == 0) return I2C_ErrLenZero;

    // Set ACK
    I2C_AcknowledgeControl(pI2CHandle->pI2Cx, ENABLE);

    // Generate START condition
    gen_start_condition(pI2CHandle->pI2Cx);

    // Confirm START condition
    WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_SB_Pos), I2C_ErrTimeout, TIMEOUT_MS);

    // Send slave address
    if (pI2CHandle->I2C_Config.I2C_DeviceAddressLen == I2C_DeviceAddr7Bits) {
        exec_addr_phase(pI2CHandle, SlaveAddr, ENABLE);
    } else {
        exec_first_10bit_addr_phase(pI2CHandle, SlaveAddr, ENABLE);
        while (!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADD10_Pos)));
        exec_second_10bit_addr_phase(pI2CHandle, SlaveAddr);
    }

    // Confirm slave device available on the bus
    WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_ADDR_Pos), I2C_ErrTimeout, TIMEOUT_MS);

    // Clear ADDR flag
    clear_addr_flag(pI2CHandle);

    // Send the data
    while (Len > 0) {
        // Wait for data register to be empty
        WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_TXE_Pos), I2C_ErrTimeout, TIMEOUT_MS);

        pI2CHandle->pI2Cx->DR = *pTXBuffer++;
        Len--;
    }

    // Wait for transmission completion
    WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_TXE_Pos), I2C_ErrTimeout, TIMEOUT_MS);
    WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_BTF_Pos), I2C_ErrTimeout, TIMEOUT_MS);

    if (!DisableStop) {
        // Generate STOP condition
        gen_stop_condition(pI2CHandle->pI2Cx);
    }

    return I2C_ErrOK;
}

/**
 * @brief Receives data from a slave device
 * @param pI2CHandle I2C peripheral handle
 * @param pRXBuffer The buffer where the read data will be placed
 * @param Len The length of the RX buffer
 * @param SlaveAddr The address of the slave
 * @param DisableStop Control if a repeated start should be generated
 * @return OK == 0; ERROR > 0
 */
I2C_Error_e I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, uint16_t SlaveAddr, I2C_DisableStop_e DisableStop) {
    if (Len == 0) return I2C_ErrLenZero;

    // Set ACK
    I2C_AcknowledgeControl(pI2CHandle->pI2Cx, ENABLE);

    // Generate START condition
    gen_start_condition(pI2CHandle->pI2Cx);

    // Confirm START condition
    WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_SB_Pos), I2C_ErrTimeout, TIMEOUT_MS);

    // Send slave address
    if (pI2CHandle->I2C_Config.I2C_DeviceAddressLen == I2C_DeviceAddr7Bits) {
        exec_addr_phase(pI2CHandle, SlaveAddr, DISABLE);
    } else {
        // Send header with WRITE command
        exec_first_10bit_addr_phase(pI2CHandle, SlaveAddr, ENABLE);
        WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADD10_Pos)), I2C_ErrTimeout, TIMEOUT_MS);
        exec_second_10bit_addr_phase(pI2CHandle, SlaveAddr);

        // Confirm slave device available on the bus
        WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_ADDR_Pos), I2C_ErrTimeout, TIMEOUT_MS);

        // Clear address flag
        clear_addr_flag(pI2CHandle);

        // Generate repeated start condition
        gen_start_condition(pI2CHandle->pI2Cx);
        WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_SB_Pos), I2C_ErrTimeout, TIMEOUT_MS);

        // Send header with READ command
        exec_first_10bit_addr_phase(pI2CHandle, SlaveAddr, DISABLE);
    }

    // Confirm slave device available on the bus
    WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_ADDR_Pos), I2C_ErrTimeout, TIMEOUT_MS);

    // Read the data depending on the length
    I2C_Error_e err;
    if (Len == 1) {
        if ((err = read_single_byte(pI2CHandle, pRXBuffer, DisableStop)) != I2C_ErrOK) return err;
    } else if (Len == 2) {
        if ((err = read_two_bytes(pI2CHandle, pRXBuffer, DisableStop)) != I2C_ErrOK) return err;
    } else {
        if ((err = read_multiple_bytes(pI2CHandle, pRXBuffer, Len, DisableStop)) != I2C_ErrOK) return err;
    }

    // Re-enable ACK
    I2C_AcknowledgeControl(pI2CHandle->pI2Cx, ENABLE);

    return I2C_ErrOK;
}

/**
 * @brief Sends data to a slave device using interrupts
 * @param pI2CHandle I2C peripheral handle
 * @param pTXBuffer The buffer which needs to be send
 * @param Len The length of the TX buffer
 * @param SlaveAddr The address of the slave
 * @return OK == 0; ERROR > 0
 */
I2C_Error_e I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint8_t Len, uint16_t SlaveAddr) {
    if (Len == 0) return I2C_ErrLenZero;
    if (pI2CHandle->I2C_ITState.TxRxState != I2C_Ready) return I2C_ErrPerNotReady;

    pI2CHandle->I2C_ITState.DevAddr = SlaveAddr;
    pI2CHandle->I2C_ITState.TxLen = Len;
    pI2CHandle->I2C_ITState.WriteEnabled = ENABLE;
    pI2CHandle->I2C_ITState.pTXBuffer = pTXBuffer;
    pI2CHandle->I2C_ITState.TxRxState = I2C_TxBusy;

    // Enable event interrupts
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN_Pos);
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN_Pos);
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN_Pos);

    // Set ACK
    I2C_AcknowledgeControl(pI2CHandle->pI2Cx, ENABLE);

    // Generate START condition
    gen_start_condition(pI2CHandle->pI2Cx);

    return I2C_ErrOK;
}

/**
 * @brief Reads data from a slave device using interrupts
 * @param pI2CHandle I2C peripheral handle
 * @param pRXBuffer The buffer where the read data will be placed
 * @param Len The length of the RX buffer
 * @param SlaveAddr The address of the slave
 * @return OK == 0; ERROR > 0
 */
I2C_Error_e I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, uint16_t SlaveAddr) {
    if (Len == 0) return I2C_ErrLenZero;
    if (pI2CHandle->I2C_ITState.TxRxState != I2C_Ready) return I2C_ErrPerNotReady;

    pI2CHandle->I2C_ITState.DevAddr = SlaveAddr;
    pI2CHandle->I2C_ITState.RxLen = Len;
    pI2CHandle->I2C_ITState.RxLenOriginal = Len;
    pI2CHandle->I2C_ITState.WriteEnabled = DISABLE;
    pI2CHandle->I2C_ITState.pRXBuffer = pRXBuffer;
    pI2CHandle->I2C_ITState.TxRxState = I2C_RxBusy;

    // Enable event interrupts
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN_Pos);
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN_Pos);
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN_Pos);

    // Set ACK
    I2C_AcknowledgeControl(pI2CHandle->pI2Cx, ENABLE);

    // Generate START condition
    gen_start_condition(pI2CHandle->pI2Cx);

    return 0;
}

/**
 * @brief Do all the necessary configurations in order to use device as a slave
 * @param pI2CHandle I2C peripheral handle
 * @return OK == 0; ERROR > 0
 */
void I2C_SlaveConfigure(I2C_PerIndex_e I2CIndex, I2C_Handle_t *pI2CHandle) {
    // Enable IRQ line
    I2C_IRQConfig(I2CIndex, 15, ENABLE);

    // Enable peripheral
    I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);

    // Enable ACK
    I2C_AcknowledgeControl(pI2CHandle->pI2Cx, ENABLE);

    // Enable event interrupts
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN_Pos);
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN_Pos);
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN_Pos);
}

/**
 * @brief Used when you want to send a single byte to the master
 * @param pI2CHandle I2C peripheral handle
 * @param data Data to send to master
 * @return OK == 0; ERROR > 0
 */
void I2C_SlaveSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t data) {
    pI2CHandle->pI2Cx->DR = data;
}

/**
 * @brief Used when you want to receive a single byte from the master
 * @param pI2CHandle I2C peripheral handle
 * @param pRXBuffer Buffer in which to store the data from master
 * @return OK == 0; ERROR > 0
 */
void I2C_SlaveReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer) {
    *pRXBuffer = pI2CHandle->pI2Cx->DR;
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
        pI2CHandle->I2C_ResetState = I2C_ResetEnabled;
    } else {
        pI2CHandle->pI2Cx->CR1 &=~ (1 << I2C_CR1_SWRST_Pos);
        pI2CHandle->I2C_ResetState = I2C_ResetDisabled;
    }
}

/**
 * @param PerIndex The index of the I2C peripheral (Possible values from @I2C_PerIndex)
 * @param IRQPriority IRQ Priority (0 - 15)
 * @param Enable If the IRQ is enabled
 */
void I2C_IRQConfig(I2C_PerIndex_e PerIndex, uint8_t IRQPriority, uint8_t Enable) {
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

/**
 * @brief The function which will be executed whenever an interrupt occurs
 * @param pI2CHandle I2C peripheral handle
 */
void I2C_IRQEventHandling(I2C_Handle_t *pI2CHandle) {
    // START condition created (Master only)
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB_Pos)) it_start_handler(pI2CHandle);

    // Address acknowledged
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR_Pos)) it_addr_handler(pI2CHandle);

    // Byte transfer finished
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF_Pos)) it_btf_handler(pI2CHandle);

    // Stop condition detected (Slave only)
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF_Pos)) it_stopf_handler(pI2CHandle);

    // Transmit buffer empty
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE_Pos)) it_txe_handler(pI2CHandle);

    // Receive buffer not empty
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE_Pos)) it_rnxe_handler(pI2CHandle);
}

void I2C_IRQErrorHandling(I2C_Handle_t *pI2CHandle) {

    // Bus error
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR_Pos)) {
        pI2CHandle->pI2Cx->SR1 &=~ (1 << I2C_SR1_BERR_Pos);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EventBusError);
    }

    // Arbitration lost
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO_Pos)) {
        pI2CHandle->pI2Cx->SR1 &=~ (1 << I2C_SR1_ARLO_Pos);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EventArbLost);
    }

    // NACK received
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF_Pos)) {
        pI2CHandle->pI2Cx->SR1 &=~ (1 << I2C_SR1_AF_Pos);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EventNACK);
    }

    // Overrun / Underrun detected
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR_Pos)) {
        pI2CHandle->pI2Cx->SR1 &=~ (1 << I2C_SR1_OVR_Pos);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EventOvr);
    }

    // TIMEOUT error
    if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT_Pos)) {
        pI2CHandle->pI2Cx->SR1 &=~ (1 << I2C_SR1_TIMEOUT_Pos);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EventTimeout);
    }
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, I2C_Event_e AppEvent) {}

void gen_start_condition(I2C_TypeDef *pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_START_Pos);
}

void clear_addr_flag(I2C_Handle_t *pI2CHandle) {
    pI2CHandle->pI2Cx->SR1;
    pI2CHandle->pI2Cx->SR2;
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
void exec_addr_phase(I2C_Handle_t *pI2CHandle, uint8_t Address, uint8_t Write) {
    if (pI2CHandle->I2C_Config.I2C_DeviceAddressLen != I2C_DeviceAddr7Bits) return;
    if (Write) {
        pI2CHandle->pI2Cx->DR = (Address << 1) & ~(1);
    } else {
        pI2CHandle->pI2Cx->DR = (Address << 1) | (1);
    }
}

void exec_first_10bit_addr_phase(I2C_Handle_t *pI2CHandle, uint16_t Address, uint8_t Write) {
    uint8_t lastTwoBits = (Address >> 8) & 0x03;
    if (Write) {
        // 11110 + ADD9 + ADD8 + WriteBit
        pI2CHandle->pI2Cx->DR = (0xF0 | (lastTwoBits << 1)) & ~(1);
    } else {
        // 11110 + ADD9 + ADD8 + ReadBit
        pI2CHandle->pI2Cx->DR = (0xF0 | (lastTwoBits << 1)) | (1);
    }
}

void exec_second_10bit_addr_phase(I2C_Handle_t *pI2CHandle, uint16_t Address) {
    uint8_t firstEightBits = Address & 0xFF;
    pI2CHandle->pI2Cx->DR = firstEightBits;
}

void I2C_AcknowledgeControl(I2C_TypeDef *pI2Cx, uint8_t Enabled) {
    if (Enabled) {
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK_Pos);
    } else {
        pI2Cx->CR1 &=~ (1 << I2C_CR1_ACK_Pos);
    }
}

/* ------------ Master read data ------------ */

I2C_Error_e read_single_byte(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, I2C_DisableStop_e DisableStop) {

    // Disable ACK
    I2C_AcknowledgeControl(pI2CHandle->pI2Cx, DISABLE);

    // Clear ADDR flag
    clear_addr_flag(pI2CHandle);

    // Wait until data available
    WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_RXNE_Pos), I2C_ErrTimeout, TIMEOUT_MS);

    if (!DisableStop) {
        // Generate STOP condition
        gen_stop_condition(pI2CHandle->pI2Cx);
    }

    *pRXBuffer = pI2CHandle->pI2Cx->DR;

    return I2C_ErrOK;
}

I2C_Error_e read_two_bytes(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, I2C_DisableStop_e DisableStop) {
    // Disable ACK and set POS
    I2C_AcknowledgeControl(pI2CHandle->pI2Cx, DISABLE);
    pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_POS_Pos);

    // Clear ADDR flag
    clear_addr_flag(pI2CHandle);

    WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF_Pos)), I2C_ErrTimeout, TIMEOUT_MS);

    if (!DisableStop) {
        // Generate STOP condition
        gen_stop_condition(pI2CHandle->pI2Cx);
    }

    pRXBuffer[0] = pI2CHandle->pI2Cx->DR;
    pRXBuffer[1] = pI2CHandle->pI2Cx->DR;

    return I2C_ErrOK;
}

I2C_Error_e read_multiple_bytes(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, I2C_DisableStop_e DisableStop) {
    // Clear ADDR flag
    clear_addr_flag(pI2CHandle);

    // Read data until 2 bytes are left
    for (int i = Len; i > 2; i--) {
        // Wait for data register to have data
        WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_RXNE_Pos), I2C_ErrTimeout, TIMEOUT_MS);
        pRXBuffer[Len - i] = pI2CHandle->pI2Cx->DR;
    }

    WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_RXNE_Pos), I2C_ErrTimeout, TIMEOUT_MS);

    // 2 bytes left
    // Disable ACK
    I2C_AcknowledgeControl(pI2CHandle->pI2Cx, DISABLE);

    WAIT_WITH_TIMEOUT(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF_Pos)), I2C_ErrTimeout, TIMEOUT_MS);

    if (!DisableStop) {
        // Generate STOP condition
        gen_stop_condition(pI2CHandle->pI2Cx);
    }

    // Read last 2 bytes
    pRXBuffer[Len - 2] = pI2CHandle->pI2Cx->DR;
    pRXBuffer[Len - 1] = pI2CHandle->pI2Cx->DR;

    return I2C_ErrOK;
}

/* ------------ Interrupt Handlers ------------ */

void it_start_handler(I2C_Handle_t *pI2CHandle) {
    exec_addr_phase(pI2CHandle, pI2CHandle->I2C_ITState.DevAddr, pI2CHandle->I2C_ITState.WriteEnabled);
}

void it_addr_handler(I2C_Handle_t *pI2CHandle) {
    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL_Pos)) {
        // Master mode
        if (!pI2CHandle->I2C_ITState.WriteEnabled) {
            // Receive mode
            if (pI2CHandle->I2C_ITState.RxLenOriginal <= 2) {
                // when 1 or 2 bytes are received
                I2C_AcknowledgeControl(pI2CHandle->pI2Cx, DISABLE);
            }
            if (pI2CHandle->I2C_ITState.RxLenOriginal == 2) {
                // when 2 bytes are received - ACK controls the next received byte
                pI2CHandle->pI2Cx->CR1 |= 1 << I2C_CR1_POS_Pos;
            }
        }
    }
    clear_addr_flag(pI2CHandle);
}

void it_btf_handler(I2C_Handle_t *pI2CHandle) {
    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL_Pos)) {
        // Master mode
        if (pI2CHandle->I2C_ITState.WriteEnabled) {
            // Write mode
            if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE_Pos) && pI2CHandle->I2C_ITState.TxLen == 0) {
                // BTF and TXE = 1; All data is sent
                gen_stop_condition(pI2CHandle->pI2Cx);

                it_close_data_flow(pI2CHandle);
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EventTxComplete);
            }
        } else {
            // Read mode
            if (pI2CHandle->I2C_ITState.RxLen == 2) {
                // Read 2 bytes

                uint8_t originalLen = pI2CHandle->I2C_ITState.RxLenOriginal;
                uint8_t currLen = pI2CHandle->I2C_ITState.RxLen;

                gen_stop_condition(pI2CHandle->pI2Cx);

                pI2CHandle->I2C_ITState.pRXBuffer[originalLen-currLen] = pI2CHandle->pI2Cx->DR;
                pI2CHandle->I2C_ITState.pRXBuffer[originalLen-currLen + 1] = pI2CHandle->pI2Cx->DR;
                pI2CHandle->I2C_ITState.RxLen -= 2;

                I2C_AcknowledgeControl(pI2CHandle->pI2Cx, ENABLE);
                it_close_data_flow(pI2CHandle);
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EventRxComplete);
            } else if (pI2CHandle->I2C_ITState.RxLen == 3) {
                // Read 3 bytes
                uint8_t originalLen = pI2CHandle->I2C_ITState.RxLenOriginal;
                uint8_t currLen = pI2CHandle->I2C_ITState.RxLen;

                I2C_AcknowledgeControl(pI2CHandle->pI2Cx, DISABLE);

                pI2CHandle->I2C_ITState.pRXBuffer[originalLen-currLen] = pI2CHandle->pI2Cx->DR;
                pI2CHandle->I2C_ITState.RxLen--;
            }
        }
    }
}

void it_stopf_handler(I2C_Handle_t *pI2CHandle) {
    // Clear the flag
    pI2CHandle->pI2Cx->CR1 |= 0x0000;

    I2C_ApplicationEventCallback(pI2CHandle, I2C_EventStop);
}

void it_txe_handler(I2C_Handle_t *pI2CHandle) {
    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL_Pos)) {
        // Master mode
        if (pI2CHandle->I2C_ITState.TxLen > 0) {
            pI2CHandle->pI2Cx->DR = *pI2CHandle->I2C_ITState.pTXBuffer++;
            pI2CHandle->I2C_ITState.TxLen--;
        }
    } else {
        // Slave mode
        // ACK received
        if (!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF_Pos))) {
            // Master reads data
            if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA_Pos)) {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EventDataRequest);
            }
        }
    }
}

void it_rnxe_handler(I2C_Handle_t *pI2CHandle) {
    uint8_t originalLen = pI2CHandle->I2C_ITState.RxLenOriginal;
    uint8_t currLen = pI2CHandle->I2C_ITState.RxLen;

    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL_Pos)) {
        // Master mode
        if (originalLen == 1) {
            gen_stop_condition(pI2CHandle->pI2Cx);
        }

        if (pI2CHandle->I2C_ITState.RxLen == 1 || pI2CHandle->I2C_ITState.RxLen > 3) {
            pI2CHandle->I2C_ITState.pRXBuffer[originalLen-currLen] = pI2CHandle->pI2Cx->DR;
            pI2CHandle->I2C_ITState.RxLen--;

            if (pI2CHandle->I2C_ITState.RxLen == 0) {
                I2C_AcknowledgeControl(pI2CHandle->pI2Cx, ENABLE);

                it_close_data_flow(pI2CHandle);
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EventRxComplete);
            }
        }
    } else {
        // Slave mode
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EventDataReceive);
    }
}

void it_close_data_flow(I2C_Handle_t *pI2CHandle) {
    // Clear POS bit (if it's 2 bytes transaction)
    if (pI2CHandle->I2C_ITState.RxLenOriginal == 2) {
        pI2CHandle->pI2Cx->CR1 &=~ (1 << I2C_CR1_POS_Pos);
    }

    // Disable interrupts
    pI2CHandle->pI2Cx->CR2 &=~ (1 << I2C_CR2_ITEVTEN_Pos);
    pI2CHandle->pI2Cx->CR2 &=~ (1 << I2C_CR2_ITBUFEN_Pos);
    pI2CHandle->pI2Cx->CR2 &=~ (1 << I2C_CR2_ITERREN_Pos);

    // Clear the interrupt state
    pI2CHandle->I2C_ITState = (I2C_ITState_t){0};
}
