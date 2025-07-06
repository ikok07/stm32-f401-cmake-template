//
// Created by Kok on 7/6/25.
//

#include "dma_driver.h"
#include "commons.h"

static uint8_t check_interrupt_enabled(DMA_Handle_t *pDMAHandle, DMA_Interrupt_e Interrupt);
static uint8_t get_interrupt_flag(DMA_Handle_t *pDMAHandle, DMA_Interrupt_e Interrupt);
static void clear_interrupt_flag(DMA_Handle_t *pDMAHandle, DMA_Interrupt_e Interrupt);

/**
 * @brief Enables or disables the access to the AHB1 bus for the corresponding DMA
 * @param pDMAHandle DMA handle
 * @param Enable If the peripheral clock should be enabled
 */
void DMA_PeriClockControl(DMA_Handle_t *pDMAHandle, uint8_t Enable) {
    if (Enable) {
        if (pDMAHandle->pDMAxStream < DMA2_Stream0) {
            RCC->AHB1ENR |= (1 << RCC_AHB1ENR_DMA1EN_Pos);
        } else {
            RCC->AHB1ENR |= (1 << RCC_AHB1ENR_DMA2EN_Pos);
        }
    } else {
        if (pDMAHandle->pDMAxStream < DMA2_Stream0) {
            RCC->AHB1ENR &=~ (1 << RCC_AHB1ENR_DMA1EN_Pos);
        } else {
            RCC->AHB1ENR &=~ (1 << RCC_AHB1ENR_DMA2EN_Pos);
        }
    }
}

/**
 * @brief Enables or disables the desired DMA stream
 * @param pDMAHandle DMA handle
 * @param Enable If the DMA stream should be enabled
 */
void DMA_PeripheralControl(DMA_Handle_t *pDMAHandle, uint8_t Enable) {
    if (Enable) {
        pDMAHandle->pDMAxStream->CR |= (1 << DMA_SxCR_EN_Pos);
    } else {
        pDMAHandle->pDMAxStream->CR &=~ (1 << DMA_SxCR_EN_Pos);
    }
}

/**
 * @brief Initializes DMA stream
 * @param pDMAHandle DMA handle
 */
void DMA_Init(DMA_Handle_t *pDMAHandle) {
    // Disable stream
    DMA_PeripheralControl(pDMAHandle, DISABLE);

    // DMA Channel
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.Channel << DMA_SxCR_CHSEL_Pos);

    // Memory burst
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.MemoryBurstConfig << DMA_SxCR_MBURST_Pos);

    // Peripheral burst
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.PeripheralBurstConfig << DMA_SxCR_PBURST_Pos);

    // Double buffer mode
    if (pDMAHandle->Config.FlowController == DMA_FlowController && pDMAHandle->Config.Direction != DMA_DirMemToMem) {
        pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.DoubleBufferMode << DMA_SxCR_DBM_Pos);
    }

    // Priority level
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.Priority << DMA_SxCR_PL_Pos);

    // Peripheral increment offset
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.PeripheralIncrementOffsetSize << DMA_SxCR_PINCOS_Pos);

    // Memory data size
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.MemoryDataSize << DMA_SxCR_MSIZE_Pos);

    // Peripheral data size
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.PeripheralDataSize << DMA_SxCR_PSIZE_Pos);

    // Memory increment mode
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.MemoryIncrementMode << DMA_SxCR_MINC_Pos);

    // Peripheral increment mode
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.PeripheralIncrementMode << DMA_SxCR_PINC_Pos);

    // Circular mode
    if (pDMAHandle->Config.FlowController == DMA_FlowController) {
        pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.CircularMode << DMA_SxCR_CIRC_Pos);
    }

    // Direct mode
    if (pDMAHandle->Config.Direction != DMA_DirMemToMem && pDMAHandle->Config.MemoryBurstConfig == DMA_Burst1 && pDMAHandle->Config.PeripheralBurstConfig == DMA_Burst1) {
        pDMAHandle->pDMAxStream->FCR |= (pDMAHandle->Config.DirectModeDisabled << DMA_SxFCR_DMDIS_Pos);
    }

    // FIFO Threshold
    pDMAHandle->pDMAxStream->FCR |= (pDMAHandle->Config.FIFOThreshold << DMA_SxFCR_FTH_Pos);

    // Data transfer direction
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.Direction << DMA_SxCR_DIR_Pos);

    // Peripheral flow controller
    pDMAHandle->pDMAxStream->CR |= (pDMAHandle->Config.FlowController << DMA_SxCR_PFCTRL_Pos);
}

void DMA_DeInit(DMA_Handle_t *pDMAHandle) {
    DMA_PeripheralControl(pDMAHandle, DISABLE);
    pDMAHandle->pDMAxStream->CR = 0;
    pDMAHandle->pDMAxStream->CR = 0;
    pDMAHandle->pDMAxStream->NDTR = 0;
    pDMAHandle->pDMAxStream->PAR = 0;
    pDMAHandle->pDMAxStream->M0AR = 0;
    pDMAHandle->pDMAxStream->M1AR = 0;
    pDMAHandle->pDMAxStream->FCR = 0x21;
    DMA_PeriClockControl(pDMAHandle, DISABLE);
}

/**
 * @brief Configures the DMA stream for a single buffer transfer
 * @param pDMAHandle DMA handle
 * @param PerAddr Peripheral address
 * @param Mem0Addr Memory address
 * @param Len Number of data items to transfer
 */
DMA_Error_e DMA_ConfigureSingleBufferTransfer(DMA_Handle_t *pDMAHandle, uint32_t PerAddr, uint32_t Mem0Addr, uint16_t Len) {
    if (pDMAHandle->pDMAxStream->CR & (1 << DMA_SxCR_DBM_Pos)) return DMA_ErrIncompatibleBufferMode;

    // Peripheral address
    pDMAHandle->pDMAxStream->PAR = PerAddr;

    // Memory address
    pDMAHandle->pDMAxStream->M0AR = Mem0Addr;

    // Data length
    if (pDMAHandle->Config.MemoryBurstConfig != DMA_Burst1) {
        uint8_t multipleOfValue = DMA_MBURST_CONFIG_VAL_TO_BEATS(pDMAHandle->Config.MemoryBurstConfig) * (pDMAHandle->Config.MemoryDataSize / pDMAHandle->Config.PeripheralDataSize);
        if (Len % multipleOfValue == 0) {
            pDMAHandle->pDMAxStream->NDTR = Len;
        } else {
            return DMA_ErrInvalidDataLength;
        }
    } else {
        pDMAHandle->pDMAxStream->NDTR = Len;
    }

    return DMA_ErrOK;
}

/**
 * @brief Configures the DMA stream for a double buffer transfer
 * @param pDMAHandle DMA handle
 * @param PerAddr Peripheral address
 * @param Mem0Addr Memory 1 address
 * @param Mem1Addr Memory 2 address
 * @param Len Number of data items to transfer
 */
DMA_Error_e DMA_ConfigureDoubleBufferTransfer(DMA_Handle_t *pDMAHandle, uint32_t PerAddr, uint32_t Mem0Addr, uint32_t Mem1Addr,
    uint16_t Len) {
    if (!(pDMAHandle->pDMAxStream->CR & (1 << DMA_SxCR_DBM_Pos))) return DMA_ErrIncompatibleBufferMode;

    // Peripheral address
    pDMAHandle->pDMAxStream->PAR = PerAddr;

    // Memory address
    pDMAHandle->pDMAxStream->M0AR = Mem0Addr;
    pDMAHandle->pDMAxStream->M1AR = Mem1Addr;

    // Data length
    if (pDMAHandle->Config.MemoryBurstConfig != DMA_Burst1) {
        uint8_t multipleOfValue = DMA_MBURST_CONFIG_VAL_TO_BEATS(pDMAHandle->Config.MemoryBurstConfig) * (pDMAHandle->Config.MemoryDataSize / pDMAHandle->Config.PeripheralDataSize);
        if (Len % multipleOfValue == 0) {
            pDMAHandle->pDMAxStream->NDTR = Len;
        } else {
            return DMA_ErrInvalidDataLength;
        }
    } else {
        pDMAHandle->pDMAxStream->NDTR = Len;
    }

    return DMA_ErrOK;
}

/**
 * @brief Start a DMA transaction
 * @param pDMAHandle DMA handle
 */
void DMA_StartTransaction(DMA_Handle_t *pDMAHandle) {
    DMA_ClearInterruptFlags(pDMAHandle);
    DMA_PeripheralControl(pDMAHandle, ENABLE);
}

/**
 * @brief Stops the currently active DMA transaction
 * @param pDMAHandle DMA handle
 */
void DMA_StopTransaction(DMA_Handle_t *pDMAHandle) {
    pDMAHandle->pDMAxStream->CR &=~ (1 << DMA_SxCR_EN_Pos);
}

/**
 * @brief Enables the desired interrupts
 * @param pDMAHandle DMA handle
 * @param EnabledInterrupts The desired interrupts to be enabled
 * @param Len The length of the desired interrupts
 */
void DMA_EnableInterrupts(DMA_Handle_t *pDMAHandle, DMA_Interrupt_e *EnabledInterrupts, uint8_t Len) {
    while (Len > 0) {
        switch (*EnabledInterrupts++) {
            case DMA_InterruptHalfTransfer:
                pDMAHandle->pDMAxStream->CR |= (1 << DMA_SxCR_HTIE_Pos);
            break;
            case DMA_InterruptTransferComplete:
                pDMAHandle->pDMAxStream->CR |= (1 << DMA_SxCR_TCIE_Pos);
            break;
            case DMA_InterruptTransferError:
                pDMAHandle->pDMAxStream->CR |= (1 << DMA_SxCR_TEIE_Pos);
            break;
            case DMA_InterruptFIFOOverrun:
                pDMAHandle->pDMAxStream->FCR |= (1 << DMA_SxFCR_FEIE_Pos);
            break;
            case DMA_InterruptDirectModeError:
                pDMAHandle->pDMAxStream->CR |= (1 << DMA_SxCR_DMEIE_Pos);
            break;
        }
        Len--;
    }
}

/**
 * @brief Disables the desired interrupts
 * @param pDMAHandle DMA handle
 * @param DisabledInterrupts The desired interrupts to be disabled
 * @param Len The length of the desired interrupts
 */
void DMA_DisableInterrupts(DMA_Handle_t *pDMAHandle, DMA_Interrupt_e *DisabledInterrupts, uint8_t Len) {
    while (Len > 0) {
        switch (*DisabledInterrupts++) {
            case DMA_InterruptHalfTransfer:
                pDMAHandle->pDMAxStream->CR &=~ (1 << DMA_SxCR_HTIE_Pos);
            break;
            case DMA_InterruptTransferComplete:
                pDMAHandle->pDMAxStream->CR &=~ (1 << DMA_SxCR_TCIE_Pos);
            break;
            case DMA_InterruptTransferError:
                pDMAHandle->pDMAxStream->CR &=~ (1 << DMA_SxCR_TEIE_Pos);
            break;
            case DMA_InterruptFIFOOverrun:
                pDMAHandle->pDMAxStream->FCR &=~ (1 << DMA_SxFCR_FEIE_Pos);
            break;
            case DMA_InterruptDirectModeError:
                pDMAHandle->pDMAxStream->CR &=~ (1 << DMA_SxCR_DMEIE_Pos);
            break;
        }
        Len--;
    }
}

/**
 * @brief Enables the corresponding DMA stream IRQ number
 * @param pDMAHandle DMA handle
 * @param IRQPriority IRQ Priority
 */
void DMA_IRQEnable(DMA_Handle_t *pDMAHandle, uint8_t IRQPriority) {
    uint8_t irqNumber = DMA_STREAM_TO_IRQ_NUMBER(pDMAHandle->pDMAxStream);
    NVIC_EnableIRQ(irqNumber);
    NVIC_SetPriority(irqNumber, IRQPriority);
}

/**
 * @brief Disables the corresponding DMA stream IRQ number
 * @param pDMAHandle DMA handle
 */
void DMA_IRQDisable(DMA_Handle_t *pDMAHandle) {
    uint8_t irqNumber = DMA_STREAM_TO_IRQ_NUMBER(pDMAHandle->pDMAxStream);
    NVIC_DisableIRQ(irqNumber);
}

void DMA_IRQHanding(DMA_Handle_t *pDMAHandle) {
    // Transfer error
    if (check_interrupt_enabled(pDMAHandle, DMA_InterruptTransferError) && get_interrupt_flag(pDMAHandle, DMA_InterruptTransferError)) {
        clear_interrupt_flag(pDMAHandle, DMA_InterruptTransferError);
        DMA_ApplicationCallback(pDMAHandle, DMA_FlagTransferError);
    }

    // Direct mode error
    if (check_interrupt_enabled(pDMAHandle, DMA_InterruptDirectModeError) && get_interrupt_flag(pDMAHandle, DMA_InterruptDirectModeError)) {
        clear_interrupt_flag(pDMAHandle, DMA_InterruptDirectModeError);
        DMA_ApplicationCallback(pDMAHandle, DMA_FlagDirectModeError);
    }

    // FIFO Overrun
    if (check_interrupt_enabled(pDMAHandle, DMA_InterruptFIFOOverrun) && get_interrupt_flag(pDMAHandle, DMA_InterruptFIFOOverrun)) {
        clear_interrupt_flag(pDMAHandle, DMA_InterruptFIFOOverrun);
        DMA_ApplicationCallback(pDMAHandle, DMA_FlagFIFOOverrun);
    }

    // Half transfer complete
    if (check_interrupt_enabled(pDMAHandle, DMA_InterruptHalfTransfer) && get_interrupt_flag(pDMAHandle, DMA_InterruptHalfTransfer)) {
        clear_interrupt_flag(pDMAHandle, DMA_InterruptHalfTransfer);
        DMA_ApplicationCallback(pDMAHandle, DMA_FlagHalfTransferComplete);
    }

    // Transfer complete
    if (check_interrupt_enabled(pDMAHandle, DMA_InterruptTransferComplete) && get_interrupt_flag(pDMAHandle, DMA_InterruptTransferComplete)) {
        clear_interrupt_flag(pDMAHandle, DMA_InterruptTransferComplete);
        DMA_ApplicationCallback(pDMAHandle, DMA_FlagTransferComplete);
    }
}

/**
 * @param pDMAHandle DMA Handle
 * @return The remaining transfers that need to be made by the DMA stream
 */
uint16_t DMA_GetRemainingTransferCount(DMA_Handle_t *pDMAHandle) {
    return pDMAHandle->pDMAxStream->NDTR & 0xFFFF;
}

/**
 * @param pDMAHandle DMA handle
 * @return If the DMA stream is currently transferring data
 */
uint8_t DMA_IsTransferActive(DMA_Handle_t *pDMAHandle) {
    return (pDMAHandle->pDMAxStream->CR & (1 << DMA_SxCR_EN_Pos)) > 0;
}

/**
 * @param pDMAHandle DMA handle
 * @return Pointer to the currently active DMA buffer
 */
uint32_t *DMA_GetCurrentBuffer(DMA_Handle_t *pDMAHandle) {
    if (!(pDMAHandle->pDMAxStream->CR & (1 << DMA_SxCR_DBM_Pos))) return (uint32_t*)pDMAHandle->pDMAxStream->M0AR;

    if (pDMAHandle->pDMAxStream->CR & (1 << DMA_SxCR_CT_Pos)) return (uint32_t*)pDMAHandle->pDMAxStream->M1AR;
    return (uint32_t*)pDMAHandle->pDMAxStream->M0AR;
}

/**
 * @brief Clears all interrupt flags
 * @param pDMAHandle DMA handle
 */
void DMA_ClearInterruptFlags(DMA_Handle_t *pDMAHandle) {
    if (pDMAHandle->pDMAxStream == DMA1_Stream0) {
        DMA1->LIFCR |= ((1 << DMA_LIFCR_CFEIF0_Pos) | (0xF << DMA_LIFCR_CDMEIF0_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream1) {
        DMA1->LIFCR |= ((1 << DMA_LIFCR_CFEIF1_Pos) | (0xF << DMA_LIFCR_CDMEIF1_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream2) {
        DMA1->LIFCR |= ((1 << DMA_LIFCR_CFEIF2_Pos) | (0xF << DMA_LIFCR_CDMEIF2_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream3) {
        DMA1->LIFCR |= ((1 << DMA_LIFCR_CFEIF3_Pos) | (0xF << DMA_LIFCR_CDMEIF3_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream4) {
        DMA1->HIFCR |= ((1 << DMA_HIFCR_CFEIF4_Pos) | (0xF << DMA_HIFCR_CDMEIF4_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream5) {
        DMA1->HIFCR |= ((1 << DMA_HIFCR_CFEIF5_Pos) | (0xF << DMA_HIFCR_CDMEIF5_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream6) {
        DMA1->HIFCR |= ((1 << DMA_HIFCR_CFEIF6_Pos) | (0xF << DMA_HIFCR_CDMEIF6_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream7) {
        DMA1->HIFCR |= ((1 << DMA_HIFCR_CFEIF7_Pos) | (0xF << DMA_HIFCR_CDMEIF7_Pos));
    }

    // DMA2 Streams
    else if (pDMAHandle->pDMAxStream == DMA2_Stream0) {
        DMA2->LIFCR |= ((1 << DMA_LIFCR_CFEIF0_Pos) | (0xF << DMA_LIFCR_CDMEIF0_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream1) {
        DMA2->LIFCR |= ((1 << DMA_LIFCR_CFEIF1_Pos) | (0xF << DMA_LIFCR_CDMEIF1_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream2) {
        DMA2->LIFCR |= ((1 << DMA_LIFCR_CFEIF2_Pos) | (0xF << DMA_LIFCR_CDMEIF2_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream3) {
        DMA2->LIFCR |= ((1 << DMA_LIFCR_CFEIF3_Pos) | (0xF << DMA_LIFCR_CDMEIF3_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream4) {
        DMA2->HIFCR |= ((1 << DMA_HIFCR_CFEIF4_Pos) | (0xF << DMA_HIFCR_CDMEIF4_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream5) {
        DMA2->HIFCR |= ((1 << DMA_HIFCR_CFEIF5_Pos) | (0xF << DMA_HIFCR_CDMEIF5_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream6) {
        DMA2->HIFCR |= ((1 << DMA_HIFCR_CFEIF6_Pos) | (0xF << DMA_HIFCR_CDMEIF6_Pos));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream7) {
        DMA2->HIFCR |= ((1 << DMA_HIFCR_CFEIF7_Pos) | (0xF << DMA_HIFCR_CDMEIF7_Pos));
    }
}

__weak void DMA_ApplicationCallback(DMA_Handle_t *pDMAHandle, DMA_Flag_e Flag) {}

uint8_t check_interrupt_enabled(DMA_Handle_t *pDMAHandle, DMA_Interrupt_e Interrupt) {
    switch (Interrupt) {
        case DMA_InterruptHalfTransfer:
            return (pDMAHandle->pDMAxStream->CR & (1 << DMA_SxCR_HTIE_Pos)) > 0;
        case DMA_InterruptTransferComplete:
            return (pDMAHandle->pDMAxStream->CR & (1 << DMA_SxCR_TCIE_Pos)) > 0;
        case DMA_InterruptTransferError:
            return (pDMAHandle->pDMAxStream->CR & (1 << DMA_SxCR_TEIE_Pos)) > 0;
        case DMA_InterruptDirectModeError:
            return (pDMAHandle->pDMAxStream->CR & (1 << DMA_SxCR_DMEIE_Pos)) > 0;
        case DMA_InterruptFIFOOverrun:
            return (pDMAHandle->pDMAxStream->FCR & (1 << DMA_SxFCR_FEIE_Pos)) > 0;
        default: return 0;
    }
}

uint8_t get_interrupt_flag(DMA_Handle_t *pDMAHandle, DMA_Interrupt_e Interrupt) {
    if (pDMAHandle->pDMAxStream == DMA1_Stream0) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA1->LISR & (1 << DMA_LISR_FEIF0_Pos)) > 0;
        return (DMA1->LISR & (1 << (2 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream1) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA1->LISR & (1 << DMA_LISR_FEIF1_Pos)) > 0;
        return (DMA1->LISR & (1 << (8 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream2) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA1->LISR & (1 << DMA_LISR_FEIF2_Pos)) > 0;
        return (DMA1->LISR & (1 << (18 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream3) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA1->LISR & (1 << DMA_LISR_FEIF3_Pos)) > 0;
        return (DMA1->LISR & (1 << (24 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream4) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA1->HISR & (1 << DMA_HISR_FEIF4_Pos)) > 0;
        return (DMA1->HISR & (1 << (2 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream5) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA1->HISR & (1 << DMA_HISR_FEIF5_Pos)) > 0;
        return (DMA1->HISR & (1 << (8 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream6) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA1->HISR & (1 << DMA_HISR_FEIF6_Pos)) > 0;
        return (DMA1->HISR & (1 << (18 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream7) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA1->HISR & (1 << DMA_HISR_FEIF7_Pos)) > 0;
        return (DMA1->HISR & (1 << (24 + Interrupt))) > 0;
    }
    // DMA2 Streams
    else if (pDMAHandle->pDMAxStream == DMA2_Stream0) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA2->LISR & (1 << DMA_LISR_FEIF0_Pos)) > 0;
        return (DMA2->LISR & (1 << (2 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream1) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA2->LISR & (1 << DMA_LISR_FEIF1_Pos)) > 0;
        return (DMA2->LISR & (1 << (8 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream2) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA2->LISR & (1 << DMA_LISR_FEIF2_Pos)) > 0;
        return (DMA2->LISR & (1 << (18 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream3) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA2->LISR & (1 << DMA_LISR_FEIF3_Pos)) > 0;
        return (DMA2->LISR & (1 << (24 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream4) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA2->HISR & (1 << DMA_HISR_FEIF4_Pos)) > 0;
        return (DMA2->HISR & (1 << (2 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream5) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA2->HISR & (1 << DMA_HISR_FEIF5_Pos)) > 0;
        return (DMA2->HISR & (1 << (8 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream6) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA2->HISR & (1 << DMA_HISR_FEIF6_Pos)) > 0;
        return (DMA2->HISR & (1 << (18 + Interrupt))) > 0;
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream7) {
        if (Interrupt == DMA_InterruptFIFOOverrun) return (DMA2->HISR & (1 << DMA_HISR_FEIF7_Pos)) > 0;
        return (DMA2->HISR & (1 << (24 + Interrupt))) > 0;
    }

    return 0; // Invalid stream
}

void clear_interrupt_flag(DMA_Handle_t *pDMAHandle, DMA_Interrupt_e Interrupt) {
    if (pDMAHandle->pDMAxStream == DMA1_Stream0) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA1->LIFCR |= (1 << DMA_LIFCR_CFEIF0_Pos);
        else DMA1->LIFCR |= (1 << (2 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream1) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA1->LIFCR |= (1 << DMA_LIFCR_CFEIF1_Pos);
        else DMA1->LIFCR |= (1 << (8 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream2) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA1->LIFCR |= (1 << DMA_LIFCR_CFEIF2_Pos);
        else DMA1->LIFCR |= (1 << (18 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream3) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA1->LIFCR |= (1 << DMA_LIFCR_CFEIF3_Pos);
        else DMA1->LIFCR |= (1 << (24 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream4) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA1->HIFCR |= (1 << DMA_HIFCR_CFEIF4_Pos);
        else DMA1->HIFCR |= (1 << (2 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream5) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA1->HIFCR |= (1 << DMA_HIFCR_CFEIF5_Pos);
        else DMA1->HIFCR |= (1 << (8 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream6) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA1->HIFCR |= (1 << DMA_HIFCR_CFEIF6_Pos);
        else DMA1->HIFCR |= (1 << (18 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA1_Stream7) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA1->HIFCR |= (1 << DMA_HIFCR_CFEIF7_Pos);
        else DMA1->HIFCR |= (1 << (24 + Interrupt));
    }
    // DMA2 Streams
    else if (pDMAHandle->pDMAxStream == DMA2_Stream0) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA2->LIFCR |= (1 << DMA_LIFCR_CFEIF0_Pos);
        else DMA2->LIFCR |= (1 << (2 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream1) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA2->LIFCR |= (1 << DMA_LIFCR_CFEIF1_Pos);
        else DMA2->LIFCR |= (1 << (8 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream2) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA2->LIFCR |= (1 << DMA_LIFCR_CFEIF2_Pos);
        else DMA2->LIFCR |= (1 << (18 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream3) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA2->LIFCR |= (1 << DMA_LIFCR_CFEIF3_Pos);
        else DMA2->LIFCR |= (1 << (24 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream4) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA2->HIFCR |= (1 << DMA_HIFCR_CFEIF4_Pos);
        else DMA2->HIFCR |= (1 << (2 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream5) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA2->HIFCR |= (1 << DMA_HIFCR_CFEIF5_Pos);
        else DMA2->HIFCR |= (1 << (8 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream6) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA2->HIFCR |= (1 << DMA_HIFCR_CFEIF6_Pos);
        else DMA2->HIFCR |= (1 << (18 + Interrupt));
    } else if (pDMAHandle->pDMAxStream == DMA2_Stream7) {
        if (Interrupt == DMA_InterruptFIFOOverrun) DMA2->HIFCR |= (1 << DMA_HIFCR_CFEIF7_Pos);
        else DMA2->HIFCR |= (1 << (24 + Interrupt));
    }
}
