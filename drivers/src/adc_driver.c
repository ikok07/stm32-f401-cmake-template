//
// Created by Kok on 7/6/25.
//

#include "adc_driver.h"

#include "stm32f4xx.h"

#include <commons.h>
#include <string.h>
#include <systick_driver.h>

static ADC_Error_e check_single_channel_only(ADC_Handle_t *pADCHandle);
static uint8_t check_interrupt_enabled(ADC_Interrupt_e Interrupt);
static uint8_t get_interrupt_flag(ADC_Interrupt_e Interrupt);

static void set_regular_channel_sequence(ADC_Channel_e *RegularChannels, uint8_t ChannelCount);
static void set_injected_channel_sequence(ADC_Channel_e *InjectedChannels, uint8_t ChannelCount);

static void enable_adc_dma(ADC_Handle_t *pADCHandle);
ADC_Error_e configure_adc_dma(ADC_Handle_t *pADCHandle);

/**
 * @brief Controls the peripheral's clock
 * @param Enabled If the peripheral's clock should be enabled
 */
void ADC_PeriClockControl(uint8_t Enabled) {
    if (Enabled) {
        RCC->APB2ENR |= (1 << RCC_APB2ENR_ADC1EN_Pos);
    } else {
        RCC->APB2ENR &=~ (1 << RCC_APB2ENR_ADC1EN_Pos);
    }
}

/**
 * @brief Enables or disables the ADC peripheral
 * @param Enabled If the peripheral should be enabled
 */
void ADC_PeripheralControl(uint8_t Enabled) {
    if (Enabled) {
        ADC1->CR2 |= (1 << ADC_CR2_ADON_Pos);
    } else {
        ADC1->CR2 &=~ (1 << ADC_CR2_ADON_Pos);
    }
}

uint8_t ADC_IsRegularConversionEnabled() {
    return (ADC1->CR2 & (1 << ADC_CR2_SWSTART_Pos)) > 0;
}

uint8_t ADC_IsInjectedConversionEnabled() {
    return (ADC1->CR2 & (1 << ADC_CR2_JSWSTART_Pos)) > 0;
}

/**
 * @brief Initializes the ADC peripheral
 * @param pADCHandle ADC handle
 */
ADC_Error_e ADC_Init(ADC_Handle_t *pADCHandle) {
    if (pADCHandle->Config.WatchdogConfig.WatchdogHigherThreshold > ADC_WATCHDOG_MAX_THRES || pADCHandle->Config.WatchdogConfig.WatchdogLowerThreshold < ADC_WATCHDOG_MIN_THRES) {
        return ADC_ErrWatchdogThresInvalid;
    }

    if (
        pADCHandle->Config.SampledRegularChannelsSequenceLength < 0 ||
        pADCHandle->Config.SampledRegularChannelsSequenceLength > ADC_REGULAR_CHAN_SEQUENCE_LEN ||
        pADCHandle->Config.SampledInjectedChannelsSequenceLength < 0 ||
        pADCHandle->Config.SampledInjectedChannelsSequenceLength > ADC_INJECTED_CHAN_SEQUENCE_LEN
    ) {
        return ADC_ErrInvalidSequenceLength;
    }

    // Resolution
    ADC1->CR1 |= (pADCHandle->Config.Resolution << ADC_CR1_RES_Pos);

    // Discontinuous channel count
    ADC1->CR1 |= (pADCHandle->Config.DiscontinuousChannelCount << ADC_CR1_DISCNUM_Pos);

    // Discontinuous mode (Regular)
    ADC1->CR1 |= (pADCHandle->Config.DiscModeRegularChannels << ADC_CR1_DISCEN_Pos);

    // Discontinuous mode (Injected)
    ADC1->CR1 |= (pADCHandle->Config.DiscModeInjectedChannels << ADC_CR1_JDISCEN_Pos);

    // Automatic injected group conversion
    ADC1->CR1 |= (pADCHandle->Config.AutoInjectedGroupConversion << ADC_CR1_JAUTO_Pos);

    // Scan mode
    ADC1->CR1 |= (pADCHandle->Config.ScanModeEnabled << ADC_CR1_SCAN_Pos);

    // Watchdog
    if (pADCHandle->Config.WatchdogConfig.WatchdogMode < ADC_WatchdogAllRegularChannels) {
        // Select channel for the watchdog
        ADC1->CR1 |= (pADCHandle->Config.WatchdogConfig.WatchdogChannel << ADC_CR1_AWDCH_Pos);
    }

    // Configure watchdog mode
    switch (pADCHandle->Config.WatchdogConfig.WatchdogMode) {
        case ADC_WatchdogSingleChannel:
            ADC1->CR1 |= (1 << ADC_CR1_AWDSGL_Pos) | (1 << ADC_CR1_AWDEN_Pos) | (1 << ADC_CR1_JAWDEN_Pos);
        break;
        case ADC_WatchdogSingleRegularChannel:
            ADC1->CR1 |= (1 << ADC_CR1_AWDSGL_Pos) | (1 << ADC_CR1_AWDEN_Pos);
        break;
        case ADC_WatchdogSingleInjectedChannel:
            ADC1->CR1 |= (1 << ADC_CR1_AWDSGL_Pos) | (1 << ADC_CR1_JAWDEN_Pos);
        break;
        case ADC_WatchdogAllChannels:
            ADC1->CR1 |= (1 << ADC_CR1_AWDEN_Pos) | (1 << ADC_CR1_JAWDEN_Pos);
        break;
        case ADC_WatchdogAllRegularChannels:
            ADC1->CR1 |= (1 << ADC_CR1_AWDEN_Pos);
        break;
        case ADC_WatchdogAllInjectedChannels:
            ADC1->CR1 |= (1 << ADC_CR1_JAWDEN_Pos);
        break;
        default:
            // No watchdog enabled
        break;
    }

    ADC1->HTR = pADCHandle->Config.WatchdogConfig.WatchdogHigherThreshold & 0xFFF;
    ADC1->LTR = pADCHandle->Config.WatchdogConfig.WatchdogLowerThreshold & 0xFFF;

    // Regular channel external trigger & event
    ADC1->CR2 |= (pADCHandle->Config.RegularExternalTrigger << ADC_CR2_EXTEN_Pos);
    ADC1->CR2 |= (pADCHandle->Config.RegularExternalEvent << ADC_CR2_EXTSEL_Pos);

    // Injected channel external trigger & event
    ADC1->CR2 |= (pADCHandle->Config.InjectedExternalTrigger << ADC_CR2_JEXTEN_Pos);
    ADC1->CR2 |= (pADCHandle->Config.InjectedExternalEvent << ADC_CR2_JEXTSEL_Pos);

    // Data alignment
    ADC1->CR2 |= (pADCHandle->Config.DataAlignment << ADC_CR2_ALIGN_Pos);

    // Continuous mode
    ADC1->CR2 |= (pADCHandle->Config.ContinuousModeEnabled << ADC_CR2_CONT_Pos);

    // Sampling times
    for (int i = 0; i < ADC_TOTAL_CHAN_COUNT; i++) {
        if (i < 10) {
            ADC1->SMPR2 |= (pADCHandle->Config.SamplingTimes[i] << (i * 3));
        } else {
            ADC1->SMPR1 |= (pADCHandle->Config.SamplingTimes[i] << ((i - 10) * 3));
        }
    }

    // ADC Clock Prescaler
    ADC->CCR |= (pADCHandle->Config.Prescaler << ADC_CCR_ADCPRE_Pos);

    // Temperature sensor
    ADC->CCR |= (pADCHandle->Config.TEMPEnabled << ADC_CCR_TSVREFE_Pos);

    // VBAT
    ADC->CCR |= (pADCHandle->Config.VBATEnabled << ADC_CCR_VBATE_Pos);

    // Injected channel offsets
    for (int i = 0; i < ADC_INJECTED_CHAN_COUNT; i++) {
        switch (i) {
            case 0:
                ADC1->JOFR1 = pADCHandle->Config.InjectedChanOffsets[i] & 0xFFF;
            break;
            case 1:
                ADC1->JOFR2 = pADCHandle->Config.InjectedChanOffsets[i] & 0xFFF;
            break;
            case 2:
                ADC1->JOFR3 = pADCHandle->Config.InjectedChanOffsets[i] & 0xFFF;
            break;
            case 3:
                ADC1->JOFR4 = pADCHandle->Config.InjectedChanOffsets[i] & 0xFFF;
            break;
            default: break;
        }
    }

    // Regular channel sequence
    set_regular_channel_sequence(pADCHandle->Config.SampledRegularChannelsSequence, pADCHandle->Config.SampledRegularChannelsSequenceLength);

    // Injected channel sequence
    set_injected_channel_sequence(pADCHandle->Config.SampledInjectedChannelsSequence, pADCHandle->Config.SampledInjectedChannelsSequenceLength);

    // Injected channel sequence length
    ADC1->JSQR |= ((pADCHandle->Config.SampledInjectedChannelsSequenceLength - 1) << ADC_JSQR_JL_Pos);

    // Setup DMA (if multiple channels setup)
    uint8_t totalConfiguredChannels = pADCHandle->Config.SampledInjectedChannelsSequenceLength + pADCHandle->Config.SampledRegularChannelsSequenceLength;
    if (totalConfiguredChannels > 1 || pADCHandle->Config.DMAConfig.Enabled) enable_adc_dma(pADCHandle);

    return ADC_ErrOK;
}

/**
 * @brief De-Initializes the ADC peripheral
 * @param pADCHandle ADC handle
 */
void ADC_DeInit(ADC_Handle_t *pADCHandle) {
    pADCHandle->ITState.pBuffer = 0;
    pADCHandle->ITState.Len = 0;

    RCC->APB2RSTR |= (1 << RCC_APB2RSTR_ADCRST_Pos);
    DMA_IRQDisable(&pADCHandle->DMAState.DMAHandle);
    DMA_DeInit(&pADCHandle->DMAState.DMAHandle);
}

/**
 * @brief Updates the regular channel sequence. This method can also be used when the ADC is turned on.
 * @param RegularChannels The channels which should be added to the regular channel sequence
 * @param ChannelCount The number of the added channels
 */
ADC_Error_e ADC_UpdateRegularSequence(ADC_Handle_t *pADCHandle, ADC_Channel_e *RegularChannels, uint8_t ChannelCount) {
    if (ChannelCount > ADC_TOTAL_CHAN_COUNT) return ADC_ErrInvalidChannelCount;

    // Modify sequence
    set_regular_channel_sequence(RegularChannels, ChannelCount);

    // Update config
    memcpy(pADCHandle->Config.SampledRegularChannelsSequence, RegularChannels, ChannelCount);
    pADCHandle->Config.SampledRegularChannelsSequenceLength = ChannelCount;

    // Reset DMA
    ADC_RestartDMA(pADCHandle);

    // Restart conversion
    ADC1->CR2 |= (1 << ADC_CR2_SWSTART_Pos);

    return ADC_ErrOK;
}

ADC_Error_e ADC_UpdateInjectedSequence(ADC_Handle_t *pADCHandle, ADC_Channel_e *InjectedChannels, uint8_t ChannelCount) {
    if (ChannelCount > ADC_TOTAL_CHAN_COUNT) return ADC_ErrInvalidChannelCount;

    // Modify sequence
    set_injected_channel_sequence(InjectedChannels, ChannelCount);

    // Update config
    memcpy(pADCHandle->Config.SampledInjectedChannelsSequence, InjectedChannels, ChannelCount);
    pADCHandle->Config.SampledInjectedChannelsSequenceLength = ChannelCount;

    // Restart conversion
    ADC1->CR2 |= (1 << ADC_CR2_JSWSTART_Pos);

    return ADC_ErrOK;
}

/**
 * @brief Reads converted data from a single channel
 * @note \b WARNING: This method is blocking and it is only used when there is one configured regular or injected channel!
 * @param pADCHandle ADC handle
 * @param Channel The desired channel from which to read
 * @param pBuffer The buffer where to store the converted data
 */
ADC_Error_e ADC_ReadSingleChannel(ADC_Handle_t *pADCHandle, uint16_t *pBuffer) {
    if (!(ADC1->CR2 & (1 << ADC_CR2_ADON_Pos))) return ADC_ErrPerNotEnabled;

    ADC_Error_e err = ADC_ErrOK;
    uint8_t regularChanLen = pADCHandle->Config.SampledRegularChannelsSequenceLength;
    uint8_t regularChanMode = regularChanLen > 0 ? 1 : 0;
    if ((err = check_single_channel_only(pADCHandle)) != ADC_ErrOK) return err;

    if (regularChanMode ? ADC_IsRegularConversionEnabled() : ADC_IsInjectedConversionEnabled()) return ADC_ErrBusy;

    // Start conversion
    if (regularChanMode) ADC1->CR2 |= (1 << ADC_CR2_SWSTART_Pos);
    else ADC1->CR2 |= (1 << ADC_CR2_JSWSTART_Pos);

    WAIT_WITH_TIMEOUT(!(ADC1->SR & (1 << (regularChanMode ? ADC_SR_EOC_Pos : ADC_SR_JEOC_Pos))), ADC_ErrTimeout, ADC_TIMEOUT_MS);

    // Read converted value
    *pBuffer = regularChanMode ? ADC1->DR : ADC1->JDR1;

    return err;
}

/**
 * @brief Reads converted data from a single channel using interrupts
 * @param pADCHandle ADC handle
 * @param pBuffer The buffer where to store the converted data
 */
ADC_Error_e ADC_ReadSingleChannelIT(ADC_Handle_t *pADCHandle, uint16_t *pBuffer) {
    if (pADCHandle->Config.ContinuousModeEnabled) return ADC_ErrContModeNotAllowed;

    ADC_Error_e err;
    uint8_t regularChanLen = pADCHandle->Config.SampledRegularChannelsSequenceLength;
    uint8_t regularChanMode = regularChanLen > 0 ? 1 : 0;
    if ((err = check_single_channel_only(pADCHandle)) != ADC_ErrOK) return err;

    if (regularChanMode ? ADC_IsRegularConversionEnabled() : ADC_IsInjectedConversionEnabled()) return ADC_ErrBusy;

    pADCHandle->ITState.pBuffer = pBuffer;
    pADCHandle->ITState.Len = 1;

    // Start conversion
    if (regularChanMode) ADC1->CR2 |= (1 << ADC_CR2_SWSTART_Pos);
    else ADC1->CR2 |= (1 << ADC_CR2_JSWSTART_Pos);

    return ADC_ErrOK;
}

/**
 * @brief Configures the ADC to wait for external trigger in order to sample data from a single channel
 * @param pADCHandle ADC Handle
 * @param pBuffer The buffer where to store the data
 */
ADC_Error_e ADC_ConfigureExternalTriggerIT(ADC_Handle_t *pADCHandle, uint16_t *pBuffer, uint8_t ChannelCount) {
    if (ChannelCount > ADC_TOTAL_CHAN_COUNT) return ADC_ErrInvalidChannelCount;
    if (pADCHandle->Config.ContinuousModeEnabled) return ADC_ErrContModeNotAllowed;

    ADC_Error_e err;
    uint8_t regularChanLen = pADCHandle->Config.SampledRegularChannelsSequenceLength;
    uint8_t regularChanMode = regularChanLen > 0 ? 1 : 0;
    if ((err = check_single_channel_only(pADCHandle)) != ADC_ErrOK) return err;

    if (regularChanMode ? ADC_IsRegularConversionEnabled() : ADC_IsInjectedConversionEnabled()) return ADC_ErrBusy;

    pADCHandle->ITState.pBuffer = pBuffer;
    pADCHandle->ITState.Len = ChannelCount;

    return ADC_ErrOK;
}

/**
 * @brief Reads converted data from a multiple \b INJECTED channels using interrupts
 * @param pADCHandle ADC Handle
 * @param pBuffer Buffer where to store the converted data
 * @param ChannelCount The desired count of channels to read
 */
ADC_Error_e ADC_ReadInjectedChannelsIT(ADC_Handle_t *pADCHandle, uint16_t *pBuffer, uint8_t ChannelCount) {
    if (ChannelCount > ADC_TOTAL_CHAN_COUNT) return ADC_ErrInvalidChannelCount;

    uint8_t injectedChanLen = pADCHandle->Config.SampledInjectedChannelsSequenceLength;
    if (injectedChanLen == 0) return ADC_ErrNoChannelInSequence;

    pADCHandle->ITState.pBuffer = pBuffer;
    pADCHandle->ITState.Len = ChannelCount;

    ADC1->CR2 |= (1 << ADC_CR2_JSWSTART_Pos);

    return ADC_ErrOK;
}

/**
 * @brief Configures ADC's DMA controller in order to be ready to transfer data when conversion is available
 * @param pADCHandle
 */
ADC_Error_e ADC_ConfigureDMA(ADC_Handle_t *pADCHandle) {
    uint8_t regularChanLen = pADCHandle->Config.SampledRegularChannelsSequenceLength;
    if (regularChanLen == 0) return ADC_ErrNoChannelInSequence;
    if ((ADC1->CR2 & (1 << ADC_CR2_DMA_Pos)) == 0) return ADC_ErrDMADisabled;
    if (ADC_IsRegularConversionEnabled()) return ADC_ErrBusy;

    ADC_Error_e err = ADC_ErrOK;
    if ((err = configure_adc_dma(pADCHandle)) != ADC_ErrOK) return err;

    DMA_StartTransaction(&pADCHandle->DMAState.DMAHandle);
    return err;
}

/**
 * @brief Stops all conversions for the regular channels and also stops the DMA transactions
 * @param pADCHandle ADC handle
 */
void ADC_StopDMA(ADC_Handle_t *pADCHandle) {
    // Stop conversions
    ADC1->CR2 &=~ (1 << ADC_CR2_CONT_Pos);
    pADCHandle->Config.ContinuousModeEnabled = DISABLE;

    // Stop DMA transaction
    DMA_StopTransaction(&pADCHandle->DMAState.DMAHandle);
}

/**
 * @brief Restarts the DMA transactions. Used in order to handle ADC errors
 * @param pADCHandle ADC Handle
 */
ADC_Error_e ADC_RestartDMA(ADC_Handle_t *pADCHandle) {
    DMA_StopTransaction(&pADCHandle->DMAState.DMAHandle);
    ADC_Error_e err = ADC_ErrOK;
    if ((err = configure_adc_dma(pADCHandle)) != ADC_ErrOK) return err;
    DMA_StartTransaction(&pADCHandle->DMAState.DMAHandle);

    return err;
}

/**
 * @brief Starts ADC's \b REGULAR \b CHANNELS conversion and activates the DMA. This method works for both single and multiple regular channels
 * @param pADCHandle ADC handle
 */
ADC_Error_e ADC_StartDMA(ADC_Handle_t *pADCHandle) {
    ADC_Error_e err = ADC_ConfigureDMA(pADCHandle);
    if (err != ADC_ErrOK) return err;

    ADC1->CR2 |= (1 << ADC_CR2_SWSTART_Pos);

    return err;
}

/**
 * @brief Enables the desired interrupts
 * @param EnabledInterrupts The desired interrupts to be enabled
 * @param Len The length of the desired interrupts
 */
void ADC_EnableInterrupts(ADC_Handle_t *pADCHandle, ADC_Interrupt_e *EnabledInterrupts, uint8_t Len) {
    while (Len > 0) {
        switch (*EnabledInterrupts++) {
            case ADC_InterruptEOC:
                ADC1->CR1 |= (1 << ADC_CR1_EOCIE_Pos);
            break;
            case ADC_InterruptJEOC:
                ADC1->CR1 |= (1 << ADC_CR1_JEOCIE_Pos);
            break;
            case ADC_InterruptAWD:
                ADC1->CR1 |= (1 << ADC_CR1_AWDIE_Pos);
            break;
            case ADC_InterruptOVR:
                ADC1->CR1 |= (1 << ADC_CR1_OVRIE_Pos);
            break;
            default: break;
        }
        Len--;
    }
}

/**
 * @brief Disables the desired interrupts
 * @param DisabledInterrupts The desired interrupts to be enabled
 * @param Len The length of the desired interrupts
 */
void ADC_DisableInterrupts(ADC_Interrupt_e *DisabledInterrupts, uint8_t Len) {
    while (Len > 0) {
        switch (*DisabledInterrupts++) {
            case ADC_InterruptEOC:
                ADC1->CR1 &=~ (1 << ADC_CR1_EOCIE_Pos);
            break;
            case ADC_InterruptJEOC:
                ADC1->CR1 &=~ (1 << ADC_CR1_JEOCIE_Pos);
            break;
            case ADC_InterruptAWD:
                ADC1->CR1 &=~ (1 << ADC_CR1_AWDIE_Pos);
            break;
            case ADC_InterruptOVR:
                ADC1->CR1 &=~ (1 << ADC_CR1_OVRIE_Pos);
            break;
            default: break;
        }
        Len--;
    }
}

/**
 * @brief Disables all ADC interrupts
 */
void ADC_DisableAllInterrupts() {
    ADC1->CR1 &=~ (1 << ADC_CR1_EOCIE_Pos);
    ADC1->CR1 &=~ (1 << ADC_CR1_JEOCIE_Pos);
    ADC1->CR1 &=~ (1 << ADC_CR1_AWDIE_Pos);
    ADC1->CR1 &=~ (1 << ADC_CR1_OVRIE_Pos);
}

void ADC_IRQEnable(uint8_t IRQPriority) {
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, IRQPriority);
}

void ADC_IRQDisable() {
    NVIC_DisableIRQ(ADC_IRQn);
}

void ADC_IRQHandling(ADC_Handle_t *pADCHandle) {
    if (check_interrupt_enabled(ADC_InterruptEOC) && get_interrupt_flag(ADC_InterruptEOC)) {
        // Regular conversion complete
        *pADCHandle->ITState.pBuffer = ADC1->DR;
        ADC_ApplicationCallback(pADCHandle, ADC_FlagEndOfRegularConversion);
    }

    if (check_interrupt_enabled(ADC_InterruptJEOC) && get_interrupt_flag(ADC_InterruptJEOC)) {
        // Injected conversion complete
        ADC1->SR &=~ (1 << ADC_SR_JEOC_Pos);

        if (pADCHandle->ITState.Len > 0) pADCHandle->ITState.pBuffer[0] = ADC1->JDR1;
        if (pADCHandle->ITState.Len > 1) pADCHandle->ITState.pBuffer[1] = ADC1->JDR2;
        if (pADCHandle->ITState.Len > 2) pADCHandle->ITState.pBuffer[2] = ADC1->JDR3;
        if (pADCHandle->ITState.Len > 3) pADCHandle->ITState.pBuffer[3] = ADC1->JDR4;
        ADC_ApplicationCallback(pADCHandle, ADC_FlagEndOfInjectedConversion);
    }

    if (check_interrupt_enabled(ADC_InterruptAWD) && get_interrupt_flag(ADC_InterruptAWD)) {
        // Watchdog triggered
        ADC1->SR &=~ (1 << ADC_SR_AWD_Pos);
        ADC_ApplicationCallback(pADCHandle, ADC_FlagAnalogWatchdog);
    }

    if (check_interrupt_enabled(ADC_InterruptOVR) && get_interrupt_flag(ADC_InterruptOVR)) {
        // Overrun
        ADC1->SR &=~ (1 << ADC_SR_OVR_Pos);
        ADC_RestartDMA(pADCHandle);
        ADC_ApplicationCallback(pADCHandle, ADC_FlagOverrun);
    }
}

__weak void ADC_ApplicationCallback(ADC_Handle_t *pADCHandle, ADC_Flag_e Flag) {};

ADC_Error_e check_single_channel_only(ADC_Handle_t *pADCHandle) {
    uint8_t regularChanLen = pADCHandle->Config.SampledRegularChannelsSequenceLength;
    uint8_t injectedChanLen = pADCHandle->Config.SampledInjectedChannelsSequenceLength;
    if (regularChanLen > 0) {
        if (regularChanLen > 1 || injectedChanLen > 0) return ADC_ErrMethodNotForCorrectChannelLength;
    } else if (injectedChanLen > 0) {
        if (regularChanLen > 0 || injectedChanLen > 1) return ADC_ErrMethodNotForCorrectChannelLength;
    } else if (regularChanLen == 0 || injectedChanLen == 0) {
        return ADC_ErrNoChannelInSequence;
    }

    return ADC_ErrOK;
}

uint8_t check_interrupt_enabled(ADC_Interrupt_e Interrupt) {
    switch (Interrupt) {
        case ADC_InterruptEOC:
            return (ADC1->CR1 & (1 << ADC_CR1_EOCIE_Pos)) > 0;
        case ADC_InterruptJEOC:
            return (ADC1->CR1 & (1 << ADC_CR1_JEOCIE_Pos)) > 0;
        case ADC_InterruptAWD:
            return (ADC1->CR1 & (1 << ADC_CR1_AWDIE_Pos)) > 0;
        case ADC_InterruptOVR:
            return (ADC1->CR1 & (1 << ADC_CR1_OVRIE_Pos)) > 0;
        default: return 0;
    }
}

uint8_t get_interrupt_flag(ADC_Interrupt_e Interrupt) {
    switch (Interrupt) {
        case ADC_InterruptEOC:
            return (ADC1->SR & (1 << ADC_SR_EOC_Pos)) > 0;
        case ADC_InterruptJEOC:
            return (ADC1->SR & (1 << ADC_SR_JEOC_Pos)) > 0;
        case ADC_InterruptAWD:
            return (ADC1->SR & (1 << ADC_SR_AWD_Pos)) > 0;
        case ADC_InterruptOVR:
            return (ADC1->SR & (1 << ADC_SR_OVR_Pos)) > 0;
        default: return 0;
    }
}

void enable_adc_dma(ADC_Handle_t *pADCHandle) {
    ADC1->CR2 |= (1 << ADC_CR2_DMA_Pos);

    // Generate DMA requests as long as data is converted
    ADC1->CR2 |= (1 << ADC_CR2_DDS_Pos);

    uint8_t circModeEnabled = pADCHandle->Config.RegularExternalTrigger ? 1 : pADCHandle->Config.ContinuousModeEnabled;

    DMA_Config_t dmaConfig = {
        .Channel = DMA_Channel0,
        .Direction = DMA_DirPerToMem,
        .Priority = DMA_PriorityMedium,
        .CircularMode = circModeEnabled,
        .FlowController = DMA_FlowController,
        .DirectModeDisabled = DISABLE,
        .DoubleBufferMode = DISABLE,
        .MemoryBurstConfig = DMA_Burst1,
        .MemoryDataSize = DMA_DataSizeHalfWord,
        .PeripheralDataSize = DMA_DataSizeHalfWord,
        .MemoryIncrementMode = ENABLE,
        .PeripheralIncrementMode = DISABLE,
        .PeripheralBurstConfig = DMA_Burst1,
        .PeripheralIncrementOffsetSize = DMA_PINCOSPSIZE,
    };

    pADCHandle->DMAState.DMAHandle.pDMAxStream = DMA2_Stream4;
    pADCHandle->DMAState.DMAHandle.Config = dmaConfig;

    DMA_PeriClockControl(&pADCHandle->DMAState.DMAHandle, ENABLE);
    DMA_Init(&pADCHandle->DMAState.DMAHandle);

    if (pADCHandle->Config.DMAConfig.Enabled && pADCHandle->Config.DMAConfig.InterruptsLength > 0) {
        DMA_EnableInterrupts(&pADCHandle->DMAState.DMAHandle, pADCHandle->Config.DMAConfig.EnabledInterrupts, pADCHandle->Config.DMAConfig.InterruptsLength);
        DMA_IRQEnable(&pADCHandle->DMAState.DMAHandle, pADCHandle->Config.DMAConfig.InterruptsPriority);
    }
}

ADC_Error_e configure_adc_dma(ADC_Handle_t *pADCHandle) {
    uint8_t channelCount = pADCHandle->Config.SampledRegularChannelsSequenceLength;
    DMA_Error_e err = DMA_ErrOK;
    if ((err = DMA_ConfigureSingleBufferTransfer(&pADCHandle->DMAState.DMAHandle, (uint32_t)&ADC1->DR, (uint32_t)&pADCHandle->DMAState.Samples, channelCount)) != DMA_ErrOK) return ADC_ErrDMAFailed;

    return err == DMA_ErrOK ? ADC_ErrOK : ADC_ErrDMAFailed;
}

void set_regular_channel_sequence(ADC_Channel_e *RegularChannels, uint8_t ChannelCount) {
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 0;
    for (int i = 0; i < ChannelCount; i++) {
        if (i < 6) {
            ADC1->SQR3 |= (RegularChannels[i] << (i * 5));
        } else if (i < 12) {
            ADC1->SQR2 |= (RegularChannels[i] << ((i - 6) * 5));
        } else {
            ADC1->SQR1 |= (RegularChannels[i] << ((i - 12) * 5));
        }
    }
    ADC1->SQR1 |= ((ChannelCount > 0 ? (ChannelCount - 1) : 0) << ADC_SQR1_L_Pos);
}

void set_injected_channel_sequence(ADC_Channel_e *InjectedChannels, uint8_t ChannelCount) {
    ADC1->JSQR = 0;
    for (int i = 0; i < ChannelCount; i++) {
        // Length == 3 => JSQ1,2,3,4; Length == 2 => JSQ2,3,4, etc...
        ADC1->JSQR |= (InjectedChannels[i] << ((i + (4 - ChannelCount)) * 5));
    }
}