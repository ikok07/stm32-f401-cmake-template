//
// Created by Kok on 7/6/25.
//

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include <dma_driver.h>
#include <stdint.h>

/* ------------ MACROS ------------ */

#define ADC_TIMEOUT_MS                      3000

#define ADC_TOTAL_CHAN_COUNT                19
#define ADC_INJECTED_CHAN_COUNT             4

#define ADC_REGULAR_CHAN_SEQUENCE_LEN       16
#define ADC_INJECTED_CHAN_SEQUENCE_LEN      4

#define ADC_WATCHDOG_MIN_THRES                  0x00
#define ADC_WATCHDOG_MAX_THRES                  0xFFF


/* ------------ ERROR CODES ------------ */

typedef enum {
    ADC_ErrOK,
    ADC_ErrBusy,
    ADC_ErrTimeout,
    ADC_ErrPerNotEnabled,
    ADD_ErrConvNotEnabled,
    ADC_ErrWatchdogThresInvalid,
    ADC_ErrInvalidSequenceLength,
    ADC_ErrInvalidChannel,
    ADC_ErrInvalidChannelCount,
    ADC_ErrMethodNotForCorrectChannelLength,
    ADC_ErrContModeNotAllowed,
    ADC_ErrNoChannelInSequence,
    ADC_ErrDMADisabled,
    ADC_ErrDMAFailed
} ADC_Error_e;

/* ------------ CONFIG STRUCTURES ------------ */

typedef enum {
    ADC_Channel0,
    ADC_Channel1,
    ADC_Channel2,
    ADC_Channel3,
    ADC_Channel4,
    ADC_Channel5,
    ADC_Channel6,
    ADC_Channel7,
    ADC_Channel8,
    ADC_Channel9,
    ADC_Channel10,
    ADC_Channel11,
    ADC_Channel12,
    ADC_Channel13,
    ADC_Channel14,
    ADC_Channel15,
    ADC_Channel16,
    ADC_Channel17,
    ADC_Channel18,
} ADC_Channel_e;

typedef enum {
    ADC_Resolution12bit,
    ADC_Resolution10bit,
    ADC_Resolution8bit,
    ADC_Resolution6bit,
} ADC_Resolution_e;

typedef enum {
    ADC_DiscMode1Channel,
    ADC_DiscMode2Channel,
    ADC_DiscMode3Channel,
    ADC_DiscMode4Channel,
    ADC_DiscMode5Channel,
    ADC_DiscMode6Channel,
    ADC_DiscMode7Channel,
    ADC_DiscMode8Channel,
} ADC_DiscModeChanCount_e;

typedef enum {
    ADC_WatchdogDisabled,
    ADC_WatchdogSingleChannel,
    ADC_WatchdogSingleRegularChannel,
    ADC_WatchdogSingleInjectedChannel,
    ADC_WatchdogAllRegularChannels,
    ADC_WatchdogAllInjectedChannels,
    ADC_WatchdogAllChannels,
} ADC_WatchdogMode_e;

typedef enum {
    ADC_WatchdogChan0,
    ADC_WatchdogChan1,
    ADC_WatchdogChan2,
    ADC_WatchdogChan3,
    ADC_WatchdogChan4,
    ADC_WatchdogChan5,
    ADC_WatchdogChan6,
    ADC_WatchdogChan7,
    ADC_WatchdogChan8,
    ADC_WatchdogChan9,
    ADC_WatchdogChan10,
    ADC_WatchdogChan11,
    ADC_WatchdogChan12,
    ADC_WatchdogChan13,
    ADC_WatchdogChan14,
    ADC_WatchdogChan15,
    ADC_WatchdogChan16,
    ADC_WatchdogChan17,
    ADC_WatchdogChan18,
} ADC_WatchdogChan_e;

typedef enum {
    ADC_ExternalTriggerDisabled,
    ADC_ExternalTriggerREdge,
    ADC_ExternalTriggerFEdge,
    ADC_ExternalTriggerRFEdge,
} ADC_ExternalTrigger_e;

typedef enum {
    ADC_RegularExternalEventTIM1CC1,
    ADC_RegularExternalEventTIM1CC2,
    ADC_RegularExternalEventTIM1CC3,
    ADC_RegularExternalEventTIM2CC2,
    ADC_RegularExternalEventTIM2CC3,
    ADC_RegularExternalEventTIM2CC4,
    ADC_RegularExternalEventTIM2TRGO,
    ADC_RegularExternalEventTIM3CC1,
    ADC_RegularExternalEventTIM3TRGO,
    ADC_RegularExternalEventTIM4CC4,
    ADC_RegularExternalEventTIM5CC1,
    ADC_RegularExternalEventTIM5CC2,
    ADC_RegularExternalEventTIM5CC3,
    ADC_RegularExternalEventEXTI11 = 0xF,
} ADC_RegularExternalEvent_e;

typedef enum {
    ADC_InjectedExternalEventTIM1CC4,
    ADC_InjectedExternalEventTIM1TRGO,
    ADC_InjectedExternalEventTIM2CC1,
    ADC_InjectedExternalEventTIM2TRGO,
    ADC_InjectedExternalEventTIM3CC2,
    ADC_InjectedExternalEventTIM3CC4,
    ADC_InjectedExternalEventTIM4CC1,
    ADC_InjectedExternalEventTIM4CC2,
    ADC_InjectedExternalEventTIM4CC3,
    ADC_InjectedExternalEventTIM4TRGO,
    ADC_InjectedExternalEventTIM5CC4,
    ADC_InjectedExternalEventTIM5TRGO,
    ADC_InjectedExternalEventEXTI15 = 0xF,
} ADC_InjectedExternalEvent_e;

typedef enum {
    ADC_DataAlignRight,
    ADC_DataAlignLeft,
} ADC_DataAlign_e;

typedef enum {
    ADC_Sampling3Cycles,
    ADC_Sampling15Cycles,
    ADC_Sampling28Cycles,
    ADC_Sampling56Cycles,
    ADC_Sampling84Cycles,
    ADC_Sampling112Cycles,
    ADC_Sampling144Cycles,
    ADC_Sampling480Cycles,
} ADC_SamplingTime_e;

typedef enum {
    ADC_Prescaler2,
    ADC_Prescaler4,
    ADC_Prescaler6,
    ADC_Prescaler8,
} ADC_Prescaler_e;

typedef enum {
    ADC_InterruptEOC,
    ADC_InterruptJEOC,
    ADC_InterruptAWD,
    ADC_InterruptOVR,
} ADC_Interrupt_e;

typedef enum {
    ADC_FlagEndOfRegularConversion,
    ADC_FlagEndOfInjectedConversion,
    ADC_FlagAnalogWatchdog,
    ADC_FlagOverrun,
} ADC_Flag_e;

typedef enum {
    ADC_DMADataReady,
    ADC_DMADataMissing,
} ADC_DMADataStatus;

typedef struct {
    ADC_WatchdogMode_e WatchdogMode;
    ADC_WatchdogChan_e WatchdogChannel;         // If watchdog mode is for single channel
    uint16_t WatchdogHigherThreshold;
    uint16_t WatchdogLowerThreshold;
} ADC_WatchdogConfig_t;

typedef struct {
    uint8_t Enabled;                // When having more than one configured channel, the DMA is always enabled
    DMA_Interrupt_e *EnabledInterrupts;
    uint8_t InterruptsLength;
    uint8_t InterruptsPriority;
} ADC_DMAConfig_t;

typedef struct {
    ADC_Resolution_e Resolution;
    ADC_DiscModeChanCount_e DiscontinuousChannelCount;
    uint8_t DiscModeInjectedChannels;
    uint8_t DiscModeRegularChannels;
    uint8_t AutoInjectedGroupConversion;
    uint8_t ScanModeEnabled;
    ADC_WatchdogConfig_t WatchdogConfig;
    ADC_DMAConfig_t DMAConfig;
    ADC_ExternalTrigger_e RegularExternalTrigger;
    ADC_ExternalTrigger_e InjectedExternalTrigger;
    ADC_RegularExternalEvent_e RegularExternalEvent;
    ADC_InjectedExternalEvent_e InjectedExternalEvent;
    ADC_DataAlign_e DataAlignment;
    ADC_SamplingTime_e SamplingTimes[ADC_TOTAL_CHAN_COUNT];
    ADC_Prescaler_e Prescaler;
    uint8_t ContinuousModeEnabled;                  // Valid only for regular channels
    uint8_t VBATEnabled;
    uint8_t TEMPEnabled;
    uint16_t InjectedChanOffsets[ADC_INJECTED_CHAN_COUNT];
    ADC_Channel_e SampledRegularChannelsSequence[ADC_REGULAR_CHAN_SEQUENCE_LEN];            // Array of the desired regular channel sequence
    ADC_Channel_e SampledInjectedChannelsSequence[ADC_INJECTED_CHAN_SEQUENCE_LEN];          // Array of the desired injected channel sequence
    uint8_t SampledRegularChannelsSequenceLength;
    uint8_t SampledInjectedChannelsSequenceLength;
} ADC_Config_t;

typedef struct {
    uint16_t *pBuffer;
    uint8_t Len;
} ADC_ITState;

typedef struct {
    DMA_Handle_t DMAHandle;
    uint16_t Samples[ADC_TOTAL_CHAN_COUNT];
} ADC_DMAState;

typedef struct {
    ADC_ITState ITState;
    ADC_DMAState DMAState;
    ADC_Config_t Config;
} ADC_Handle_t;

/* ------------ METHODS ------------ */

/*
 * Peripheral control
 */
void ADC_PeriClockControl(uint8_t Enabled);
void ADC_PeripheralControl(uint8_t Enabled);

/*
 * Status
 */
uint8_t ADC_IsRegularConversionEnabled();
uint8_t ADC_IsInjectedConversionEnabled();

/*
 * Init and De-Init
 */
ADC_Error_e ADC_Init(ADC_Handle_t *pADCHandle);
void ADC_DeInit(ADC_Handle_t *pADCHandle);

/*
 * Sequence control
 */
ADC_Error_e ADC_UpdateRegularSequence(ADC_Handle_t *pADCHandle, ADC_Channel_e *RegularChannels, uint8_t ChannelCount);
ADC_Error_e ADC_UpdateInjectedSequence(ADC_Handle_t *pADCHandle, ADC_Channel_e *InjectedChannels, uint8_t ChannelCount);

/*
 * Data sampling
 */
ADC_Error_e ADC_ReadSingleChannel(ADC_Handle_t *pADCHandle, uint16_t *pBuffer);
ADC_Error_e ADC_ReadSingleChannelIT(ADC_Handle_t *pADCHandle, uint16_t *pBuffer);
ADC_Error_e ADC_ConfigureExternalTriggerIT(ADC_Handle_t *pADCHandle, uint16_t *pBuffer, uint8_t ChannelCount);
ADC_Error_e ADC_ReadInjectedChannelsIT(ADC_Handle_t *pADCHandle, uint16_t *pBuffer, uint8_t ChannelCount);

// DMA only
ADC_Error_e ADC_ConfigureDMA(ADC_Handle_t *pADCHandle);
ADC_Error_e ADC_StartDMA(ADC_Handle_t *pADCHandle);
void ADC_StopDMA(ADC_Handle_t *pADCHandle);
ADC_Error_e ADC_RestartDMA(ADC_Handle_t *pADCHandle);

/*
 * IRQ Handling
 */
void ADC_EnableInterrupts(ADC_Handle_t *pADCHandle, ADC_Interrupt_e *EnabledInterrupts, uint8_t Len);
void ADC_DisableInterrupts(ADC_Interrupt_e *DisabledInterrupts, uint8_t Len);
void ADC_DisableAllInterrupts();

void ADC_IRQEnable(uint8_t IRQPriority);
void ADC_IRQDisable();
void ADC_IRQHandling(ADC_Handle_t *pADCHandle);

/*
 * Application callback
 */
void ADC_ApplicationCallback(ADC_Handle_t *pADCHandle, ADC_Flag_e Flag);

#endif //ADC_DRIVER_H
