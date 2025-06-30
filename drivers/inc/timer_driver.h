//
// Created by Kok on 6/29/25.
//

#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include "stm32f4xx.h"

/* ------------ MACROS ------------ */

#define TIM_PCLK_EN(perIndex, offset)           do {\
                                                    if ((perIndex) == 1 || (perIndex) > 8) RCC->APB2ENR |= (1 << (offset));\
                                                    else RCC->APB1ENR |= (1 << (offset));\
                                                } while (0)

#define TIM_PCLK_DI(perIndex, offset)           do {\
                                                    if ((perIndex) == 1 || (perIndex) > 8) RCC->APB2ENR &=~ (1 << (offset));\
                                                    else RCC->APB1ENR &=~ (1 << (offset));\
                                                } while (0)

#define TIM_RESET(perIndex, offset)             do {\
                                                    if ((perIndex) == 1 || (perIndex) > 8) {\
                                                        RCC->APB2RSTR |= (1 << (offset));\
                                                        RCC->APB2RSTR &=~ (1 << (offset));\
                                                    }\
                                                    else {\
                                                        RCC->APB1RSTR |= (1 << (offset));\
                                                        RCC->APB1RSTR &=~ (1 << (offset));\
                                                    }\
                                                } while (0)

#define TIM_CC_EN(CCER, chan)                   do {\
                                                    if (chan == TIM_CaptureCompareChan1) CCER |= (1 << TIM_CCER_CC1E_Pos);\
                                                    else if (chan == TIM_CaptureCompareChan2) CCER |= (1 << TIM_CCER_CC2E_Pos);\
                                                    else if (chan == TIM_CaptureCompareChan3) CCER |= (1 << TIM_CCER_CC3E_Pos);\
                                                    else if (chan == TIM_CaptureCompareChan4) CCER |= (1 << TIM_CCER_CC4E_Pos);\
                                                } while (0)

#define TIM_CC_DI(CCER, chan)                   do {\
                                                    if (chan == TIM_CaptureCompareChan1) CCER &=~ (1 << TIM_CCER_CC1E_Pos);\
                                                    else if (chan == TIM_CaptureCompareChan2) CCER &=~ (1 << TIM_CCER_CC2E_Pos);\
                                                    else if (chan == TIM_CaptureCompareChan3) CCER &=~ (1 << TIM_CCER_CC3E_Pos);\
                                                    else if (chan == TIM_CaptureCompareChan4) CCER &=~ (1 << TIM_CCER_CC4E_Pos);\
                                                } while (0)

#define TIM_CC_POL_SET(CCER, chan, polarity)    do {\
                                                    if (chan == TIM_CaptureCompareChan1) CCER |= (polarity << TIM_CCER_CC1P_Pos);\
                                                    else if (chan == TIM_CaptureCompareChan2) CCER |= (polarity << TIM_CCER_CC2P_Pos);\
                                                    else if (chan == TIM_CaptureCompareChan3) CCER |= (polarity << TIM_CCER_CC3P_Pos);\
                                                    else if (chan == TIM_CaptureCompareChan4) CCER |= (polarity << TIM_CCER_CC4P_Pos);\
                                                } while (0)

/**
 * @TimerInterrupts
 */
#define TIM_IT_UPDATE    (1 << TIM_DIER_UIE_Pos)
#define TIM_IT_CC1       (1 << TIM_DIER_CC1IE_Pos)
#define TIM_IT_CC2       (1 << TIM_DIER_CC2IE_Pos)
#define TIM_IT_CC3       (1 << TIM_DIER_CC3IE_Pos)
#define TIM_IT_CC4       (1 << TIM_DIER_CC4IE_Pos)
#define TIM_IT_COM       (1 << TIM_DIER_COMIE_Pos)
#define TIM_IT_TRIGGER   (1 << TIM_DIER_TIE_Pos)
#define TIM_IT_BREAK     (1 << TIM_DIER_BIE_Pos)


/* ------------ ERROR CODES ------------ */

/* ------------ CONFIG STRUCTURES ------------ */

typedef enum {
    TIM_ARRPreloadDisabled,
    TIM_ARRPreloadEnabled,
} TIM_ARRPreload_e;

typedef enum {
    TIM_DirectionUp,
    TIM_DirectionDown,
} TIM_Direction_e;

typedef enum {
    TIM_CenterMode1,        /* Output compare interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set only when the counter is counting down */
    TIM_CenterMode2,        /* Output compare interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set only when the counter is counting up */
    TIM_CenterMode3,        /* Output compare interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set both when the counter is counting up or down */
} TIM_CenterModeSelection_e;

typedef enum {
    TIM_OnePulseDisabled,
    TIM_OnePulseEnabled,
} TIM_OnePulseMode_e;

typedef enum {
    TIM_UpdateReqSrcMode1,              /*These events can be:
                                            – Counter overflow/underflow
                                            – Setting the UG bit
                                            – Update generation through the slave mode controller
                                        */
    TIM_UpdateReqSrcMode2,              /* Only counter overflow/underflow generates an update interrupt or DMA request if enabled. */
} TIM_UpdateReqSrcMode_e;

typedef enum {
    TIM_UpdateEventEnabled,
    TIM_UpdateEventDisabled,
} TIM_UpdateEventToggle_e;

typedef enum {
    TIM_CaptureCompareChan1,
    TIM_CaptureCompareChan2,
    TIM_CaptureCompareChan3,
    TIM_CaptureCompareChan4,
} TIM_CaptureCompareChan_e;

typedef enum {
    TIM_CaptureComparePolarityHIGH,
    TIM_CaptureComparePolarityLOW,
    TIM_CaptureComparePolarityBOTH,             // Used in Input Compare mode
} TIM_CaptureComparePolarity_e;

typedef enum {
    TIM_OutputCompareMode1,             /* Frozen - The comparison between the output compare register and the
                                                    counter has no effect on the outputs
                                        */
    TIM_OutputCompareMode2,             /* Set channel X to active level on match */
    TIM_OutputCompareMode3,             /* Set channel X to inactive level on match */
    TIM_OutputCompareMode4,             /* Toggle channel X */
    TIM_OutputCompareMode5,             /* Force inactive level */
    TIM_OutputCompareMode6,             /* Force active level */
    TIM_OutputCompareMode7,             /*
                                            PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
                                            else inactive. In downcounting, channel 1 is inactive as long as
                                            TIMx_CNT>TIMx_CCR1 else active.
                                        */
    TIM_OutputCompareMode8,             /*
                                            PWM mode 2 - In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1
                                            else active. In downcounting, channel 1 is active as long as
                                            TIMx_CNT>TIMx_CCR1 else inactive.
                                        */
} TIM_OutputCompareMode_e;

typedef enum {
    TIM_OutputComparePreloadDisabled,               // TIMx_CCR1 can be written at anytime, the new value is taken in account immediately.
    TIM_OutputComparePreloadEnabled                 // Read/Write operations access the preload register. TIMx_CCR1 preload value is loaded in the active register at each update event.
} TIM_OutputComparePreloadToggle_e;

typedef enum {
    TIM_BreakPolarityLOW,
    TIM_BreakPolarityHIGH,
} TIM_BreakPolarity_e;

typedef enum {
    TIM_OffStateModeOutputDisabled,
    TIM_OffStateModeOutputInactive,
} TIM_OffStateMode_e;

typedef enum {
    TIM_DeadTimeClkDiv1,
    TIM_DeadTimeClkDiv2,
    TIM_DeadTimeClkDiv3,
    TIM_DeadTimeClkDiv4,
} TIM_DeadTimeClkDivision_e;

typedef enum {
    TIM_LockLevelOFF,
    TIM_LockLevel1,         // DTG bits in TIMx_BDTR register, OISx and OISxN bits in TIMx_CR2 register and BKE/BKP/AOE bits in TIMx_BDTR register can no longer be written.
    TIM_LockLevel2,         // LOCK Level 1 + CC Polarity bits (CCxP/CCxNP bits in TIMx_CCER register, as long as the related channel is configured in output through the CCxS bits) as well as OSSR and OSSI bits can no longer be written
    TIM_LockLevel3,         // LOCK Level 2 + CC Control bits (OCxM and OCxPE bits in TIMx_CCMRx registers, as long as the related channel is configured in output through the CCxS bits) can no longer be written.
} TIM_LockLevel_e;

typedef enum {
    TIM_FlagUpdate = TIM_SR_UIF_Pos,
    TIM_FlagCC1 = TIM_SR_CC1IF_Pos,
    TIM_FlagCC2 = TIM_SR_CC2IF_Pos,
    TIM_FlagCC3 = TIM_SR_CC3IF_Pos,
    TIM_FlagCC4 = TIM_SR_CC4IF_Pos,
    TIM_FlagCOM = TIM_SR_COMIF_Pos,
    TIM_FlagTrigger = TIM_SR_TIF_Pos,
    TIM_FlagBreak = TIM_SR_BIF_Pos,
    TIM_FlagCC1OF = TIM_SR_CC1OF_Pos,
    TIM_FlagCC2OF = TIM_SR_CC2OF_Pos,
    TIM_FlagCC3OF = TIM_SR_CC3OF_Pos,
    TIM_FlagCC4OF = TIM_SR_CC4OF_Pos,
} TIM_Flag_e;

typedef struct {
    TIM_ARRPreload_e PreloadEnabled;
    TIM_Direction_e Direction;
    TIM_CenterModeSelection_e CenterModeSelection;
    TIM_OnePulseMode_e OnePulseMode;
    TIM_UpdateReqSrcMode_e UpdateRequestSourceMode;
    TIM_UpdateEventToggle_e UpdateEventToggle;
} TIM_Config_t;

typedef struct {
    TIM_OutputCompareMode_e Mode;
    TIM_OutputComparePreloadToggle_e Preload;
} TIM_OutputCompareConfig_t;

typedef struct {
    TIM_DeadTimeClkDivision_e ClockDivision;
    uint8_t Duration;
} TIM_DeadTimeGeneratorConfig_t;

typedef struct {
    uint8_t IRQNumber;
    uint8_t Priority;
} TIM_AdvancedTimerIRQPair_t;

typedef struct {
    TIM_TypeDef *pTIMx;
    TIM_Config_t TIM_Config;
} TIM_Handle_t;

/* ------------ METHODS ------------ */

/*
 * Clock Setup
 */
void TIM_PeriClockControl(TIM_TypeDef *pTIMx, uint8_t Enabled);

/*
 * Init and De-Init
 */
void TIM_Init(TIM_Handle_t *pTIMHandle);
void TIM_DeInit(TIM_Handle_t *pTIMHandle);

/*
 * Start / stop control
 */
void TIM_Start(TIM_Handle_t *pTIMHandle);
void TIM_Stop(TIM_Handle_t *pTIMHandle);

/*
 * Event generation
 */
void TIM_GenerateUpdateEvent(TIM_Handle_t *pTIMHandle);
void TIM_GenerateCaptureCompareEvent(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel);

/*
 * Output compare
 */
void TIM_ConfigureOutputCompare(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel, TIM_OutputCompareConfig_t Config);

/*
 * Capture / Compare enable or disable
 */
void TIM_CaptureCompareChannelControl(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel, uint8_t Enabled);
void TIM_CaptureCompareChannelPolarity(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel, TIM_CaptureComparePolarity_e Polarity);

/*
 * Controls
 */
void TIM_SetPrescaler(TIM_Handle_t *pTIMHandle, uint16_t Prescaler);
void TIM_SetAutoReload(TIM_Handle_t *pTIMHandle, uint16_t AutoReloadValue);
void TIM_SetCounter(TIM_Handle_t *pTIMHandle, uint16_t Value);
uint32_t TIM_GetCounter(TIM_Handle_t *pTIMHandle);
void TIM_SetCompareValue(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel, uint16_t CompareValue);
uint32_t TIM_GetCompareValue(TIM_Handle_t *pTIMHandle, TIM_CaptureCompareChan_e Channel);

/*
 * Advanced timer features (TIM1)
 */
void TIM_SetRepetitionCounter(TIM_Handle_t *pTIMHandle, uint8_t Value);
void TIM_MasterMainOutputControl(TIM_Handle_t *pTIMHandle, uint8_t OutputEnabled);
void TIM_AutomaticOutputEnable(TIM_Handle_t *pTIMHandle, uint16_t Enabled);
void TIM_SetBreakPolarity(TIM_Handle_t *pTIMHandle, TIM_BreakPolarity_e Polarity);
void TIM_BreakControl(TIM_Handle_t *pTIMHandle, uint8_t Enabled);
void TIM_OffStateForRunModeControl(TIM_Handle_t *pTIMHandle, TIM_OffStateMode_e Mode);
void TIM_OffStateForIdleModeControl(TIM_Handle_t *pTIMHandle, TIM_OffStateMode_e Mode);
void TIM_ConfigureDeadTimeGenerator(TIM_Handle_t *pTIMHandle, TIM_DeadTimeGeneratorConfig_t Config);
void TIM_LockConfiguration(TIM_Handle_t *pTIMHandle, TIM_LockLevel_e LockLevel);

/*
 * Interrupts
 */
void TIM_EnableInterrupts(TIM_Handle_t *pTIMHandle, uint32_t InterruptMask);
void TIM_DisableInterrupts(TIM_Handle_t *pTIMHandle, uint32_t InterruptMask);
void TIM_IRQEnable(TIM_Handle_t *pTIMHandle, uint8_t Priority);
void TIM_IRQDisable(TIM_Handle_t *pTIMHandle);
void TIM_IRQTIM1Enable(TIM_AdvancedTimerIRQPair_t *IRQPairs, uint8_t Len);
void TIM_IRQTIM1Disable(uint8_t *IRQNumbers, uint8_t Len);

/*
 * Status flags
 */
uint8_t TIM_GetStatusFlag(TIM_Handle_t *pTIMHandle, TIM_Flag_e Flag);
void TIM_ClearStatusFlag(TIM_Handle_t *pTIMHandle, TIM_Flag_e Flag);

#endif //TIMER_DRIVER_H
