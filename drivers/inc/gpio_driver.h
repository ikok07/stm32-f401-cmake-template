//
// Created by Kok on 5/25/25.
// GPIO Driver suited for STM32H7A3XX
//

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32f4xx.h"

/**
 * @brief Enables the GPIO peripheral clock
 * @param offset The offset of the enable register specific to each GPIO port
 */
#define GPIO_PCLK_EN(offset)        (RCC->AHB1ENR |= (1U << offset))

/**
 * @brief Disables the GPIO peripheral clock
 * @param offset The offset of the enable register specific to each GPIO port
 */
#define GPIO_PCLK_DI(offset)        (RCC->AHB1ENR &=~ (1U << offset))

/**
 * @brief Resets the GPIO port
 * @param offset The offset of the reset register specific to each GPIO port
 */
#define GPIO_REG_RESET(offset)      do { (RCC->AHB1RSTR |= (1U << offset)); (RCC->AHB1RSTR &=~ (1U << offset)); } while (0)

/**
 * @brief Converts a GPIO Base Address to a code to be used in the SYSCFG_EXTICRx register
 * @param address GPIO Base Address
 */
#define GPIO_BASEADDR_TO_CODE(address) ((address == GPIOA) ? 0 :\
                                       (address == GPIOB) ? 1 :\
                                       (address == GPIOC) ? 2 :\
                                       (address == GPIOD) ? 3 :\
                                       (address == GPIOE) ? 4 :\
                                       (address == GPIOH) ? 7 : 0)

/**
 * @brief Gets the corresponding IRQ Number for the provided GPIO Pin
 * @param pin GPIO Pin
 */
#define GPIO_PIN_TO_IRQ_NUMBER(pin) ((pin == 0) ? EXTI0_IRQn :\
                                     (pin == 1) ? EXTI1_IRQn :\
                                     (pin == 2) ? EXTI2_IRQn :\
                                     (pin == 3) ? EXTI3_IRQn :\
                                     (pin == 4) ? EXTI4_IRQn :\
                                     (pin >= 5 && pin <= 9) ? EXTI9_5_IRQn :\
                                     (pin >= 10 && pin <= 15) ? EXTI15_10_IRQn : EXTI0_IRQn)

typedef enum {
    GPIO_PinNo0,
    GPIO_PinNo1,
    GPIO_PinNo2,
    GPIO_PinNo3,
    GPIO_PinNo4,
    GPIO_PinNo5,
    GPIO_PinNo6,
    GPIO_PinNo7,
    GPIO_PinNo8,
    GPIO_PinNo9,
    GPIO_PinNo10,
    GPIO_PinNo11,
    GPIO_PinNo12,
    GPIO_PinNo13,
    GPIO_PinNo14,
    GPIO_PinNo15
} GPIO_Pin_e;

typedef enum {
    GPIO_ModeInput,
    GPIO_ModeOutput,
    GPIO_ModeAlternate,
    GPIO_ModeAnalog,
    GPIO_ModeInputFEdge,
    GPIO_ModeInputREdge,
    GPIO_ModeInputRFEdge
} GPIO_PinMode_e;

typedef enum {
    GPIO_SpeedLow,
    GPIO_SpeedMedium,
    GPIO_SpeedHigh,
    GPIO_SpeedVeryHigh
} GPIO_Speed_e;

typedef enum {
    GPIO_NoPuPd,
    GPIO_Pu,
    GPIO_Pd
} GPIO_PullUpDownMode_e;

typedef enum {
    GPIO_OpTypePP,
    GPIO_OpTypeOD
} GPIO_OutputType_e;

typedef enum {
    GPIO_AF0,
    GPIO_AF1,
    GPIO_AF2,
    GPIO_AF3,
    GPIO_AF4,
    GPIO_AF5,
    GPIO_AF6,
    GPIO_AF7,
    GPIO_AF8,
    GPIO_AF9,
    GPIO_AF10,
    GPIO_AF11,
    GPIO_AF12,
    GPIO_AF13,
    GPIO_AF14,
    GPIO_AF15,
} GPIO_AlternateFunction_e;

typedef struct {
    GPIO_Pin_e GPIO_PinNumber;
    GPIO_PinMode_e GPIO_PinMode;
    GPIO_Speed_e GPIO_PinSpeed;
    GPIO_PullUpDownMode_e GPIO_PinPuPdControl;
    GPIO_OutputType_e GPIO_PinOPType;
    GPIO_AlternateFunction_e GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
    GPIO_TypeDef *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t Enabled);

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, GPIO_Pin_e PinNumber);
uint32_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, GPIO_Pin_e PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint32_t Value);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, GPIO_Pin_e PinNumber);

/*
 * IRQ Configuration
 */
void GPIO_IRQConfig(GPIO_Pin_e PinNumber, uint8_t IRQPriority, uint8_t Enabled);
void GPIO_IRQHandling(GPIO_Pin_e PinNumber);

#endif //GPIO_DRIVER_H
