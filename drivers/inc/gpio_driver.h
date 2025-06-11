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

/**
 * @GPIO_PINS
 */
#define GPIO_PIN_NO_0               0
#define GPIO_PIN_NO_1               1
#define GPIO_PIN_NO_2               2
#define GPIO_PIN_NO_3               3
#define GPIO_PIN_NO_4               4
#define GPIO_PIN_NO_5               5
#define GPIO_PIN_NO_6               6
#define GPIO_PIN_NO_7               7
#define GPIO_PIN_NO_8               8
#define GPIO_PIN_NO_9               9
#define GPIO_PIN_NO_10              10
#define GPIO_PIN_NO_11              11
#define GPIO_PIN_NO_12              12
#define GPIO_PIN_NO_13              13
#define GPIO_PIN_NO_14              14
#define GPIO_PIN_NO_15              15

/**
 * @GPIO_PIN_MODES
 */
#define GPIO_MODE_INPUT             0
#define GPIO_MODE_OUTPUT            1
#define GPIO_MODE_ALTERNATE         2
#define GPIO_MODE_ANALOG            3
#define GPIO_MODE_INPUT_F_EDGE      4
#define GPIO_MODE_INPUT_R_EDGE      5
#define GPIO_MODE_INPUT_RF_EDGE     6

/**
 * @GPIO_PIN_SPEEDS
 */
#define GPIO_SPEED_LOW              0
#define GPIO_SPEED_MEDIUM           1
#define GPIO_SPEED_HIGH             2
#define GPIO_SPEED_VERY_HIGH        3

/**
 * @GPIO_PULL_UP_DOWN_MODES
 */
#define GPIO_NO_PUPD                0
#define GPIO_PU                     1
#define GPIO_PD                     2

/**
 * @GPIO_PULL_OUTPUT_MODES
 */
#define GPIO_OP_TYPE_PP             0
#define GPIO_OP_TYPE_OD             1

/**
 * @GPIO_ALTERNATE_FUNCTIONS
 */
#define GPIO_AF0                    0
#define GPIO_AF1                    1
#define GPIO_AF2                    2
#define GPIO_AF3                    3
#define GPIO_AF4                    4
#define GPIO_AF5                    5
#define GPIO_AF6                    6
#define GPIO_AF7                    7
#define GPIO_AF8                    8
#define GPIO_AF9                    9
#define GPIO_AF10                   10
#define GPIO_AF11                   11
#define GPIO_AF12                   12
#define GPIO_AF13                   13
#define GPIO_AF14                   14
#define GPIO_AF15                   15


typedef struct {
    uint8_t GPIO_PinNumber;         /** Possible values from @GPIO_PINS */
    uint8_t GPIO_PinMode;           /** Possible values from @GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed;          /** Possible values from @GPIO_PIN_SPEEDS */
    uint8_t GPIO_PinPuPdControl;    /** Possible values from @GPIO_PULL_UP_DOWN_MODES */
    uint8_t GPIO_PinOPType;         /** Possible values from @GPIO_PULL_OUTPUT_MODES */
    uint8_t GPIO_PinAltFunMode;
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
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);
uint32_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint32_t Value);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration
 */
void GPIO_IRQConfig(uint8_t PinNumber, uint8_t IRQPriority, uint8_t Enabled);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif //GPIO_DRIVER_H
