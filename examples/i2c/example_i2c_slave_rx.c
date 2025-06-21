//
// Created by Kok on 6/21/25.
//

#include <i2c_driver.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "gpio_driver.h"

/*
 * I2C GPIOs:
 * SDA      =>  PB7
 * SCL      =>  PB8
*/

#define LED_PIN             13
#define BTN_PIN             0

#define I2C_SDA             7
#define I2C_SCL             8

#define CURR_ADDR           0x01
#define MASTER_ADDR         0x03

volatile uint8_t button_trigger = 0;
uint8_t rx_buffer[32];
uint8_t rx_buffer_index = 0;
uint8_t err;

I2C_Handle_t i2cHandle = {
    .pI2Cx = I2C1,
    .I2C_Config = {
        .I2C_SCLSpeed = I2C_SCL_SPEED_SM,
        .I2C_DeviceAddressLen = I2C_DEVICE_ADDR_7_BITS,
        .I2C_DeviceAddress = CURR_ADDR,
        .I2C_FMDutyCycle = I2C_FM_DUTY_2
    }
};

int main(void) {
    GPIO_Handle_t gpioHandle = {
        .pGPIOx = GPIOC,
        .GPIO_PinConfig = {
            .GPIO_PinNumber = LED_PIN,
            .GPIO_PinMode = GPIO_MODE_OUTPUT,
            .GPIO_PinOPType = GPIO_OP_TYPE_PP,
            .GPIO_PinPuPdControl = GPIO_NO_PUPD,
            .GPIO_PinSpeed = GPIO_SPEED_HIGH
        }
    };

    // Init the LED
    GPIO_Init(&gpioHandle);

    // Init the BTN
    gpioHandle.pGPIOx = GPIOA;
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = BTN_PIN;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT_F_EDGE;
    GPIO_Init(&gpioHandle);
    GPIO_IRQConfig(BTN_PIN, 1, ENABLE);

    // Init I2C SCL
    gpioHandle.pGPIOx = GPIOB;
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = I2C_SCL;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTERNATE;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpioHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    gpioHandle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4;
    GPIO_Init(&gpioHandle);

    // Init I2C SDA
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = I2C_SDA;
    GPIO_Init(&gpioHandle);

    // Init I2C
    I2C_PeriClockControl(i2cHandle.pI2Cx, ENABLE);
    I2C_Init(&i2cHandle);
    I2C_SlaveConfigure(&i2cHandle);

    while (1) {
        if (button_trigger) {
            // Small delay because of debouncing
            for (int i = 0; i < 1000000; i++);

            GPIO_ToggleOutputPin(GPIOC, LED_PIN);

            button_trigger = 0;
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}

void I2C1_EV_IRQHandler() {
    I2C_IRQEventHandling(&i2cHandle);
}

void I2C1_ER_IRQHandler() {
    I2C_IRQErrorHandling(&i2cHandle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent) {
    if (pI2CHandle->pI2Cx == I2C1) {
        switch (AppEvent) {
            case I2C_EVENT_DATA_RECEIVE:
                if (rx_buffer_index < 32) {
                    I2C_SlaveReceiveDataIT(pI2CHandle, &rx_buffer[rx_buffer_index++]);
                }

                if (rx_buffer_index == 31) {
                    I2C_AcknowledgeControl(pI2CHandle->pI2Cx, DISABLE);
                }
                break;
            case I2C_EVENT_STOP:
                GPIO_ToggleOutputPin(GPIOC, LED_PIN);
                break;
            // Other events...
            default:
                break;
        }
    }
}