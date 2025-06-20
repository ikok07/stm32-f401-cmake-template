//
// Created by Kok on 6/18/25.
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

#define SLAVE_ADDR          0x03

volatile uint8_t button_trigger = 0;

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

    I2C_Handle_t i2cHandle = {
        .pI2Cx = I2C1,
        .I2C_Config = {
            .I2C_SCLSpeed = I2C_SCL_SPEED_SM,
            .I2C_DeviceAddressLen = I2C_DEVICE_ADDR_7_BITS,
            .I2C_DeviceAddress = 0x01,
            .I2C_FMDutyCycle = I2C_FM_DUTY_2
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

    while (1) {
        if (button_trigger) {
            // Small delay because of debouncing
            for (int i = 0; i < 1000000; i++);

            // uint8_t tx_data[16] = "Hello, World!";
            uint8_t command = 0x51;
            uint8_t length = 0;

            I2C_PeripheralControl(i2cHandle.pI2Cx, ENABLE);

            // Get the length
            I2C_MasterSendData(&i2cHandle, &command, sizeof(command), SLAVE_ADDR, I2C_STOP_DISABLED);
            I2C_MasterReceiveData(&i2cHandle, &length, sizeof(length), SLAVE_ADDR, I2C_STOP_DISABLED);

            // Receive the data
            uint8_t rx_data[length];
            if (length > 0) {
                command = 0x52;
                I2C_MasterSendData(&i2cHandle, &command, sizeof(command), SLAVE_ADDR, I2C_STOP_DISABLED);
                I2C_MasterReceiveData(&i2cHandle, rx_data, sizeof(rx_data), SLAVE_ADDR, I2C_STOP_ENABLED);
            }

            if (rx_data[3] == 4) GPIO_ToggleOutputPin(GPIOC, LED_PIN);

            I2C_PeripheralControl(i2cHandle.pI2Cx, DISABLE);
            button_trigger = 0;
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}