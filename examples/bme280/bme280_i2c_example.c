//
// Created by Kok on 6/22/25.
//

#include <bme280_i2c_driver.h>
#include <i2c_driver.h>
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

volatile uint8_t button_trigger = 0;
uint8_t err;

GPIO_Handle_t gpioHandle = {
    .pGPIOx = GPIOC,
    .GPIO_PinConfig = {
        .GPIO_PinNumber = LED_PIN,
        .GPIO_PinMode = GPIO_ModeOutput,
        .GPIO_PinOPType = GPIO_OpTypePP,
        .GPIO_PinPuPdControl = GPIO_NoPuPd,
        .GPIO_PinSpeed = GPIO_SpeedHigh
    }
};

I2C_Handle_t i2cHandle = {
    .pI2Cx = I2C1,
    .I2C_Config = {
        .I2C_SCLSpeed = I2C_SclSpeedFM4K,
        .I2C_DeviceAddressLen = I2C_DeviceAddr7Bits,
        .I2C_DeviceAddress = CURR_ADDR,
        .I2C_FMDutyCycle = I2C_FmDuty2
    }
};

BME280_Handle_t bme280Handle = {
    .pI2C_Handle = &i2cHandle,
    .BME280_Config = {
        .AddrPin = BME280_AddrPinLOW,
        .FilterCoeff = BME280_FilterCoeffOFF,
        .PressureOversampling = BME280_Oversampling1,
        .TemperatureOversampling = BME280_Oversampling1,
        .HumidityOversampling = BME280_Oversampling1,
        .NormalModeStanbyDuration = BME280_StandbyDuration100
    }
};

void SystemInit() {
    // Enable FPU
    SCB->CPACR |= 0xF << 20;
}

int main(void) {
    // Init the LED
    GPIO_Init(&gpioHandle);

    // Init the BTN
    gpioHandle.pGPIOx = GPIOA;
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = BTN_PIN;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_Pu;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeInputFEdge;
    GPIO_Init(&gpioHandle);
    GPIO_IRQConfig(BTN_PIN, 1, ENABLE);

    // Init I2C SCL
    gpioHandle.pGPIOx = GPIOB;
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = I2C_SCL;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeAlternate;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NoPuPd;
    gpioHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OpTypeOD;
    gpioHandle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4;
    GPIO_Init(&gpioHandle);

    // Init I2C SDA
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = I2C_SDA;
    GPIO_Init(&gpioHandle);

    // Init I2C
    I2C_PeriClockControl(i2cHandle.pI2Cx, ENABLE);
    I2C_Init(&i2cHandle);
    I2C_PeripheralControl(i2cHandle.pI2Cx, ENABLE);

    // Configure BME280
    BME280_Configure(&bme280Handle);
    if (BME280_CheckDeviceID(&bme280Handle) != BME280_ErrOK) {
        // Device not connected!
        while (1);
    }

    BME280_SetMode(&bme280Handle, BME280_ModeNormal);

    while (1) {
        if (button_trigger) {
            // Small delay because of debouncing
            for (int i = 0; i < 1000000; i++);

            GPIO_ToggleOutputPin(GPIOC, LED_PIN);

            BME280_Result_t result = {0};
            if ((err = BME280_GetSample(&bme280Handle, &result)) != 0) {
                while (1);
            }

            button_trigger = 0;
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}
