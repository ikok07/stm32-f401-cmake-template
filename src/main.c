//
// Created by Kok on 7/4/25.
//

#include <generic_methods.h>
#include <i2c_driver.h>
#include <ssd1306_i2c_driver.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "gpio_driver.h"

/*
 * DISPLAY GPIO:
 * PWR      => PB6
 * I2C GPIOs:
 * SDA      =>  PB7
 * SCL      =>  PB8
*/

#define LED_PIN             13
#define BTN_PIN             0

#define DISPLAY_PWR         6

#define I2C_SDA             7
#define I2C_SCL             8


volatile uint8_t button_trigger = 0;
uint8_t counter = 0;

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
        .I2C_DeviceAddress = I2C_DeviceAddr7Bits,
        .I2C_FMDutyCycle = I2C_FmDuty2,
        .I2C_SCLSpeed = I2C_SclSpeedFM4K
    }
};

uint8_t tempImg[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0x81, 0x98, 0x3C, 0x3C, 0x98, 0x81, 0xE7, 0xFF, 0xFF, 0xFF, 0xFF
};

uint8_t sunImg[] = {
    0x10, 0x54, 0x38, 0x28, 0x62, 0x9f, 0x9f, 0x60
};

uint8_t desktopImg[] = {
    0x00, 0xfc, 0x04, 0x24, 0x24, 0x04, 0x24, 0x04, 0x04, 0x24, 0x04, 0x04, 0x04, 0x04, 0xfc, 0x00,
    0x00, 0x3f, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3f, 0x00
};

uint8_t squareImg[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

uint8_t testImg[] = {
    0x00, 0x38, 0x7C, 0x44, 0x44, 0x7C, 0x38, 0x00,
    0x00, 0x10, 0x00, 0x20, 0x00, 0x10, 0x00, 0x20
};

int main(void) {
    Generic_InitSysTick();

    // Init the LED
    GPIO_Init(&gpioHandle);

    // Init the BTN
    gpioHandle.pGPIOx = GPIOA;
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = BTN_PIN;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_Pu;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeInputFEdge;
    GPIO_Init(&gpioHandle);
    GPIO_IRQConfig(BTN_PIN, 1, ENABLE);

    // Init display power pin
    gpioHandle.pGPIOx = GPIOB;
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = DISPLAY_PWR;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NoPuPd;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeOutput;
    GPIO_Init(&gpioHandle);

    // Init I2C SCL
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = I2C_SCL;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeAlternate;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NoPuPd;
    gpioHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OpTypeOD;
    gpioHandle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4;
    GPIO_Init(&gpioHandle);

    // Init I2C SDA
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = I2C_SDA;
    GPIO_Init(&gpioHandle);

    // Configure I2C
    I2C_Error_e i2cError;
    I2C_PeriClockControl(i2cHandle.pI2Cx, ENABLE);
    if ((i2cError = I2C_Init(&i2cHandle)) != I2C_ErrOK) {
        return SSD1306_ErrComm;
    }
    I2C_PeripheralControl(i2cHandle.pI2Cx, ENABLE);

    // Setup the display
    SSD1306_Handle_t display = {
        .I2CHandle = &i2cHandle,
        .PowerConfig = {
            .pGPIOx = GPIOB,
            .PinNumber = DISPLAY_PWR
        }
    };

    SSD1306_Config_t displayConfig = {
        .MUXRatio = 63,
        .DisplayOffset = 0,
        .DisplayStartLine = 0,
        .SegmentsRemapped = DISABLE,
        .COMScanRemapped = DISABLE,
        .AlternativeCOMPinConfigEnabled = DISABLE,
        .COMLeftRightRemapEnabled = DISABLE,
        .Contrast = 255,
        .DivideRatio = 0,
        .OSCFreq = 0xF
    };

    SSD1306_SetFont(&display, SSD1306_Font6x8);
    SSD1306_Error_e displayErr = SSD1306_Init(&display, displayConfig);
    displayErr = SSD1306_SetMemoryAddrMode(&display, SSD1306_MemAddrHorizontal);
    displayErr = SSD1306_SetWriteAreaHV(&display, SSD1306_MIN_WIDTH, SSD1306_MAX_WIDTH, SSD1306_MIN_HEIGHT, SSD1306_MAX_HEIGHT);
    displayErr = SSD1306_DisplayControl(&display, ENABLE);
    displayErr = SSD1306_ClearAreaHV(&display);

    if (displayErr) {
        while (1) {
            uint8_t test = 1;
            (void) test;
        }
    }

    while (1) {
        if (button_trigger) {
            button_trigger = 0;

            Generic_Delay(1000);
            GPIO_ToggleOutputPin(GPIOC, LED_PIN);
            // SSD1306_ClearAreaHV(&display);
            displayErr = SSD1306_SetWriteAreaHV(&display, SSD1306_MIN_WIDTH, SSD1306_MAX_WIDTH - 1, SSD1306_MIN_HEIGHT, SSD1306_MAX_HEIGHT);
            displayErr = SSD1306_WriteHV(&display, 0, 0, "Hello, World ");

            // char buffer[4];
            // sprintf(buffer, "%d", counter);
            // displayErr = SSD1306_WriteHV(&display, strlen("Hello, World ") * SSD1306_FONT_WIDTH, 0, buffer);

            // displayErr = SSD1306_DrawHV(&display, 10, 10, 8, 8, sunImg, sizeof(sunImg));

            counter++;
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}
