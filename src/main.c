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
    0x00, 0x00, 0x03, 0xc0, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40,
    0x02, 0x40, 0x02, 0x40, 0x06, 0x60, 0x04, 0x20, 0x04, 0x20, 0x06, 0x60, 0x03, 0xc0, 0x00, 0x00
};

uint8_t sunImg[] = {
    0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x18, 0x00, 0x00, 0x18, 0x00, 0x0c, 0x18, 0x30, 0x0e,
    0x00, 0x70, 0x06, 0x00, 0x60, 0x00, 0x3c, 0x00, 0x00, 0xff, 0x00, 0x00, 0xc3, 0x00, 0x01, 0x81,
    0x80, 0x79, 0x81, 0x9e, 0x79, 0x81, 0x9e, 0x01, 0x81, 0x80, 0x00, 0xc3, 0x00, 0x00, 0xff, 0x00,
    0x00, 0x3c, 0x00, 0x06, 0x00, 0x60, 0x0e, 0x00, 0x70, 0x0c, 0x18, 0x30, 0x00, 0x18, 0x00, 0x00,
    0x18, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00
};

uint8_t rainImg[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x0f, 0xfc, 0x00, 0x1e, 0x1e, 0x00, 0x38,
    0x07, 0x00, 0x30, 0x03, 0x00, 0x70, 0x03, 0xf0, 0x60, 0x01, 0xf8, 0x60, 0x00, 0x1c, 0x60, 0x00,
    0x0e, 0x60, 0x00, 0x06, 0x70, 0xc0, 0xc6, 0x30, 0xc0, 0xc6, 0x39, 0x99, 0x8e, 0x19, 0x99, 0x9c,
    0x03, 0xbb, 0x98, 0x03, 0x33, 0x00, 0x03, 0x33, 0x00, 0x06, 0x66, 0x00, 0x06, 0x66, 0x00, 0x00,
    0xe0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00
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

    SSD1306_Error_e displayErr = SSD1306_Init(&display, displayConfig);
    displayErr = SSD1306_SetFont(&display, SSD1306_Font8x16, DISABLE);
    displayErr = SSD1306_SetMemoryAddrMode(&display, SSD1306_MemAddrHorizontal);
    displayErr = SSD1306_SetWriteAreaH(&display, SSD1306_MIN_WIDTH, SSD1306_MAX_WIDTH - 1, SSD1306_MIN_HEIGHT, SSD1306_MAX_HEIGHT);
    displayErr = SSD1306_DisplayControl(&display, ENABLE);
    displayErr = SSD1306_ClearAreaH(&display);
    displayErr = SSD1306_UpdateH(&display);

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
            displayErr = SSD1306_SetWriteAreaH(&display, SSD1306_MIN_WIDTH, SSD1306_MAX_WIDTH - 1, SSD1306_MIN_HEIGHT, SSD1306_MAX_HEIGHT);
            // displayErr = SSD1306_WriteH(&display, 0, 0, "Hello");

            // char buffer[4];
            // sprintf(buffer, "%d", counter);
            // displayErr = SSD1306_WriteHV(&display, strlen("Hello, World ") * SSD1306_FONT_WIDTH, 0, buffer);

            displayErr = SSD1306_DrawH(&display, 10, 10, 24, 24, sunImg, sizeof(sunImg));

            displayErr = SSD1306_DrawH(&display, 40, 10, 24, 24, rainImg, sizeof(rainImg));

            displayErr = SSD1306_UpdateH(&display);
            counter++;
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}
