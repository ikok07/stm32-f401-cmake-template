#include <spi_driver.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "gpio_driver.h"

/*
 * SPI GPIOs:
 * NSS      =>  PB9
 * SCK      =>  PB10
 * MISO     =>  PB14
 * MOSI     =>  PB15
*/

#define LED_PIN             13
#define BTN_PIN             0

#define SPI_NSS             9
#define SPI_SCK             10
#define SPI_MISO            14
#define SPI_MOSI            15

volatile uint8_t button_trigger = 0;
volatile uint8_t trigger_count = 0;

SPI_Handle_t spiHandle = {
    .pSPIx = SPI2,
    .SPIConfig = {
        .SPI_DeviceMode = SPI_DEVICE_MODE_MASTER,
        .SPI_BusConfig = SPI_BUS_CFG_FULL_DUPLEX,
        .SPI_DF = SPI_DF_8BITS,
        .SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256,
        .SPI_BitsOrder = SPI_BO_MSBFIRST,
        .SPI_FrameFormat = SPI_FF_MOTOROLA,
        .SPI_SSM = SPI_SSM_HW,
        .SPI_CPHA = SPI_CPHA_1EDGE,
        .SPI_CPOL = SPI_CPOL_LOW,
        .SPI_SS_ActiveLevel = SPI_SS_LOW,
        .SPI_SS_OutputEnabled = SPI_SS_OUTPUT_ENABLED
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

    // Init SPI NSS
    gpioHandle.pGPIOx = GPIOB;
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = SPI_NSS;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTERNATE;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpioHandle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF5;
    GPIO_Init(&gpioHandle);

    // Init SPI SCK
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = SPI_SCK;
    GPIO_Init(&gpioHandle);

    // Init SPI MISO
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = SPI_MISO;
    GPIO_Init(&gpioHandle);

    // Init SPI MOSI
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = SPI_MOSI;
    GPIO_Init(&gpioHandle);

    // Init SPI
    SPI_PeriClockControl(spiHandle.pSPIx, ENABLE);
    SPI_Init(&spiHandle);
    SPI_IRQConfig(2, 15, ENABLE);
    SPI_PeripheralControl(&spiHandle, ENABLE);

    while (1) {
        if (button_trigger) {
            uint8_t data = 10;
            SPI_SendDataIT(&spiHandle, &data, 1);
            button_trigger = 0;
        }
        // Disable the SPI peripheral after 3 transmits
        else if (trigger_count == 4) {
            SPI_PeripheralControl(&spiHandle, DISABLE);
            SPI_DeInit(spiHandle.pSPIx);
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    trigger_count += 1;
    button_trigger = 1;
}

void SPI2_IRQHandler() {
    IRQ_Handling(&spiHandle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent) {
    switch (AppEvent) {
        case SPI_EVENT_TX_COMPLETE:
            GPIO_ToggleOutputPin(GPIOC, LED_PIN);
        break;
        case SPI_EVENT_RX_COMPLETE:
            // ...
        break;
        case SPI_EVENT_OVR_ERR_COMPLETE:
            // ...
        break;
        default:
            // ...
    }
}