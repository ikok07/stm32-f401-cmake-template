//
// Created by Kok on 6/26/25.
//

#include <string.h>

#include "stm32f4xx.h"
#include "gpio_driver.h"
#include "usart_driver.h"

/*
 * USART GPIOs:
 * CK       =>  PA8
 * TX       =>  PA9
 * RX       =>  PA10
 * CTS      =>  PA11
 * RTS      =>  PA12
*/

#define LED_PIN             13
#define BTN_PIN             0

#define USART_CK            8
#define USART_TX            9
#define USART_RX            10
#define USART_CTS           11
#define USART_RTS           12

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

USART_Handle_t usartHandle = {
    .pUSARTx = USART1,
    .USART_Config = {
        .BaudRate = USART_BaudRate_9600,
        .Oversampling = USART_OversamplingBy8,
        .BitMethod = USART_ThreeSampleBitMethod,
        .ParityControl = USART_ParityControlDisabled,
        .ParitySelection = USART_EvenParity,
        .StopBits = USART_StopBits1,
        .WordLength = USART_WordLength8Bits,
        .CTSControl = USART_CTSDisabled,        // Optional
        .RTSControl = USART_RTSDisabled,        // Optional
        .ClockConfig = {
            // Optional clock provider
            .ClockControl = USART_ClockEnabled,
            .CPHA = USART_PhaseFirstEdge,
            .CPOL = USART_PolarityLOW,
            .LastBitClockPulse = USART_LastBitLOW
        }
    }
};

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

    // Init USART CK
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = USART_CK;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NoPuPd;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeAlternate;
    gpioHandle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF7;
    GPIO_Init(&gpioHandle);

    // Init USART TX
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = USART_TX;
    GPIO_Init(&gpioHandle);
    //
    // // Init USART RX
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = USART_RX;
    GPIO_Init(&gpioHandle);

    // Init USART CTS
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = USART_CTS;
    GPIO_Init(&gpioHandle);

    // Init USART RTS
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = USART_RTS;
    GPIO_Init(&gpioHandle);

    USART_PeriClockControl(&usartHandle, ENABLE);
    USART_Init(&usartHandle);
    USART_PeripheralControl(&usartHandle, ENABLE);
    USART_PeripheralTXControl(&usartHandle, ENABLE);

    // TX interrupts will be enabled when calling the send method
    USART_IRQEnable(&usartHandle, NULL, 0);

    while (1) {
        if (button_trigger) {
            // Small delay because of debouncing
            for (int i = 0; i < 1000000; i++);

            uint8_t data = 3;

            // USART_SendData(&usartHandle, &data, sizeof(data)); // Polling method
            USART_SendDataIT(&usartHandle, &data, sizeof(data));

            button_trigger = 0;
        }
    };
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandling(BTN_PIN);
    button_trigger = 1;
}

void USART1_IRQHandler() {
    USART_IRQHandler(&usartHandle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, USART_Event_e AppEvent) {
    switch (AppEvent) {
        case USART_EventTxComplete:
            GPIO_ToggleOutputPin(GPIOC, LED_PIN);
        break;
        default:
        break;
    }
}