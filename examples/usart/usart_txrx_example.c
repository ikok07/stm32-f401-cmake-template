//
// Created by Kok on 6/27/25.
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
uint8_t buffer[30];
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
        .CTSControl = USART_CTSDisabled,
        .RTSControl = USART_RTSDisabled,
        .ClockConfig = {
            // Optional clock provider
            .ClockControl = USART_ClockDisabled,
            // .CPHA = USART_PhaseFirstEdge,
            // .CPOL = USART_PolarityLOW,
            // .LastBitClockPulse = USART_LastBitLOW
        }
    }
};

int __io_putchar(int ch) {
    USART_PeripheralTXControl(&usartHandle, ENABLE);
    USART_SendData(&usartHandle, (uint8_t*)&ch, sizeof(ch));
    USART_PeripheralTXControl(&usartHandle, DISABLE);
    return ch;
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

    // // Init USART CK
    // gpioHandle.GPIO_PinConfig.GPIO_PinNumber = USART_CK;
    // gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NoPuPd;
    // gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeAlternate;
    // gpioHandle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF7;
    // GPIO_Init(&gpioHandle);

    // Init USART TX
    gpioHandle.GPIO_PinConfig.GPIO_PinNumber = USART_TX;
    gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NoPuPd;
    gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_ModeAlternate;
    gpioHandle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF7;
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

    // TX interrupts will be enabled when calling the send method
    USART_Interrupt_e interrupts[1] = {USART_InterruptIdleLine};
    USART_IRQEnable(&usartHandle, interrupts, sizeof(interrupts));

    while (1) {
        if (button_trigger) {
            // Small delay because of debouncing
            for (int i = 0; i < 1000000; i++);
            GPIO_ToggleOutputPin(GPIOC, LED_PIN);
            char *msg = "Hello, from STM32!";
            USART_PeripheralTXControl(&usartHandle, ENABLE);
            USART_PeripheralRXControl(&usartHandle, ENABLE);

            // USART_SendData(&usartHandle, (uint8_t*)msg, strlen(msg) + 1); // Polling method
            USART_SendDataIT(&usartHandle, (uint8_t*)msg, strlen(msg) + 1);
            while (usartHandle.USART_ITState.InterruptState != USART_InterruptStateReady);
            USART_ReceiveDataIT(&usartHandle, buffer, sizeof(buffer));

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
            USART_PeripheralTXControl(&usartHandle, DISABLE);
        break;
        case USART_EventRxComplete:
            USART_PeripheralRXControl(&usartHandle, DISABLE);
            if (strcmp((char*)pUSARTHandle->USART_ITState.pRXBuffer, "Hello, from Arduino Leonardo!") == 0) {
                GPIO_ToggleOutputPin(GPIOC, LED_PIN);
            }
        case USART_EventIdleDetected:
            // If the length is lower than the buffer size
            uint8_t actualLen = sizeof(buffer) - pUSARTHandle->USART_ITState.RxLen;
            (void)actualLen;

            USART_PeripheralRXControl(&usartHandle, DISABLE);
            if (strcmp((char*)pUSARTHandle->USART_ITState.pRXBuffer, "Hello, from Arduino Leonardo!") == 0) {
                GPIO_ToggleOutputPin(GPIOC, LED_PIN);
            }
        break;
        default:
        break;
    }
}
