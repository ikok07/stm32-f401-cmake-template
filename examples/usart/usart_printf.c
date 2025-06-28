//
// Created by Kok on 6/28/25.
//

#include "usart_driver.h"

int __io_putchar(int ch) {
    USART_PeripheralTXControl(&usartHandle, ENABLE);
    USART_SendData(&usartHandle, (uint8_t*)&ch, sizeof(ch));
    USART_PeripheralTXControl(&usartHandle, DISABLE);
    return ch;
};