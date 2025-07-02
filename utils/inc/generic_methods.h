//
// Created by Kok on 6/30/25.
//

#ifndef GENERIC_METHODS_H
#define GENERIC_METHODS_H

#include <stdint.h>
#include "systick_driver.h"

void Generic_InitSysTick();
SYSTICK_Error_e Generic_Delay(uint32_t ms);

#endif //GENERIC_METHODS_H
