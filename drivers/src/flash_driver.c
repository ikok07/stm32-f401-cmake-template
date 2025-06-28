//
// Created by Kok on 6/28/25.
//

#include "flash_driver.h"

/**
 * @brief Sets the ratio of the CPU clock period to the Flash memory access time
 * @param WaitState The desired wait state
 */
void FLASH_SetLatency(FLASH_WaitState_e WaitState) {
    FLASH->ACR &=~ (0xF << FLASH_ACR_LATENCY_Pos);
    FLASH->ACR |= (WaitState << FLASH_ACR_LATENCY_Pos);
}

/**
 * @brief Selects the program FLASH parallelism
 * @param Parallelism Parallelism level
 */
void FLASH_SetParallelism(FLASH_Parallelism_e Parallelism) {
    FLASH->CR |= (Parallelism << FLASH_CR_PSIZE_Pos);
}

/**
 * @brief Gets the current flash memory state
 * @return If the a flash memory operation is ongoing
 */
FLASH_MemoryState FLASH_GetCurrentState() {
    return (FLASH->SR & (1 << FLASH_SR_BSY_Pos)) > 0 ? 1 : 0;
}

void FLASH_UnlockCR() {
    FLASH->KEYR = FLASH_CR_UNLOCK_KEY1;
    FLASH->KEYR = FLASH_CR_UNLOCK_KEY2;
}

void FLASH_LockCR() {
    FLASH->CR |= (1 << FLASH_CR_LOCK_Pos);
}

void FLASH_UnlockOPTCR() {
    FLASH->OPTKEYR = FLASH_OPTCR_UNLOCK_KEY1;
    FLASH->OPTKEYR = FLASH_OPTCR_UNLOCK_KEY2;
}

void FLASH_LockOPTCR() {
    FLASH->OPTKEYR |= (1 << FLASH_OPTCR_OPTLOCK_Pos);
}
