//
// Created by Kok on 5/25/25.
//

#ifndef COMMONS_H
#define COMMONS_H

#define ENABLE             1
#define DISABLE            0
#define SET                ENABLE
#define RESET              DISABLE

#define __weak             __attribute__((weak))

#define WAIT_WITH_TIMEOUT(cb, err, timeoutMS, ...)      do {\
                                                            uint32_t startTicks, currTicks; \
                                                            SYSTICK_Error_e sysTickErr = SYSTICK_GetCurrTicks(&startTicks); \
                                                            if ((sysTickErr = SYSTICK_GetCurrTicks(&currTicks)) != SYSTICK_ErrOK) return err; \
                                                            while(cb) { \
                                                                if (currTicks - startTicks > timeoutMS) return err; \
                                                                else { if ((sysTickErr = SYSTICK_GetCurrTicks(&currTicks)) != SYSTICK_ErrOK) return err; } \
                                                            } \
                                                        } while (0) \

#endif //COMMONS_H
