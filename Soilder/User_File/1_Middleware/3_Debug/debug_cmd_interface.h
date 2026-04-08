#ifndef DEBUG_CMD_INTERFACE_H
#define DEBUG_CMD_INTERFACE_H

#include "stm32f407xx.h"

#ifdef __cplusplus
extern "C" {
#endif

    void Debug_FIFO_Push(uint8_t data);
    void Debug_Cmd_Poll_Callback(void);

#ifdef __cplusplus
}
#endif


#endif
