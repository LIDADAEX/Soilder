#include "intercation.h"
#include "4_Interaction/ita_robot.h"

#ifdef __cplusplus
extern "C" {
#endif

    void USB_Interaction(uint8_t* buf, uint32_t len){
        Robot::navigation.CMDProcess(buf, len);
    }

#ifdef __cplusplus
}
#endif