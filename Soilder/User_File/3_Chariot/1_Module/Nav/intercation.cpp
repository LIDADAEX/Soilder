#include "intercation.h"
#include "4_Interaction/ita_robot.h"

extern bool init_finished;

#ifdef __cplusplus
extern "C" {
#endif
    void USB_Interaction(uint8_t* buf, uint32_t len){
		if(!init_finished) return;
        Robot::navigation.CMDProcess(buf, len);
    }

#ifdef __cplusplus
}
#endif