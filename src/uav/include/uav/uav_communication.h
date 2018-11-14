#ifndef __UAV_COMMUNICATION_H__
#define __UAV_COMMUNICATION_H__

namespace uav 
{

typedef enum
{
    RISE = 0xA0,
    DROP = 0xA1,
    TURN_LEFT = 0xA2,
    TURN_RIGHT = 0xA3,
    FORWARD = 0xA4,
    BACKWARD = 0xA5,
    LEFTWARD = 0xA6,
    RIGHTWARD = 0xA7
}motion_t;


}
#endif
