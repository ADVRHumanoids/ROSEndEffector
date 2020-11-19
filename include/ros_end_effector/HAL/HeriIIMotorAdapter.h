#ifndef __HERI_II_MOTOR_ADAPTER__
#define __HERI_II_MOTOR_ADAPTER__

#include <ros_end_effector/HAL/HeriIIMotorPacket.h>
#include <xbot2/xbot2.h>
#include <xbot2/hal/device.h>


#include <string>
#include <memory>

namespace XBot {namespace Hal {
     
    
class HeriIIMotorAdapter : public XBot::Hal::DeviceTplCommon<HeriIIMotorPacket::Rx, HeriIIMotorPacket::Tx>
{
public:
    HeriIIMotorAdapter(XBot::Hal::DeviceInfo devInfo);
    
    bool sense() override;

    bool move() override;

private:
    unsigned short int finger_id;
    unsigned short int motor_in_finger_id;
    
    
};

}}

#endif // __HERI_II_MOTOR_ADAPTER__
