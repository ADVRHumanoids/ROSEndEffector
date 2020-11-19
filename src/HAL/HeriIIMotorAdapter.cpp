#include <ros_end_effector/HAL/HeriIIMotorAdapter.h>

XBot::Hal::HeriIIMotorAdapter::HeriIIMotorAdapter(XBot::Hal::DeviceInfo devInfo ):
    DeviceTplCommon(devInfo) {
        
    //TODO fill finger id from deviceinfo??
    
}

bool XBot::Hal::HeriIIMotorAdapter::sense() {
    //TODO fill _rx with moto pos

    return true;
}

bool XBot::Hal::HeriIIMotorAdapter::move() {
    //TODO read _tx and trasmit to heri PDO

    return true;
}



