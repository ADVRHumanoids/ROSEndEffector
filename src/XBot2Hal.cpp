#include <ros_end_effector/XBot2Hal.h>

ROSEE::XBot2Hal::XBot2Hal( ROSEE::EEInterface::Ptr ee_interface ) : EEHal ( ee_interface ) {
    
    //_xbot_ee = _robot->getDeviceInstance<ROSEE::XBotEEClient>("XBotEEHal");
}


bool ROSEE::XBot2Hal::sense() {
    
}
    
bool ROSEE::XBot2Hal::move() {
    
}
    
bool ROSEE::XBot2Hal::setMotorPositionReference (std::string motor_name, double position_ref) {
  
    return _xbot_ee->setMotorPositionReference(motor_name, position_ref);;
}
    
bool ROSEE::XBot2Hal::getMotorPosition (std::string motor_name, double &position) {
    
    
    return _xbot_ee->getMotorPosition(motor_name, position);;
    
}
