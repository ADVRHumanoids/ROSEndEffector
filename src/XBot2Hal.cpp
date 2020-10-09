#include <ros_end_effector/XBot2Hal.h>

ROSEE::XBot2Hal::XBot2Hal( ROSEE::EEInterface::Ptr ee_interface ) : EEHal ( ee_interface ) {
    
    XBot::Hal::DeviceInfo devinfo{"XBotEE_0", "XbotEE", 1};
    _xbot_ee_client = std::make_shared<ROSEE::XBotEEClient>(devinfo);
    _xbot_ee_client = ROSEE::XBotEEClient(devinfo);
}


bool ROSEE::XBot2Hal::sense() {
    return true;
}
    
bool ROSEE::XBot2Hal::move() {
    return true;
}
    
bool ROSEE::XBot2Hal::setMotorPositionReference (std::string motor_name, double position_ref) {
      
    return _xbot_ee_client->setMotorPositionReference(motor_name, position_ref);
    //return _xbot_ee_client->move();
    
}
    
bool ROSEE::XBot2Hal::getMotorPosition (std::string motor_name, double &position) {
    
   // return _xbot_ee_client->getMotorPosition(motor_name, position);    
}

