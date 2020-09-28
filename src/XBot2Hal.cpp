#include <ros_end_effector/XBot2Hal.h>

bool ROSEE::XBot2Hal::sense() {
    
}
    
bool ROSEE::XBot2Hal::move() {
    
}
    
bool ROSEE::XBot2Hal::setMotorPositionReference (std::string motor_name, double position_ref) {
    
    _tx.motor_name = motor_name;
    _tx.motor_position_ref = position_ref;
    return true;
}
    
bool ROSEE::XBot2Hal::getMotorPosition (std::string motor_name, double &position) {
    
    std::map <std::string, double> motors_position_refs;

    auto it = _rx.motors_position_refs.find(motor_name);
    
    if (it != _rx.motors_position_refs.find(motor_name).end(){
        
        position = it.second;
        return true;
        
    }
    
    return false;
    
    
}


/********************** Xbot DRIVER *******************************************/
ROSEE::XbotEEDriverXbotEEDriver(Hal::DeviceInfo devinfo):
    DeviceDriverTpl(devinfo),
    _timeout(2s)
{

}

bool ROSEE::XbotEEDriver::move_impl()
{
    
    for (auto it = _rx.motors_position_refs) {
        double filtered = filt_q.process(it->second);
    }
    
    _qref_filtered = _filt_q.process ( _qref );
    return true;
}

bool Hal::RocketDriver::sense_impl()
{
    // do stuff with _rx
    return true;
}
