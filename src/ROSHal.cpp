#include <ros_end_effector/ROSHal.h>

ROSEE::ROSHal::ROSHal ( ROSEE::EEInterface::Ptr ee_interface ) : EEHal ( ee_interface ) {

}


bool ROSEE::ROSHal::setMotorPositionReference (std::string motor_name, double position_ref){
    
    //TODO store the ref 

    
}

bool ROSEE::ROSHal::getMotorPosition(std::string motor_name, double& position) {
    
    //TODO get from sub
    //TODO store pos in clbk
}

bool ROSEE::ROSHal::sense() {
    
    //TODO spin the ros?
    
}

bool ROSEE::ROSHal::move(){
    
    //TODO pub on ros topic
}

