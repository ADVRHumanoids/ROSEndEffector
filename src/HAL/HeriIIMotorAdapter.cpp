#include <ros_end_effector/HAL/HeriIIMotorAdapter.h>

XBot::Hal::HeriIIMotorAdapter::HeriIIMotorAdapter(XBot::Hal::DeviceInfo devInfo ):
    DeviceTplCommon(devInfo) {
        
    //HACK we are sure that the name (given in the conf yaml) will have the format xxx-x
    //TODO there is a better way to do this?
        

}

bool XBot::Hal::HeriIIMotorAdapter::sense() {
    //TODO fill _rx with moto pos

    return true;
}

bool XBot::Hal::HeriIIMotorAdapter::move() {
    //TODO read _tx and trasmit to heri PDO

    return true;
}

//TODO somebody has to call the move and sense of the adapter... and also create the _srv
//     _srv =  std::make_unique<ServerManager>(_rocket_vec,
//                                         "shm",
//                                         "HeriIIMotorAdapter");
// and call the run and send for srv. In the example this is gazebo plugin attached to urdf, but for 
// a real robot? we need a new node/executable?

