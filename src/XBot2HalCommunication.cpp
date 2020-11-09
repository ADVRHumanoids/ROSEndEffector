#include <ros_end_effector/XBot2HalCommunication.h>

/** CLIENT   ****************/


ROSEE::XBotEEClient::XBotEEClient(XBot::Hal::DeviceInfo devinfo) : DeviceClientTpl( devinfo )
{
}


bool ROSEE::XBotEEClient::setMotorPositionReference(std::string motor_name, double position_ref) {
   // _tx.motor_name = motor_name;
    _tx.motor_position_ref = position_ref;
   return true;
}

bool ROSEE::XBotEEClient::getMotorPosition(std::string motor_name, double& position) {
        std::map <std::string, double> motors_position_refs;
/*
    auto it = _rx.motors_position_refs.find(motor_name);
    
    if (it != _rx.motors_position_refs.end() ) {
        
        position = it->second;
        return true;
        
    }*/

    position = _rx.motor_position;
    
    return false;
}

/********************** Xbot DRIVER *******************************************/
ROSEE::XBotGazeboDriver::XBotGazeboDriver(XBot::Hal::DeviceInfo devinfo):
    DeviceDriverTpl(devinfo)
{

}

bool ROSEE::XBotGazeboDriver::move_impl()
{
    
//     for (auto it = _rx.motors_position_refs) {
//         double filtered = filt_q.process(it->second);
//     }
//     
//     _qref_filtered = _filt_q.process ( _qref );
    return true;
}

bool ROSEE::XBotGazeboDriver::sense_impl()
{
    // do stuff with _rx
    return true;
}

ROSEE::XBotGazeboDriverContainer::XBotGazeboDriverContainer(std::vector<XBot::Hal::DeviceInfo> devinfo):
    DeviceContainer(devinfo),
    _srv_alive(false)
{
    
    std::vector<XBot::Hal::DeviceRt::Ptr> devs;

    std::copy(get_device_vector().begin(),
              get_device_vector().end(),
              std::back_inserter(devs));

    _cli = std::make_unique<XBot::ClientManager>(
        devs,
        "shm",
        "XBotEE"
        );
}

bool ROSEE::XBotGazeboDriverContainer::sense_all()
{
    if(!_srv_alive)
    {
        _srv_alive = _cli->check_server_alive();
        return false;
    }

    bool ret = _cli->recv();

    return ret && DeviceContainer::sense_all();
}

bool ROSEE::XBotGazeboDriverContainer::move_all()
{
    DeviceContainer::move_all();

    _cli->send();

    return true;
}



XBOT2_REGISTER_DEVICE(ROSEE::XBotGazeboDriverContainer, ROSEE::XBotEEClientContainer, XbotEE)
