#include <ros_end_effector/XBot2HalCommunication.h>

/** CLIENT   ****************/


ROSEE::XBotEEClient::XBotEEClient(XBot::Hal::DeviceInfo devinfo, unsigned int nMotors) : DeviceClientTpl( devinfo )
{
    _nMotors = nMotors;
    _tx.motor_id = new unsigned int[nMotors]();
    _rx.motor_id = new unsigned int[nMotors]();
    _tx.motor_position_reference = new double[nMotors]();
    _rx.motor_position_actual = new double[nMotors]();
}


bool ROSEE::XBotEEClient::setMotorsPositionsReferences(std::vector<unsigned int> motor_id, std::vector<double> position_refs) {
   // _tx.motor_name = motor_name;
    
    if (motor_id.size() != _nMotors || position_refs.size() != _nMotors) {
        //TODO print error
        return false;
    }
    
    for (int i=0; i<_nMotors; i++){
        _tx.motor_id[i] = motor_id[i];
        _tx.motor_position_reference[i] = position_refs[i];
    }
    return true;
}

bool ROSEE::XBotEEClient::getMotorsPositions(std::vector<unsigned int>& motor_id, std::vector<double> &positions) {
    
    for (int i=0; i<_nMotors; i++){
        
        motor_id.push_back(_rx.motor_id[i]);
        positions.push_back(_rx.motor_position_actual[i]);
        
    }

    return true;
}

/********************** Xbot DRIVER *******************************************/
ROSEE::XBotEEDriver::XBotEEDriver(XBot::Hal::DeviceInfo devinfo):
    DeviceDriverTpl(devinfo)
{

}

bool ROSEE::XBotEEDriver::move_impl()
{
    
//     for (auto it = _rx.motors_position_refs) {
//         double filtered = filt_q.process(it->second);
//     }
//     
//     _qref_filtered = _filt_q.process ( _qref );
    return true;
}

bool ROSEE::XBotEEDriver::sense_impl()
{
    // do stuff with _rx
    return true;
}

ROSEE::XBotEEDriverContainer::XBotEEDriverContainer(std::vector<XBot::Hal::DeviceInfo> devinfo):
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

bool ROSEE::XBotEEDriverContainer::sense_all()
{
    if(!_srv_alive)
    {
        _srv_alive = _cli->check_server_alive();
        return false;
    }

    bool ret = _cli->recv();

    return ret && DeviceContainer::sense_all();
}

bool ROSEE::XBotEEDriverContainer::move_all()
{
    DeviceContainer::move_all();

    _cli->send();

    return true;
}


ROSEE::HeriIIAdapter::HeriIIAdapter(XBot::Hal::DeviceInfo devInfo):
    DeviceTplCommon(devInfo) {
    //TODO init heri
    
}

bool ROSEE::HeriIIAdapter::sense() {
    //TODO fill _rx with moto pos

    return false;
}

bool ROSEE::HeriIIAdapter::move() {
    //TODO read _tx and trasmit to heri PDO

    return true;
}






XBOT2_REGISTER_DEVICE(ROSEE::XBotEEDriverContainer, ROSEE::XBotEEClientContainer, XbotEE)
