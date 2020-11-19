#include <ros_end_effector/HAL/XBot2HalDevice.h>


ROSEE::XBot2HalDevice::XBot2HalDevice( ros::NodeHandle* nh ) : EEHal ( nh ) {
    
   // XBot::Hal::DeviceInfo devinfo{"XBotEE_0", "XbotEE", 1};
    //_xbot_ee_client = std::make_shared<ROSEE::XBotEEClient>(devinfo);
    //_xbot_ee_client = ROSEE::XBotEEClient(devinfo);
    
    //TODO is there a method to wait for xbot core to be active?
    
    std::string path_to_config_file = XBot::Utils::getXBotConfig();
        
    _robot = XBot::RobotInterface::getRobot(path_to_config_file);
    
    auto robot = 
        std::static_pointer_cast<XBot::RobotInterfaceXBot2Rt>(_robot);
        
    _device = robot->getDeviceInstance<ROSEE::XBotEEBase>("dev_0");
    
    _mr_msg.name.resize(_robot->getJointNum());
    _mr_msg.position.resize(_robot->getJointNum());
    _js_msg.name.resize(_robot->getJointNum());
    _js_msg.position.resize(_robot->getJointNum());
    
}


bool ROSEE::XBot2HalDevice::sense() {
    
    _robot->sense();
    _robot->getJointPosition(_jointPositionActualMap);
    
    if (_js_msg.name.size() != _jointPositionActualMap.size() ||
        _js_msg.position.size() != _jointPositionActualMap.size()) {
        //TODO print error
        return false;
    }
    
    int i = 0;
    for (auto it : _jointPositionActualMap ) {
        
        //TODO it is better to fill the names only once in the costructor?
        _js_msg.name.at(i) = it.first;
        _js_msg.position.at(i) = it.second;
        i++;
    }
    
    return true;
}
    
bool ROSEE::XBot2HalDevice::move() {
    
    //NOTE if robot does not move, check if "rosservice call /xbotcore/ros_ctrl/switch 1"
    for (int i=0; i<_mr_msg.position.size(); i++) {
        
        _jointPositionReferenceMap.insert(std::make_pair(
            _mr_msg.name.at(i), _mr_msg.position.at(i)));
    }
    
    _robot->setPositionReference(_jointPositionReferenceMap);
    _robot->move();
    return true;
}
