#include <ros_end_effector/XBot2Hal.h>


ROSEE::XBot2Hal::XBot2Hal( ros::NodeHandle* nh ) : EEHal ( nh ) {
    
   // XBot::Hal::DeviceInfo devinfo{"XBotEE_0", "XbotEE", 1};
    //_xbot_ee_client = std::make_shared<ROSEE::XBotEEClient>(devinfo);
    //_xbot_ee_client = ROSEE::XBotEEClient(devinfo);
    
    //TODO is there a method to wait for xbot core to be active?
    
    std::string path_to_config_file = XBot::Utils::getXBotConfig();
    
    //NOTE Also Arturo in "xbot2_wip/src/rt_plugin/control_plugin.cpp" does this cast, hence should be safe
    _robot = XBot::RobotInterface::getRobot(path_to_config_file);
    
    _mr_msg.name.resize(_robot->getJointNum());
    _mr_msg.position.resize(_robot->getJointNum());
    _js_msg.name.resize(_robot->getJointNum());
    _js_msg.position.resize(_robot->getJointNum());
    
}


bool ROSEE::XBot2Hal::sense() {
    
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
    
bool ROSEE::XBot2Hal::move() {
    
    //NOTE if robot does not move, check if "rosservice call /xbotcore/ros_ctrl/switch 1"
    for (int i=0; i<_mr_msg.position.size(); i++) {
        
        _jointPositionReferenceMap.insert(std::make_pair(
            _mr_msg.name.at(i), _mr_msg.position.at(i)));
    }
    
    _robot->setPositionReference(_jointPositionReferenceMap);
    _robot->move();
    return true;
}
