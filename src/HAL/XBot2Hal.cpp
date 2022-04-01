#include <end_effector/HAL/XBot2Hal.h>


ROSEE::XBot2Hal::XBot2Hal( ros::NodeHandle* nh ) : EEHal ( nh ) {
    
    //little HACK to be sure xbot is ready and gazebo is unpaused (this service is on after xbot2-core
    // is on AND gazebo is unpaused
    ROS_INFO_STREAM("[XBot2Hal constructor]: wait for xbot2core and gazebo to be ready...");
    ros::service::waitForService("/xbotcore/ros_ctrl/switch", -1);
    ROS_INFO_STREAM("[XBot2Hal constructor]: xbot2core and gazebo ready!");
    
    //We call the xbot service to command the hand through ros
//     std_srvs::SetBool serviceMsg;
//     serviceMsg.request.data = true;
//     ros::ServiceClient client = nh->serviceClient<std_srvs::SetBool>("/xbotcore/ros_ctrl/switch");
// 
//     if (client.call(serviceMsg) ) {
//         
//         
//         
//         if (!serviceMsg.response.success){
//             ROS_ERROR_STREAM("[XBot2Hal constructor]: Failed to call /xbotcore/ros_ctrl/switch with error message: " << serviceMsg.response.message << ". Aborting");
//             exit(-1);
//         }    
//         ROS_INFO_STREAM("[XBot2Hal constructor]: Succesfully called /xbotcore/ros_ctrl/switch");
//  
//     } else {
//         ROS_ERROR_STREAM("[XBot2Hal constructor]: Failed to call /xbotcore/ros_ctrl/switch. Aborting");
//         exit(-1);
//     }
    

    std::string path_to_config_file = XBot::Utils::getXBotConfig();
    
    _robot = XBot::RobotInterface::getRobot(path_to_config_file);
    
    _mr_msg.name.resize(_robot->getJointNum());
    _mr_msg.position.resize(_robot->getJointNum());
    _js_msg.name.resize(_robot->getJointNum());
    _js_msg.position.resize(_robot->getJointNum());
    _js_msg.velocity.resize(_robot->getJointNum());
    _js_msg.effort.resize(_robot->getJointNum());
    
}


bool ROSEE::XBot2Hal::sense() {
    if (! _robot->sense()){
        return false;
    }
    _robot->getJointPosition(_jointPositionActualMap);
    if (_js_msg.name.size() != _jointPositionActualMap.size() ||
        _js_msg.position.size() != _jointPositionActualMap.size()) {
        
        ROS_ERROR_STREAM("[XBot2Hal::" << __func__ << "] size of _js_msg is different from the size" <<
            " or what received from the robot (_jointPositionActualMap)");
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

        _jointPositionReferenceMap[_mr_msg.name.at(i)] = _mr_msg.position.at(i);
    }
    
    if (! _robot->setPositionReference(_jointPositionReferenceMap)) {
        return false;
    }

    return  _robot->move();
}
