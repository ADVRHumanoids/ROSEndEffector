#include <vector>
#include <unordered_map>

#include <ros/ros.h>

#include <ros_end_effector/HAL/EEHal.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "EEHalExecutor" );
    ros::NodeHandle nh("EEHalExecutor");
    
    std::string hal_lib;
    if ( ! nh.getParam ( "/rosee/hal_library_name", hal_lib ) ) {
        ROS_ERROR_STREAM( "Ros parameter 'hal_library_name' not found" );
        return -1;
    }
      
    std::unique_ptr<ROSEE::EEHal> eeHalPtr = ROSEE::Utils::loadObject<ROSEE::EEHal>
                                         (hal_lib, "create_object_"+hal_lib, &nh);

    if (eeHalPtr == nullptr) {
        ROS_ERROR_STREAM( "[EEHalExecutor ERROR] in loading the EEHal Object" );
        return -1;    
    }
    
    ROS_INFO_STREAM ( "[EEHalExecutor] Loaded "<<  hal_lib << " HAL"  );   
    
    if (eeHalPtr->isHandInfoPresent()) { 
        eeHalPtr->init_hand_info_response();
        eeHalPtr->setHandInfoCallback();
    }
      
    //TODO take rate from param
    ros::Rate r(100);
    while(ros::ok()) {
        
        //TODO check order of these
        
        //receive info from robot, and fill _js_msg
        eeHalPtr->sense();
        
        //make the robot move following the refs in _mr_msg
        eeHalPtr->move();
        
        //send _js_msg to external (ie to ROSEE main node)
        eeHalPtr->publish_joint_state();
        
        ros::spinOnce();
        r.sleep();
        
    }
    
    return 0;
    
}
