#include <ros/ros.h>

#include <ros_end_effector/HAL/EEHal.h>
#include <ros_end_effector/HAL/DummyHal.h>
#include <ros_end_effector/HAL/XBot2Hal.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "EEHalExecutor" );
    ros::NodeHandle nh("EEHalExecutor");
    //TODO load the correct derived class
    //ROSEE::EEHal::Ptr eeHalPtr = std::make_shared<ROSEE::DummyHal>(&nh);
    ROSEE::EEHal::Ptr eeHalPtr = std::make_shared<ROSEE::XBot2Hal>(&nh);
    
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
