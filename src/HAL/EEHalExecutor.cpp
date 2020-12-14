#include <vector>
#include <unordered_map>

#include <ros/ros.h>

#include <ros_end_effector/HAL/EEHal.h>

#include <ros_end_effector/Utils.h>
#include <ros_end_effector/UtilsEigen.h>

#include <ros_end_effector/HandInfoServices.h>
#include <geometry_msgs/Point.h>

bool init_hand_info_response(ROSEE::HandInfoServices& hand_info_service_handler, 
                             std::unique_ptr<ROSEE::EEHal>& eeHalPtr) {

    
    std::vector <std::string> fingers_names, motors_names;
    std::unordered_map<std::string, Eigen::MatrixXd> tips_jacobians;
    std::unordered_map<std::string, Eigen::Vector3d> tips_forces;
    Eigen::MatrixXd transmission_matrix;
    Eigen::VectorXd motors_stiffness, tips_frictions, motors_torque_limits;

    eeHalPtr->getFingersNames(fingers_names);
    eeHalPtr->getMotorsNames(motors_names);

    eeHalPtr->getTipsJacobians(tips_jacobians);
    eeHalPtr->getTransmissionMatrix(transmission_matrix);
    eeHalPtr->getMotorStiffness(motors_stiffness);
    eeHalPtr->getTipsFrictions(tips_frictions);
    eeHalPtr->getTipsForces(tips_forces);
    eeHalPtr->getMotorTorqueLimits(motors_torque_limits);
    
    hand_info_service_handler.response.fingers_names = fingers_names;
    hand_info_service_handler.response.motors_names = motors_names;
    
    for (auto it : tips_jacobians) {
        hand_info_service_handler.response.tips_jacobians.push_back( 
            ROSEE::Utils::eigenMatrixToFloat32MultiArray(it.second));
    }
    hand_info_service_handler.response.transmission_matrix = 
        ROSEE::Utils::eigenMatrixToFloat32MultiArray(transmission_matrix);
        
    hand_info_service_handler.response.motors_stiffness =
        ROSEE::Utils::eigenVectorToStdVector(motors_stiffness);
        
    hand_info_service_handler.response.tips_frictions =
        ROSEE::Utils::eigenVectorToStdVector(tips_frictions);
    
    for (auto it : tips_forces) {
        geometry_msgs::Point point;
        point.x = it.second[0];
        point.y = it.second[1];
        point.z = it.second[2];
        hand_info_service_handler.response.tips_forces.push_back(point);
    }
        
    hand_info_service_handler.response.motors_torque_limits =
        ROSEE::Utils::eigenVectorToStdVector(motors_torque_limits);
    
    return true;
}

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
    
    ROSEE::HandInfoServices hand_info_service_handler (&nh);
    if (eeHalPtr->isHandInfoPresent()) { 
        init_hand_info_response(hand_info_service_handler, eeHalPtr);
        hand_info_service_handler.init_ros_service();
        
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
