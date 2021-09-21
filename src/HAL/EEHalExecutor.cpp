#include <vector>
#include <unordered_map>

#include <ros/ros.h>

#include <ros_end_effector/HAL/EEHal.h>

#ifdef _MATLOGGER2
    #include <matlogger2/matlogger2.h>
    #include <matlogger2/utils/mat_appender.h>
#endif

int main ( int argc, char **argv ) {
    
    ros::init ( argc, argv, "EEHalExecutor" );
    ros::NodeHandle nh("EEHalExecutor");
    
    std::string hal_lib;
    if ( ! nh.getParam ( "/rosee/hal_library_name", hal_lib ) ) {
        ROS_ERROR_STREAM( "Ros parameter 'hal_library_name' not found" );
        return -1;
    }

#ifdef _MATLOGGER2
    std::string matlogger_path;
    XBot::MatLogger2::Ptr logger; /* mt logger */
    XBot::MatAppender::Ptr appender; /* mt logger */

    bool logging = false;

    if ( nh.getParam ( "/rosee/matlogger_path", matlogger_path ) && matlogger_path.size() != 0) {
        ROS_INFO_STREAM( "Logging data with matlogger into " << matlogger_path  );

        logger = XBot::MatLogger2::MakeLogger(matlogger_path); // date-time automatically appended
        appender = XBot::MatAppender::MakeInstance();
        appender->add_logger(logger);
        appender->start_flush_thread();        
        logging = true;
    }
#endif

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
        //But be sure that someone has sent a motor command to the hal
        if (eeHalPtr->checkCommandPub()) {
            eeHalPtr->move();
            ROS_WARN_STREAM_ONCE ("[EEHalExecutor] someone is publishing on '/ros_end_effector/motor_reference_pos', I will call the move()" );   
            
        } else {
            ROS_WARN_STREAM_THROTTLE (60, "[EEHalExecutor] no-one is publishing on '/ros_end_effector/motor_reference_pos', I will not call the move()" );   
        }
        
        //send _js_msg to external (ie to ROSEE main node)
        eeHalPtr->publish_joint_state();
        
        
        
        if (eeHalPtr->_pressure_active) {
            eeHalPtr->publish_pressure();
        } 
        
#ifdef _MATLOGGER2        
        if (logging) {
            logger->add("motor_pos_ref", eeHalPtr->getMotorReference());
            logger->add("motor_pos", eeHalPtr->getJointPosition());
            logger->add("motor_eff", eeHalPtr->getJointEffort());
            auto pressures =  eeHalPtr->getPressure();
            logger->add("pressure1", pressures.row(0));
            logger->add("pressure2", pressures.row(1));
            logger->add("pressure3", pressures.row(2));
            logger->add("pressure4", pressures.row(3));
        }
#endif

        ros::spinOnce();
        r.sleep();
        
    }
    
    return 0;
    
}
