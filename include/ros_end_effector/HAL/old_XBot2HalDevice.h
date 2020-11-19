#ifndef __ROSEE_XBOT2_HAL__
#define __ROSEE_XBOT2_HAL__

#include <ros_end_effector/HAL/EEHal.h>
#include <ros_end_effector/HAL/XBot2HalCommunication.h>
#include <xbot2/xbot2.h>
#include <xbot2/robot_interface/robot_interface_xbot_rt.h>



#include <string>
#include <memory>

namespace ROSEE {

/**
    * @brief Concrete class which communicate directly with ROS topics
    * 
    */
class XBot2HalDevice : public ROSEE::EEHal
{

public:
    
    typedef std::shared_ptr<XBot2HalDevice> Ptr;
    typedef std::shared_ptr<const XBot2HalDevice> ConstPtr;

    XBot2HalDevice( ros::NodeHandle* nh );
    virtual ~XBot2HalDevice() { };
    
    bool sense() ;
    
    bool move() ;
        
private:
    
    ROSEE::XBotEEBase*_device;
    XBot::RobotInterface::Ptr _robot;

    XBot::JointNameMap _jointPositionActualMap;
    XBot::JointNameMap _jointPositionReferenceMap;

};


    
} //namespace roseee

#endif // __ROSEE_XBOT2_HAL__
