#ifndef __ROSEE_XBOT2_HAL__
#define __ROSEE_XBOT2_HAL__

#include <ros_end_effector/HAL/EEHal.h>
#include <xbot2/xbot2.h>
#include <xbot2/robot_interface/robot_interface_xbot_rt.h>
#include <std_srvs/SetBool.h>


#include <string>
#include <memory>

namespace ROSEE {

/**
    * @brief Concrete class which communicate directly with ROS topics
    * 
    */
class XBot2Hal : public ROSEE::EEHal
{

public:
    
    typedef std::shared_ptr<XBot2Hal> Ptr;
    typedef std::shared_ptr<const XBot2Hal> ConstPtr;

    XBot2Hal( ros::NodeHandle* nh );
    virtual ~XBot2Hal() { };
    
    virtual bool sense() override;
    
    virtual bool move() override;
        
private:
    
    XBot::RobotInterface::Ptr _robot;
    XBot::JointNameMap _jointPositionActualMap;
    XBot::JointNameMap _jointPositionReferenceMap;

};

HAL_CREATE_OBJECT(XBot2Hal)
    
} //namespace roseee

#endif // __ROSEE_XBOT2_HAL__
