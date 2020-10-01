#ifndef __ROSEE_XBOT2_HAL__
#define __ROSEE_XBOT2_HAL__

#include <ros_end_effector/EEHal.h>
#include <ros_end_effector/XBot2HalCommunication.h>
#include <xbot2/xbot2.h>


#include <string>
#include <memory>

namespace ROSEE {

/**
    * @brief Concrete class which communicate directly with ROS topics
    * 
    */
class XBot2Hal : public EEHal 
{

public:
    
    typedef std::shared_ptr<XBot2Hal> Ptr;
    typedef std::shared_ptr<const XBot2Hal> ConstPtr;
    
    XBot2Hal( ROSEE::EEInterface::Ptr ee_interface );
    virtual ~XBot2Hal() { };
    
    bool sense() override;
    
    bool move() override;
    
    bool setMotorPositionReference (std::string motor_name, double position_ref) override;
    
    bool getMotorPosition (std::string motor_name, double &position) override; 
    
    
private:
    
    //xbot stuffs
    std::shared_ptr<ROSEE::XBotEEBase> _xbot_ee_client;
    //ROSEE::XBotEEClient _xbot_ee_client;
    

};


    
} //namespace roseee

#endif // __ROSEE_XBOT2_HAL__
