#ifndef __ROSEE_XBOT2_HAL__
#define __ROSEE_XBOT2_HAL__

#include <ros_end_effector/EEHal.h>
#include <xbot2/hal/device.h>

#include <string>
#include <memory>

namespace ROSEE {
    
    
/**
    * @brief Concrete class which communicate directly with ROS topics
    * 
    */
class XBot2Hal : public EEHal
                 public Xbot::Hal::DeviceClientTpl<ROSEEPacket::Rx,
                                        ROSEEPacket::Tx> {

public:
    
    typedef std::shared_ptr<XBot2Hal> Ptr;
    typedef std::shared_ptr<const XBot2Hal> ConstPtr;
    
    XBot2Hal( ROSEE::EEInterface::Ptr ee_interface );
    virtual ~XBot2Hal();
    
    bool sense() override;
    
    bool move() override;
    
    bool setMotorPositionReference (std::string motor_name, double position_ref) override;
    
    bool getMotorPosition (std::string motor_name, double &position) override; 
    
    
private:
    
    //xbot stuffs
    ROSEE::XBotEEBase * _xbot_ee;
    

};

class XbotEEDriver : public DeviceDriverTpl<ROSEEPacket::Rx,
                                        ROSEEPacket::Tx>
{

public:

    XbotEEDriver(DeviceInfo devinfo);

    /**
     * @brief move_impl allows to override the tx data from
     * the framework before it's actually sent to the device
     */
    bool move_impl() override;

    /**
     * @brief sense_impl allows to override the rx data from
     * the device before it's actually sent to the framework
     * @return if return false, rx is not actually sent to
     * the framework!
     */
    bool sense_impl() override;

private:

    std::chrono::nanoseconds _timeout;
};
    
} //namespace roseee

#endif // __ROSEE_XBOT2_HAL__
