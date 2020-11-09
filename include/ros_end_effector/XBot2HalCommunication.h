#ifndef __ROSEE_XBOT2_HAL_COMMUNICATION__
#define __ROSEE_XBOT2_HAL_COMMUNICATION__

#include <ros_end_effector/ROSEEPacket.h>
#include <xbot2/xbot2.h>
#include <xbot2/hal/device.h>
#include <xbot2/client_server/client_manager.h>


#include <string>
#include <memory>

namespace ROSEE {
   
class XBotEEBase : virtual public XBot::Hal::DeviceBase
{

public:

    XBOT2_DECLARE_SMART_PTR(XBotEEBase)

    virtual bool setMotorPositionReference (std::string motor_name, double position_ref) = 0;
    
    virtual bool getMotorPosition (std::string motor_name, double &position) = 0; 

};   

using XBotEEClientBase = XBot::Hal::DeviceClientTpl<ROSEEPacket::Rx, ROSEEPacket::Tx>;
    

class XBotEEClient : public XBotEEClientBase,
                     public virtual XBotEEBase
{

public:

    using XBotEEClientBase::XBotEEClientBase;
    
    XBotEEClient(XBot::Hal::DeviceInfo);

    bool setMotorPositionReference (std::string motor_name, double position_ref) override;
    
    bool getMotorPosition (std::string motor_name, double &position) override; 

};

/**
 * @brief The RocketClientContainer class implements optional API
 * methods referring to multiple rockets together.
 */
class XBotEEClientContainer : public XBot::Hal::DeviceContainer<XBotEEClient>
{

public:

    using XBot::Hal::DeviceContainer<XBotEEClient>::DeviceContainer;

};



class XBotGazeboDriver : public XBot::Hal::DeviceDriverTpl<ROSEEPacket::Rx, ROSEEPacket::Tx>
{

public:

    XBotGazeboDriver(XBot::Hal::DeviceInfo devinfo);

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

    //std::chrono::nanoseconds _timeout;
};

/**
 * @brief The RocketDev class implements the client side
 * for all rocket device instances. If the protocol in use is
 * xbot2's client-server protocol, this is pure boilerplate
 * code that can be autogenerated. Otherwise, the user must
 * write the code to connect to the remote device.
 */
class XBotGazeboDriverContainer : public XBot::Hal::DeviceContainer<XBotGazeboDriver>
{

public:

    XBotGazeboDriverContainer(std::vector<XBot::Hal::DeviceInfo> devinfo);

    bool sense_all() override;

    bool move_all() override;

private:

    bool _srv_alive;

    XBot::ClientManager::UniquePtr _cli;

};
    
} //namespace roseee

#endif // __ROSEE_XBOT2_HAL_COMMUNICATION__
