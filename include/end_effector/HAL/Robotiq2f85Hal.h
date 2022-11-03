#ifndef __ROSEE_ROBOTIQ2F85_HAL__
#define __ROSEE_ROBOTIQ2F85_HAL__

#include <end_effector/HAL/EEHal.h>
#include <end_effector/ModbusRTU.h>
#include <string>
#include <memory>

namespace ROSEE {

/**
    * @brief Concrete class which communicate directly with ROS topics
    * 
    */
class Robotiq2f85Hal : public ROSEE::EEHal
{

public:
    
    typedef std::shared_ptr<Robotiq2f85Hal> Ptr;
    typedef std::shared_ptr<const Robotiq2f85Hal> ConstPtr;

    Robotiq2f85Hal( ros::NodeHandle* nh );
    virtual ~Robotiq2f85Hal();
    
    virtual bool init() override;
    
    virtual bool sense() override;
    
    virtual bool move() override;
        
private:
    ModbusRTU mb_;
    std::string gripper_ip_;
    const int slave_id_{0x0009};
    const int baud_rate_{115200};
    const char parity_{'N'};
    const int data_bit_{8};
    const int stop_bit_{1};
    const int reg_address_wr{0x03E8};
    const int reg_address_rd{0x07D0};
    std::vector<uint16_t> values_, data_, data_rd_;
    std::vector<uint16_t> values_rd_;

    int rACT_, rGTO_, rSP_, rFR_, rATR_, rPR_; // Writing values
    int gACT_, gGTO_, gSTA_, gOBJ_, gFLT_, gPR_, gPO_, gCU_; // reading values

    const int max_width_value_{0};
    const int min_width_value_{230};
    double width_closure_, max_width_, min_width_, error_tollerance_;

    void fillBytes();
    bool fillReadedValues();
    
    double min_joint_pos_, max_joint_pos_;

};

HAL_CREATE_OBJECT(Robotiq2f85Hal)
    
} //namespace roseee

#endif // __ROSEE_ROBOTIQ2F85_HAL__
