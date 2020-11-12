#ifndef __ROSEE_ROS_HAL__
#define __ROSEE_ROS_HAL__

#include <ros_end_effector/HAL/EEHal.h>

#include <string>
#include <memory>

namespace ROSEE {
    
    
    /**
     * @brief Concrete class which communicate directly with ROS topics
     * 
     */
    class ROSHal : public EEHal  {

    public:
        
        typedef std::shared_ptr<ROSHal> Ptr;
        typedef std::shared_ptr<const ROSHal> ConstPtr;
        
        ROSHal( EEHal::EEInterface::Ptr ee_interface );
        virtual ~ROSHal();
        
        bool sense() override;
        
        bool move() override;
        
        bool setMotorPositionReference (std::string motor_name, double position_ref) override;
        
        bool getMotorPosition (std::string motor_name, double &position) override; 
        
    private:
        
        
        //std::map<std::string, double> _joint_position;
        //std::map<std::string, double> _joint_velocity;
        //std::map<std::string, double> _joint_effort;
    };
    
}

#endif // __ROSEE_ROS_HAL__
