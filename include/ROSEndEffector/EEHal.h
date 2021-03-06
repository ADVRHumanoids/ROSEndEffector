/*
 * Copyright (C) 2019 IIT-HHCM
 * Author: Luca Muratore
 * email:  luca.muratore@iit.it
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef __ROSEE_EE_HAL__
#define __ROSEE_EE_HAL__

#include <ROSEndEffector/IMotor.h>

#include <ROSEndEffector/EEInterface.h>

#include <string>
#include <memory>

namespace ROSEE {
    
    
    /**
     * @brief Class representing an end-effector
     * 
     */
    class EEHal : public IMotor {

    public:
        
        typedef std::shared_ptr<EEHal> Ptr;
        typedef std::shared_ptr<const EEHal> ConstPtr;
        
        EEHal( ROSEE::EEInterface::Ptr ee_interface );
        virtual ~EEHal();
        
        virtual bool sense() = 0;
        
        virtual bool move() = 0;
        
    protected:
        
        virtual bool setJointPosition( std::string joint_name, double joint_position);
        virtual bool setJointVelocity( std::string joint_name, double joint_velocity);
        virtual bool setJointEffort( std::string joint_name, double joint_effort);
        
        virtual bool getJointPosition( std::string joint_name, double& joint_position);
        virtual bool getJointVelocity( std::string joint_name, double& joint_velocity);
        virtual bool getJointEffort( std::string joint_name, double& joint_effort);
        
        ROSEE::EEInterface::Ptr _ee_inteface;
        
    private:
        
        
        std::map<std::string, double> _joint_postion;
        std::map<std::string, double> _joint_velocity;
        std::map<std::string, double> _joint_effort;
    };
    
}

#endif // __ROSEE_EE_HAL__
