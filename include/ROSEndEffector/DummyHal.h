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

#ifndef __ROSEE_DUMMY_HAL__
#define __ROSEE_DUMMY_HAL__

#include <ROSEndEffector/EEHal.h>

#include <string>
#include <memory>

namespace ROSEE {
    
    
    /**
     * @brief Class representing an end-effector
     * 
     */
    class DummyHal : public EEHal {

    public:
        
        typedef std::shared_ptr<DummyHal> Ptr;
        typedef std::shared_ptr<const DummyHal> ConstPtr;
        
        DummyHal( ROSEE::EEInterface::Ptr ee_interface );
        virtual ~DummyHal();
        
        
        virtual bool getMotorPosition( std::string joint_name, double& motor_position );
        virtual bool getMotorVelocity( std::string joint_name, double& motor_velocity );
        virtual bool getMotorEffort( std::string joint_name, double& motor_effort );
        
        virtual bool setPositionReference( std::string joint_name, double position_reference );
                
        
        
        virtual bool sense();
        virtual bool move();
        
        
    };
    
}

#endif // __ROSEE_I_DUMMY_HAL__
