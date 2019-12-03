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

#ifndef __ROSEE_I_MOTOR__
#define __ROSEE_I_MOTOR__

#include <string>
#include <memory>


namespace ROSEE {
    
    
    /**
     * @brief Class representing a Motor
     * 
     */
    class IMotor {

    public:
        
        typedef std::shared_ptr<IMotor> Ptr;
        typedef std::shared_ptr<const IMotor> ConstPtr;
        
        virtual bool getMotorPosition( std::string motor_name, double& motor_position ) = 0;
        virtual bool getMotorVelocity( std::string motor_name, double& motor_velocity ) = 0;
        virtual bool getMotorEffort( std::string motor_name, double& motor_effort ) = 0;
        
        virtual bool setPositionReference ( std::string motor_name, double position_reference ) = 0;
        
        
    };
    
}

#endif // __ROSEE_I_MOTOR__
