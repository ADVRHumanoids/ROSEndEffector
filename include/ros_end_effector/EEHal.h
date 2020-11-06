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


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <vector>
#include <memory>

namespace ROSEE {
    
    
    /**
     * @brief Class representing an end-effector
     * 
     */
    class EEHal  {

    public:
        
        typedef std::shared_ptr<EEHal> Ptr;
        typedef std::shared_ptr<const EEHal> ConstPtr;
        
        EEHal ( ros::NodeHandle* nh );
        virtual ~EEHal() {};
        
        virtual bool sense() = 0;
        
        virtual bool move() = 0;
        
        virtual bool publish_joint_state();
        
        //virtual bool setMotorPositionReference(std::string motor, double pos) = 0;
        //virtual bool getJointPosition(std::string joint, std::vector<double> &pos ) = 0;
                        
    protected:
        
        ros::NodeHandle* _nh; 
        
        sensor_msgs::JointState _mr_msg;
        ros::Subscriber _motor_reference_sub;
        virtual void motor_reference_clbk(const sensor_msgs::JointState::ConstPtr& msg);
        
        sensor_msgs::JointState _js_msg;
        ros::Publisher _joint_state_pub;
        
    private:
        
    };
    
}

#endif // __ROSEE_EE_HAL__
