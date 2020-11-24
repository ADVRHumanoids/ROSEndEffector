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

#include <ros/ros.h>

#include <ros_end_effector/HAL/EEHal.h>

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
        
        DummyHal( ros::NodeHandle* nh);
        virtual ~DummyHal() {};
        
        virtual bool sense() override;
        virtual bool move() override;
      
    private:
        /**
         * @brief this will publish to joint_state_publisher
         */
        ros::Publisher _hal_joint_state_pub;
        
        /**
         * @brief this will subscribe to joint_state_publisher
         */
        ros::Subscriber _hal_joint_state_sub;
        
        void hal_js_clbk(const sensor_msgs::JointState::ConstPtr& msg);
        
    };
    
HAL_CREATE_OBJECT(DummyHal)    
}

#endif // __ROSEE_DUMMY_HAL__
