/*
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

#ifndef __ROSEE_DAGANA_HAL__
#define __ROSEE_DAGANA_HAL__

#include <ros/ros.h>

#include <end_effector/HAL/EEHal.h>

#include <string>
#include <memory>

namespace ROSEE {
    
    /**
     * @brief Class representing an end-effector
     * 
     */
    class DaganaHal : public EEHal {

    public:
        
        typedef std::shared_ptr<DaganaHal> Ptr;
        typedef std::shared_ptr<const DaganaHal> ConstPtr;
        
        DaganaHal( ros::NodeHandle* nh);
        virtual ~DaganaHal() {};
        
        virtual bool init() override;
        virtual bool sense() override;
        virtual bool move() override;
      
    private:
        
        std::string _dagana_name;
        
        /**
         * @brief this will publish to the dagana topic
         */
        ros::Publisher _hal_joint_state_pub;
        
        /**
         * @brief this will subscribe to the dagana topic
         */
        ros::Subscriber _hal_joint_state_sub;
        
        void hal_js_clbk(const sensor_msgs::JointState::ConstPtr& msg);
        
    };
    
HAL_CREATE_OBJECT(DaganaHal)    
}

#endif // __ROSEE_DAGANA_HAL__
