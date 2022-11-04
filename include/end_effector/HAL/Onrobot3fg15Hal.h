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

#ifndef __ROSEE_ONROBOT3FG15_HAL__
#define __ROSEE_ONROBOT3FG15_HAL__

#include <ros/ros.h>

#include <end_effector/HAL/EEHal.h>
#include <end_effector/ModbusTCP.h>

#include <string>
#include <memory>

namespace ROSEE {
    
    /**
     * @brief Class representing an end-effector
     * 
     */
    class Onrobot3fg15Hal : public EEHal {

    public:
        
        typedef std::shared_ptr<Onrobot3fg15Hal> Ptr;
        typedef std::shared_ptr<const Onrobot3fg15Hal> ConstPtr;
        
        Onrobot3fg15Hal( ros::NodeHandle* nh);
        virtual ~Onrobot3fg15Hal() {};
        
        virtual bool init() override;
        virtual bool sense() override;
        virtual bool move() override;
      
    private:
        ModbusTCP mb_;
        std::string gripper_ip_;
        
        double min_joint_pos_, max_joint_pos_;
        uint8_t w_force_, w_diameter_, w_grip_type_, w_control_ ; // Writing values (control = 1->grasp(force), 2->move, 4->stop)
        uint8_t r_status_, r_raw_diameter_, r_diameter_fingertip_offset_, r_force_, r_finger_angle_,
            r_finger_length_, r_finger_position_, r_fingertip_offset_;
        uint8_t r_min_diameter_, r_max_diameter_;
        
        const int reg_address_wr_{0x0000};
        const int reg_address_rd_{0x0100};
        
        std::vector<uint16_t> values_rd_;
        
        bool fillReadedValues();
        bool readDiameterInit();

        
    };
    
HAL_CREATE_OBJECT(Onrobot3fg15Hal)    
}

#endif // __ROSEE_ONROBOT3FG15_HAL__
