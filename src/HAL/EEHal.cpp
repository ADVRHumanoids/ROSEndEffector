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

#include <ros_end_effector/HAL/EEHal.h>

ROSEE::EEHal::EEHal(ros::NodeHandle* nh) {
    
    _nh = nh;
    
    //init sub to receive reference from UniversalROSEEEX
    //TODO take topic name from roslaunch
    std::string motor_reference_topic  = "/ros_end_effector/motor_reference_pos";

    _motor_reference_sub = _nh->subscribe(motor_reference_topic, 1,
                                          &ROSEE::EEHal::motor_reference_clbk, this);
    
    std::string joint_state_topic = "/ros_end_effector/joint_states";
    
    _joint_state_pub = _nh->advertise<sensor_msgs::JointState>(joint_state_topic, 10);
    
}

void ROSEE::EEHal::motor_reference_clbk(const sensor_msgs::JointState::ConstPtr& msg) {
    
    _mr_msg = *msg;
    
}

bool ROSEE::EEHal::publish_joint_state() {
    
    _joint_state_pub.publish(_js_msg);
    
    return true;
    
}
