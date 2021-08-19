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

#include <ros_end_effector/HAL/DummyHal.h>

ROSEE::DummyHal::DummyHal ( ros::NodeHandle *nh) : EEHal ( nh ) {
    
    std::string out;
    
    _hal_joint_state_pub = nh->advertise<sensor_msgs::JointState>("/dummyHal/joint_command", 1);
    _hal_joint_state_sub = nh->subscribe("/dummyHal/joint_states", 1, &ROSEE::DummyHal::hal_js_clbk, this);
}


bool ROSEE::DummyHal::sense() {
    //do nothing, it is the hal_js_clbk who "sense"
    return true;
}

bool ROSEE::DummyHal::move() {
    _hal_joint_state_pub.publish(_mr_msg);
    return true;
}

void ROSEE::DummyHal::hal_js_clbk(const sensor_msgs::JointState::ConstPtr& msg) {
    
    _js_msg = *msg;
}

