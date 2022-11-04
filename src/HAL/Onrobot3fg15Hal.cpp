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

#include <end_effector/HAL/Onrobot3fg15Hal.h>

ROSEE::Onrobot3fg15Hal::Onrobot3fg15Hal ( ros::NodeHandle *nh) : EEHal ( nh ), values_rd_(18) {
    
}

bool ROSEE::Onrobot3fg15Hal::init()
{
    //TODO hardcoded bad
    gripper_ip_ = "192.168.1.1";
    min_joint_pos_ = 0;
    max_joint_pos_ = 0.81;

    int attempt_num = 3;
    int attempt_cnt = 0;

    while (!mb_.connectDevice(gripper_ip_, MODBUS_TCP_DEFAULT_PORT) && (attempt_cnt < attempt_num)) 
    {
        std::cout << "Onrobot3fg15Hal: Unable to connect to " << gripper_ip_ << ". Attempt " << attempt_cnt+1 << "/" << attempt_num << ".\n";
        attempt_cnt++;
        ros::Duration(2.0).sleep();
    }
    if (attempt_cnt == attempt_num)
        return false;

    if (!(readDiameterInit() && fillReadedValues()))
        return false;
    
    // Reset the variables
    w_force_ = 0;
    w_diameter_ = r_raw_diameter_;
    w_grip_type_ = 0;
    w_control_ = 4; // stop
    
    _js_msg.position.resize(1);
    _js_msg.velocity.resize(1);
    _js_msg.effort.resize(1);
    _js_msg.name.resize(1);
    
    _mr_msg.position.resize(1);
    _mr_msg.velocity.resize(1);
    _mr_msg.effort.resize(1);
    _mr_msg.name.resize(1);
    
    _mr_msg.position[0] = 0;
    
    _js_msg.header.stamp = ros::Time::now();
    _js_msg.header.seq =0;
    
    //TODO
    _js_msg.name[0] = "robot_onrobot_3fg15_finger_joint1";

    return true;
}



bool ROSEE::Onrobot3fg15Hal::sense() {

    return true;
}

bool ROSEE::Onrobot3fg15Hal::move() {

    return true;
}

// void ROSEE::Onrobot3fg15Hal::fillBytes() {
//   data_.clear();
//   data_.push_back(rACT_ + (rGTO_<<3) + (rATR_<<4));
//   data_.push_back(0);
//   data_.push_back(0);
//   data_.push_back(rPR_);
//   data_.push_back(rSP_);
//   data_.push_back(rFR_);
// 
//   values_.clear();
//   // Fill message by combining two bytes in one register
//   for(int i=0; i<(data_.size()/2); i++)
//      values_.push_back((data_[2*i] << 8) + data_[2*i+1]);
// }

bool ROSEE::Onrobot3fg15Hal::fillReadedValues() {
    if(!mb_.read(reg_address_rd_, values_rd_.size(), values_rd_)) {
        std::cout << "Onrobot3fg15Hal: Error in reading register " << reg_address_rd_ << std::endl;
        return false;
    }

    r_status_ = values_rd_[0];
    r_raw_diameter_ = values_rd_[1];
    r_diameter_fingertip_offset_ = values_rd_[2];
    r_force_ = values_rd_[3];
    r_finger_angle_ = values_rd_[5];
    r_finger_length_ = values_rd_[14];
    r_finger_position_ = values_rd_[16];
    r_fingertip_offset_ = values_rd_[17];
    

    return true;
}

bool ROSEE::Onrobot3fg15Hal::readDiameterInit() {
    std::vector<uint16_t> max_min_diameter;
    if(!mb_.read(0x0201, 2, max_min_diameter)) {
        std::cout << "Onrobot3fg15Hal: Error in reading register " << reg_address_rd_ << std::endl;
        return false;
    }
    
    r_min_diameter_ = max_min_diameter[0];
    r_max_diameter_ = max_min_diameter[1];
    
    return true;
}
