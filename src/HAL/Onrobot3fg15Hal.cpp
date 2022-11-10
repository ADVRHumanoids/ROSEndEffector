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

ROSEE::Onrobot3fg15Hal::Onrobot3fg15Hal ( ros::NodeHandle *nh) : EEHal ( nh ), values_rd_(18), values_wr_(4) {
    
}

bool ROSEE::Onrobot3fg15Hal::init()
{
    //TODO hardcoded bad
    gripper_ip_ = "192.168.1.1";
    slave_device_num_ = 65;
    
    min_joint_pos_ = 0;
    max_joint_pos_ = 2.9;

    int attempt_num = 3;
    int attempt_cnt = 0;

    while (!mb_.connectDevice(gripper_ip_, MODBUS_TCP_DEFAULT_PORT, slave_device_num_) && (attempt_cnt < attempt_num)) 
    {
        std::cout << "Onrobot3fg15Hal: Unable to connect to " << gripper_ip_ << ". Attempt " << attempt_cnt+1 << "/" << attempt_num << ".\n";
        attempt_cnt++;
        ros::Duration(2.0).sleep();
    }
    if (attempt_cnt == attempt_num)
        return false;
    
    if (!(readDiameterInit() && readWriteFingerSetupInit() && fillReadedValues()))
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

    if (!fillReadedValues()) {
        std::cout << "Onrobot3fg15Hal::getGripperWidthClosure(): Error in reading width closure" << std::endl;
        return false;
    }

    double old_range = (r_min_diameter_ - r_max_diameter_);  
    double new_range = (max_joint_pos_ - min_joint_pos_);  
    
    _js_msg.position[0] = (((r_raw_diameter_ - r_max_diameter_) * new_range) / old_range) + min_joint_pos_;
    
    _js_msg.header.stamp = ros::Time::now();
    _js_msg.header.seq =_js_msg.header.seq+1;

    return true;
}

bool ROSEE::Onrobot3fg15Hal::move() {

    if(_mr_msg.position.size() != 1) {
        std::cout << "Onrobot3fg15Hal::move: Error mr position message size is not 1 but: " << _mr_msg.position.size() << std::endl;
        return false;
    } 
    
    double old_range = (max_joint_pos_ - min_joint_pos_);  
    double new_range = (r_min_diameter_ - r_max_diameter_);  
    double diameter = (((_mr_msg.position[0] - min_joint_pos_) * new_range) / old_range) + r_max_diameter_; // diameter is 1/10 mm 

    w_force_ = 0;
    w_diameter_ = std::round(diameter);
    w_grip_type_ = 0;
    w_control_ = 2;

    fillValuesToWrite();

    if(!mb_.write(reg_address_wr_, values_wr_)) {
        std::cout << "Onrobot3fg15Hal::move: Error in writing on register " << reg_address_wr_ << std::endl;
        return false;
    }

    return true;
}

void ROSEE::Onrobot3fg15Hal::fillValuesToWrite() {

    values_wr_[0] = w_force_;
    values_wr_[1] = w_diameter_;
    values_wr_[2] = w_grip_type_;
    values_wr_[3] = w_control_;
    
}

bool ROSEE::Onrobot3fg15Hal::fillReadedValues() {
    if(!mb_.read(reg_address_rd_, values_rd_.size(), values_rd_)) {
        std::cout << "Onrobot3fg15Hal: Error in reading register " << reg_address_rd_ << std::endl;
        return false;
    }

    r_status_ = values_rd_[0]; //just the first 4 bit are significant, other are reserved
    r_raw_diameter_ = values_rd_[1];
    r_diameter_fingertip_offset_ = values_rd_[2];
    
    r_force_ = values_rd_[3];
    r_finger_angle_ = values_rd_[5];
    r_finger_length_ = values_rd_[14];
    r_finger_position_ = values_rd_[16];
    r_fingertip_offset_ = values_rd_[17];
    
    //cout for debug
//     std::cout << "r_status_:" << r_status_ << "\n";
//    std::cout << "r_raw_diameter_ (1/10mm): " << unsigned(r_raw_diameter_) << "\n";
//    std::cout << "max (1/10mm): " << unsigned(r_max_diameter_) << "\n";
//    std::cout << "min (1/10mm): " << unsigned(r_min_diameter_) << "\n";
//     std::cout << "r_diameter_fingertip_offset_:" << unsigned(r_diameter_fingertip_offset_) << "\n";
//     std::cout << "r_force_:" << unsigned(r_force_) << "\n";
//     std::cout << "r_finger_angle_:" << unsigned(r_finger_angle_) << "\n";
//     std::cout << "r_finger_length_:" << unsigned(r_finger_length_) << "\n";
//     std::cout << "r_finger_position_:" << unsigned(r_finger_position_) << "\n";
//     std::cout << "r_fingertip_offset_:" << unsigned(r_fingertip_offset_) << "\n";
    
//     std::cout << std::endl;
    

    return true;
}

bool ROSEE::Onrobot3fg15Hal::readDiameterInit() {
    
    std::vector<uint16_t> max_min_diameter;
    const int reg_address = 0x0201;
    const int n_address = 2;

    if (!mb_.read(reg_address, n_address, max_min_diameter)) {
        std::cout << "Onrobot3fg15Hal: Error in reading register " << reg_address << std::endl;
        return false;
    }
    
    //r_min_diameter_ = max_min_diameter[0];
    r_min_diameter_ = 346;  //true tested min
    r_max_diameter_ = max_min_diameter[1];
    
    //cout for debug
//    std::cout << "r_min_diameter_: (1/10mm): " << unsigned(r_min_diameter_) << "\n";
//    std::cout << "r_max_diameter_: " << unsigned(r_max_diameter_) << "\n";
    
    return true;
}

bool ROSEE::Onrobot3fg15Hal::readWriteFingerSetupInit() {
    std::vector<uint16_t> finger_setup_data;
    
    const int reg_addres = 0x0401;
    if(!mb_.read(reg_addres, 4, finger_setup_data)) {
        std::cout << "Onrobot3fg15Hal: Error in reading register 0x0401" << std::endl;
        return false;
    }
    
    //cout for debug
    //std::cout << "finger lenght: (1/10mm): " << finger_setup_data[0] << "\n";
    //std::cout << "finger position (1-2-3): " << finger_setup_data[2] << "\n";
    //std::cout << "finger offset: (1/100mm): " << finger_setup_data[3] << "\n";
    
    finger_setup_data[2] = 2; //position of the finger mount (1 2 3)
    
    if(!mb_.write(reg_addres, finger_setup_data)) {
        std::cout << "Onrobot3fg15Hal::move: Error in writing on register " << 0x0401 << std::endl;
        return false;
    }

    
    return true;
}
