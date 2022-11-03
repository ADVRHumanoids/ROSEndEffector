#include <end_effector/HAL/Robotiq2f85Hal.h>

ROSEE::Robotiq2f85Hal::Robotiq2f85Hal ( ros::NodeHandle *nh) : EEHal ( nh ), values_rd_(3) {
}


ROSEE::Robotiq2f85Hal::~Robotiq2f85Hal() {
    
    rACT_ = 0;
    rGTO_ = 0;
    rATR_ = 0;
    rPR_ = 0;
    rSP_ = 0;
    rFR_ = 0;

    fillBytes();

    if(!mb_.write(reg_address_wr, values_)) {
        std::cout << "Robotiq2FGripperInterface (deactivation): Error in writing on register " << reg_address_wr << std::endl;
    }

    mb_.closeConnection();
}

bool ROSEE::Robotiq2f85Hal::init() {
    
    //if(!config_options_.getParameter("gripper_ip", gripper_ip_)) 
    //    return false;
    //  if(!config_options_.getParameter("epsilon", epsilon_)) return false;
    
    //TODO hardcoded bad
    gripper_ip_ = "/dev/ttyUSB0";
    min_joint_pos_ = 0;
    max_joint_pos_ = 0.81;

    int attempt_num = 3;
    int attempt_cnt = 0;

    while (!mb_.connectDevice(gripper_ip_, slave_id_, baud_rate_, parity_, data_bit_, stop_bit_) && (attempt_cnt < attempt_num)) 
    {
        std::cout << "Robotiq2FGripperInterface: Unable to connect to " << gripper_ip_ << ". Attempt " << attempt_cnt+1 << "/" << attempt_num << ".\n";
        attempt_cnt++;
        ros::Duration(2.0).sleep();
    }
    if (attempt_cnt == attempt_num)
        return false;

    // Reset the gripper
    rACT_ = 0;
    rGTO_ = 0;
    rATR_ = 0;
    rPR_ = 0;
    rSP_ = 0;
    rFR_ = 0;

    fillBytes();

    if(!mb_.write(reg_address_wr, values_)) {
        std::cout << "Robotiq2FGripperInterface (activation): Error in writing on register " << reg_address_wr << "\n";
        return false;
    }

    // Activate the gripper
    rACT_ = 1;
    rGTO_ = 1;
    rATR_ = 0;
    rPR_ = 0;
    rSP_ = 255;
    rFR_ = 150;

    fillBytes();

    if(!mb_.write(reg_address_wr, values_)) {
        std::cout << "Robotiq2FGripperInterface (activation): Error in writing on register " << reg_address_wr << "\n";
        return false;
    }

    min_width_ = 0.;
    max_width_ = 0.85;

    error_tollerance_ = 0.02;
    
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
    _js_msg.name[0] = "finger_joint";

    return true;
}


bool ROSEE::Robotiq2f85Hal::sense() {
    
    if (!fillReadedValues()) {
        std::cout << "Robotiq2FGripperInterface::getGripperWidthClosure(): Error in reading width closure" << std::endl;
        return false;
    }

    width_closure_ = max_width_ - (max_width_ * gPO_ / min_width_value_);

    double old_range = (min_width_ - max_width_);  
    double new_range = (max_joint_pos_ - min_joint_pos_);  
    
    _js_msg.position[0] = (((width_closure_ - max_width_) * new_range) / old_range) + min_joint_pos_;
    
    _js_msg.header.stamp = ros::Time::now();
    _js_msg.header.seq =_js_msg.header.seq+1;

    return true;
}

bool ROSEE::Robotiq2f85Hal::move() {
    
    if(_mr_msg.position.size() != 1) {
        std::cout << "Robotiq2FGripperInterface::move: Error mr position message size is not 1 but: " << _mr_msg.position.size() << std::endl;
        return false;
    } 
    
    double old_range = (max_joint_pos_ - min_joint_pos_);  
    double new_range = (min_width_ - max_width_);  
    double width = (((_mr_msg.position[0] - min_joint_pos_) * new_range) / old_range) + max_width_;
    
    double speed = 70;
    
    double wid, spe;

    wid = (min_width_value_*(max_width_-width)/max_width_);
    spe = 255*speed/100;

    rACT_ = 1;
    rGTO_ = 1;
    rATR_ = 0;
    rPR_ = wid;
    rSP_ = spe;
    rFR_ = 150;

    fillBytes();

    if(!mb_.write(reg_address_wr, values_)) {
        std::cout << "Robotiq2FGripperInterface::move: Error in writing on register " << reg_address_wr << std::endl;
        return false;
    }

//     do {
//         if (!fillReadedValues()) {
//         std::cout << "Robotiq2FGripperInterface::move: Error in reading values" << std::endl;
//         return -1;
//         }
//     } while (gGTO_ == 0 || gOBJ_ == 0);

    return true;
}

void ROSEE::Robotiq2f85Hal::fillBytes() {
  data_.clear();
  data_.push_back(rACT_ + (rGTO_<<3) + (rATR_<<4));
  data_.push_back(0);
  data_.push_back(0);
  data_.push_back(rPR_);
  data_.push_back(rSP_);
  data_.push_back(rFR_);

  values_.clear();
  //Fill message by combining two bytes in one register
  for(int i=0; i<(data_.size()/2); i++)
     values_.push_back((data_[2*i] << 8) + data_[2*i+1]);
}

bool ROSEE::Robotiq2f85Hal::fillReadedValues() {
    if(!mb_.read(reg_address_rd, values_rd_.size(), values_rd_)) {
        std::cout << "Robotiq2FGripperInterface: Error in reading register " << reg_address_rd << std::endl;
        return false;
    }

    data_rd_.clear();
    for(int i=0; i<values_rd_.size(); i++) {
        data_rd_.push_back((values_rd_[i] & 0xFF00) >> 8);
        data_rd_.push_back(values_rd_[i] & 0x00FF);
    }

    gACT_ = (data_rd_[0] >> 0) & 0x01;
    gGTO_ = (data_rd_[0] >> 3) & 0x01;
    gSTA_ = (data_rd_[0] >> 4) & 0x03;
    gOBJ_ = (data_rd_[0] >> 6) & 0x03;
    gFLT_ =  data_rd_[2];
    gPR_ =  data_rd_[3];
    gPO_ =  data_rd_[4];
    gCU_ =  data_rd_[5];

    return true;
}




