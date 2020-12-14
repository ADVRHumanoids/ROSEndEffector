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
    
    _hand_info_present = parseHandInfo();
    
}

bool ROSEE::EEHal::isHandInfoPresent() { return _hand_info_present; }

void ROSEE::EEHal::motor_reference_clbk(const sensor_msgs::JointState::ConstPtr& msg) {
    
    _mr_msg = *msg;
    
}

bool ROSEE::EEHal::publish_joint_state() {
    
    //NOTE _js_msg must be filled by the derived class
    _joint_state_pub.publish(_js_msg);
    
    return true;
    
}

bool ROSEE::EEHal::getFingersNames(std::vector<std::string> &fingers_names){
    
    if (this->fingers_names.size() != 0) {
    
        fingers_names = this->fingers_names;
        return true;

    } else {
        return false;
    }
    
}
bool ROSEE::EEHal::getMotorsNames(std::vector<std::string> &motors_names){
    
    if (this->motors_names.size() != 0) {
    
        motors_names = this->motors_names;
        return true;

    } else {
        return false;
    }
    
}

bool ROSEE::EEHal::getMotorStiffnessDiagonal(Eigen::VectorXd &motors_stiffness_diagonal){
    
    if (this->motors_stiffness_diagonal.size() != 0) {
    
        motors_stiffness_diagonal = this->motors_stiffness_diagonal;
        return true;

    } else {
        return false;
    }
    
}

bool ROSEE::EEHal::getTipsFrictions(Eigen::VectorXd &tips_frictions){
    
    if (this->tips_frictions.size() != 0) {
    
        tips_frictions = this->tips_frictions;
        return true;

    } else {
        return false;
    }
    
}

bool ROSEE::EEHal::getMotorTorqueLimits(Eigen::VectorXd &motors_torque_limits){
    
    if (this->motors_torque_limits.size() != 0) {
    
        motors_torque_limits = this->motors_torque_limits;
        return true;

    } else {
        return false;
    }
    
}

bool ROSEE::EEHal::parseHandInfo() {
    
    std::string _rosee_config_path;
    if (! _nh->getParam ( "/ros_ee_config_path", _rosee_config_path )) {
        return false;
    }
    
    std::ifstream ifile(_rosee_config_path);
    if (! ifile) {
        ROS_WARN_STREAM ("EEHALExecutor: config file " << _rosee_config_path << " not found");
        return false;
    }
    
    YAML::Node node = YAML::LoadFile(_rosee_config_path);
    
    if (! node["hand_info"]) {
        ROS_WARN_STREAM ("EEHALExecutor: config file " << _rosee_config_path << " does not contain "
            << "hand_info node. I will not parse hand information");
        
        return false;
    }
    
    if(node["hand_info"]["fingers_names"]){

       fingers_names = node["hand_info"]["fingers_names"].as<std::vector<std::string>>();
    }
    
    if(node["hand_info"]["motors_names"]){

       motors_names = node["hand_info"]["motors_names"].as<std::vector<std::string>>();
    }
    
    if(node["hand_info"]["motors_stiffness_diagonal"]){

       motors_stiffness_diagonal = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["motors_stiffness_diagonal"]);
    }
    
    if(node["hand_info"]["tips_frictions"]){

       tips_frictions = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["tips_frictions"]);
    }
    
    if(node["hand_info"]["motors_torque_limits"]){

       motors_torque_limits = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["motors_torque_limits"]);
    }
    
    return true;
}
