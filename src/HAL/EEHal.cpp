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

#include <end_effector/HAL/EEHal.h>

ROSEE::EEHal::EEHal(ros::NodeHandle* nh) {
    
    _nh = nh;
    
    //init sub to receive reference from UniversalROSEEEX
    //TODO take topic name from roslaunch
    std::string motor_reference_topic  = "/ros_end_effector/motor_reference_pos";

    _motor_reference_sub = _nh->subscribe(motor_reference_topic, 10,
                                          &ROSEE::EEHal::motor_reference_clbk, this);
    
    std::string joint_state_topic = "/ros_end_effector/joint_states";
    
    _joint_state_pub = _nh->advertise<sensor_msgs::JointState>(joint_state_topic, 10);    
    
    _hand_info_present = parseHandInfo();
   
    if (_hand_info_present) { 
        _hand_info_service_name = "hand_info";
    }
    
    _pressure_active = false; // if a derived class want to use this, it must call initPressureSensing()
    

    
}

bool ROSEE::EEHal::checkCommandPub() {
    
    return (_motor_reference_sub.getNumPublishers() > 0 && _mr_msg.name.size() != 0);
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

bool ROSEE::EEHal::getMotorStiffness(Eigen::VectorXd &motors_stiffness){
    
    if (this->motors_stiffness.size() != 0) {
    
        motors_stiffness = this->motors_stiffness;
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

bool ROSEE::EEHal::getTipJointToTipFrameX(Eigen::VectorXd &tip_joint_to_tip_frame_x) {
   
    if (this->tip_joint_to_tip_frame_x.size() != 0) {

        tip_joint_to_tip_frame_x = this->tip_joint_to_tip_frame_x;
        return true;

    } else {
        return false;
    }  
}

bool ROSEE::EEHal::getTipJointToTipFrameY(Eigen::VectorXd &tip_joint_to_tip_frame_y) {
   
    if (this->tip_joint_to_tip_frame_y.size() != 0) {

        tip_joint_to_tip_frame_y = this->tip_joint_to_tip_frame_y;
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
    
    if(node["hand_info"]["motors_stiffness"]){

       motors_stiffness = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["motors_stiffness"]);
    }
    
    if(node["hand_info"]["tips_frictions"]){

       tips_frictions = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["tips_frictions"]);
    }
    
    if(node["hand_info"]["motors_torque_limits"]){

       motors_torque_limits = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["motors_torque_limits"]);
    }
    
    if(node["hand_info"]["tip_joint_to_tip_frame_x"]){

       tip_joint_to_tip_frame_x = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["tip_joint_to_tip_frame_x"]);
    }
    
    if(node["hand_info"]["tip_joint_to_tip_frame_y"]){

       tip_joint_to_tip_frame_y = ROSEE::Utils::yamlVectorToEigen(node["hand_info"]["tip_joint_to_tip_frame_y"]);
    }
    
    return true;
}

bool ROSEE::EEHal::init_hand_info_response() {
    
    _hand_info_response.fingers_names = fingers_names;
    
    _hand_info_response.motors_names = motors_names;
        
    _hand_info_response.motors_stiffness =
        ROSEE::Utils::eigenVectorToStdVector(motors_stiffness);
        
    _hand_info_response.tips_frictions =
        ROSEE::Utils::eigenVectorToStdVector(tips_frictions);
        
    _hand_info_response.motors_torque_limits =
        ROSEE::Utils::eigenVectorToStdVector(motors_torque_limits);
    
    return true;
}

bool ROSEE::EEHal::setHandInfoCallback() {
    
    _hand_info_server = _nh->advertiseService(_hand_info_service_name, 
        &ROSEE::EEHal::handInfoEEHalCallback, this);
    return true;
}

bool ROSEE::EEHal::handInfoEEHalCallback (    
    rosee_msg::HandInfo::Request& request,
    rosee_msg::HandInfo::Response& response) {
   
    //generic hal does not read the request
    
    response = _hand_info_response;
    
    return true;
}

bool ROSEE::EEHal::initPressureSensing()
{
    
    std::string topic_name = "/ros_end_effector/pressure_phalanges";
    
    _pressure_pub = _nh->advertise<rosee_msg::MotorPhalangePressure>(topic_name, 10);

    _pressure_active = true;
    
    return true;
}

bool ROSEE::EEHal::publish_pressure() {
    
    //NOTE _pressure_msg must be filled by the derived class
    _pressure_pub.publish(_pressure_msg);
    
    return true;
    
}

Eigen::VectorXd ROSEE::EEHal::getMotorReference() const {
    
    Eigen::VectorXd motorRef;
    motorRef.resize(_mr_msg.name.size());
    for (int i=0; i<_mr_msg.name.size(); i++ ) {
        
        motorRef(i) = _mr_msg.position.at(i);
    }
    
    return motorRef;
}

Eigen::VectorXd ROSEE::EEHal::getJointPosition() const {

    Eigen::VectorXd jointPos;
    jointPos.resize(_js_msg.name.size());
    for (int i=0; i<_js_msg.name.size(); i++ ) {
        
        jointPos(i) = _js_msg.position.at(i);
    }
    
    return jointPos;
}

Eigen::VectorXd ROSEE::EEHal::getJointEffort() const {

    Eigen::VectorXd jointEffort;
    jointEffort.resize(_js_msg.name.size());
    for (int i=0; i<_js_msg.name.size(); i++ ) {
        
        jointEffort(i) = _js_msg.effort.at(i);
    }
    
    return jointEffort;
}

Eigen::MatrixXd ROSEE::EEHal::getPressure() const {

    Eigen::MatrixXd pressure;
    pressure.resize(4, _pressure_msg.pressure_finger1.size()); //message has 4 finger field
    for (int i=0; i<_pressure_msg.pressure_finger1.size(); i++ ) {

        pressure(0, i) = _pressure_msg.pressure_finger1.at(i);
        pressure(1, i) = _pressure_msg.pressure_finger2.at(i);
        pressure(2, i) = _pressure_msg.pressure_finger3.at(i);
        pressure(3, i) = _pressure_msg.pressure_thumb.at(i);
    }
    
    return pressure;
    
}
