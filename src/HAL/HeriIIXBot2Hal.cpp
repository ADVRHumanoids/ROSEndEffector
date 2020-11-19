/*
 * Copyright 2020 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros_end_effector/HAL/HeriIIXBotHal.h>

ROSEE::HeriIIXBotHal::HeriIIXBotHal ( ros::NodeHandle* nh) : EEHal ( nh ) {
    
    std::string path_to_config_file = XBot::Utils::getXBotConfig();
        
    _robot = XBot::RobotInterface::getRobot(path_to_config_file);
    
    _robot2Rt = std::static_pointer_cast<XBot::RobotInterfaceXBot2Rt>(_robot);
    
    //actuator is the type written in the yaml file
    _motors = dynamic_cast<XBot::Hal::HeriIIMotorClientContainer*>(  _robot2Rt->getDeviceContainer("actuator"));
    
    if (! _motors) {
        ROS_ERROR_STREAM("[HeriIIXBotHal::" << __func__ << "] failed cast to heri II client container");
        return;
    }
    
    _mr_msg.name.resize(4);
    _mr_msg.position.resize(4);
    _mr_msg.velocity.resize(4);
    _mr_msg.effort.resize(4);    
    _js_msg.name.resize(4);
    _js_msg.position.resize(4);
    _js_msg.velocity.resize(4);
    _js_msg.effort.resize(4);
    
    _js_msg.name[0] = "motor_finger1";
    _js_msg.name[1] = "motor_finger2";
    _js_msg.name[2] = "motor_finger3";
    _js_msg.name[3] = "motor_thumb";
        
}

bool ROSEE::HeriIIXBotHal::move() {
    
    for (int i=0; i<_mr_msg.position.size(); i++) {

        //TODO hardcoded conversion: in the msg we know there are the urdf actuator names, and we know their conversion to xxx-x
        if (_mr_msg.name[i].compare("motor_finger1") == 0) {
            _motors->get_device("112-1")->setMotorPositionReference(_mr_msg.position[i]);
            
        } else if (_mr_msg.name[i].compare("motor_finger2") == 0) {
            
            _motors->get_device("112-2")->setMotorPositionReference(_mr_msg.position[i]);
            
        } else if (_mr_msg.name[i].compare("motor_finger3") == 0) {
            
            _motors->get_device("113-1")->setMotorPositionReference(_mr_msg.position[i]);
            
        } else if (_mr_msg.name[i].compare("motor_thumb") == 0) {
            
            _motors->get_device("113-2")->setMotorPositionReference(_mr_msg.position[i]);
            
        } else {
            
            ROS_ERROR_STREAM("[HeriIIXBotHal::" << __func__ << "] motor name " << _mr_msg.name[i] << "in the ros message does not correspond to an actuator I know");
            return false;    
        }
    }
    
    //TODO send also the current here?
    
    
    return _motors->move_all();
    
}

bool ROSEE::HeriIIXBotHal::sense() {
    
    if (! _motors->sense_all()) {
        return false;
    }

   _js_msg.position[0] = _motors->get_device("112-1")->getMotorPosition();
   _js_msg.position[1] = _motors->get_device("112-2")->getMotorPosition();
   _js_msg.position[2] = _motors->get_device("113-1")->getMotorPosition();
   _js_msg.position[3] = _motors->get_device("113-2")->getMotorPosition();
   
   return true;

}

