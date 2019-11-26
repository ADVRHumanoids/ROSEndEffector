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

#include <ROSEndEffector/EEInterface.h>

ROSEE::EEInterface::EEInterface ( const std::map< std::string, std::vector< std::string > >& ee_description ) {
    
    // get the ee description
    _ee_description = ee_description;
    
    // save the finger names
    for( const auto& finger_joints_pair : ee_description ) {
        
        std::string finger_name = finger_joints_pair.first;
        _fingers_names.push_back(finger_name);
        
        _joints_num += finger_joints_pair.second.size();
        
        for( const auto& j : finger_joints_pair.second ) {
            
            _actuated_joints.push_back(j);
        }
    }

}

const std::vector< std::string >& ROSEE::EEInterface::getFingers() {
    return _fingers_names;
}

bool ROSEE::EEInterface::isFinger ( std::string finger_name ) {
    return ( _ee_description.count(finger_name) > 0 );
}

void ROSEE::EEInterface::getActuatedJoints ( std::vector< std::string >& actuated_joints ) {
    
    actuated_joints = _actuated_joints;
}


bool ROSEE::EEInterface::getActuatedJointsInFinger ( std::string finger_name, std::vector< std::string >& actuated_joints ) {

    if( !isFinger(finger_name) ) {
        
        return false;
    }
    
    actuated_joints = _ee_description.at(finger_name);
    return true;
}

int ROSEE::EEInterface::getActuatedJointsNum() {
    
    return _joints_num;
}

int ROSEE::EEInterface::getActuatedJointsNumInFinger ( std::string finger_name ) {
    
    if( !isFinger(finger_name) ) {
        
        return -1;
    } 

    return _ee_description.at(finger_name).size();
}



ROSEE::EEInterface::~EEInterface() {

}
