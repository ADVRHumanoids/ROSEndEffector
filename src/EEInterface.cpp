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

#include <end_effector/EEInterface.h>

ROSEE::EEInterface::EEInterface ( const ROSEE::Parser& p  ) {
    
    // get number of joints from parse and resize dynamic structure
    _joints_num = p.getActuatedJointsNumber();
    _upper_limits.resize(_joints_num);
    _lower_limits.resize(_joints_num);
    
    // get the EE name
    _ee_name = p.getEndEffectorName();
    
    // get the ee description
    _ee_description = p.getFingerJointMap();
    
    // get he joints urdf description
    _urdf_joint_map = p.getUrdfJointMap();
    
    int id = 0;
    
    // save the finger names
    for( const auto& finger_joints_pair : _ee_description ) {
        
        // fingers
        std::string finger_name = finger_joints_pair.first;
        _fingers_names.push_back(finger_name);

        for( const auto& j : finger_joints_pair.second ) {
            
            // joint name
            _actuated_joints.push_back(j);
            
            // internal id
            _joints_internal_id_map[j] = id;
            
            // finger internal id map 
            _finger_joints_internal_id_map[finger_name].push_back(id);
            
            // upper limits
            _upper_limits[id] = _urdf_joint_map.at(j)->limits->upper;
            
            // lower limits
            _lower_limits[id] = _urdf_joint_map.at(j)->limits->lower;
            
            // increase internal id
            id++;
        }
    }

}

std::string ROSEE::EEInterface::getName() {
    
    return _ee_name;
}


Eigen::VectorXd ROSEE::EEInterface::getLowerPositionLimits() {

    return _lower_limits;
}

Eigen::VectorXd ROSEE::EEInterface::getUpperPositionLimits() {

    return _upper_limits;
}

bool ROSEE::EEInterface::getInternalIdForJoint ( std::string joint_name, int& internal_id ) {
        
    if ( _joints_internal_id_map.count(joint_name) ) {
        
        internal_id = _joints_internal_id_map.at(joint_name);
        return true;
    }
    
    return false;

}


bool ROSEE::EEInterface::getInternalIdsForFinger ( std::string finger_name, std::vector< int >& internal_ids ) {

    if ( _finger_joints_internal_id_map.count(finger_name) ) {
        
        internal_ids = _finger_joints_internal_id_map.at(finger_name);
        return true;
    }
    
    return false;

}

const std::vector< std::string >& ROSEE::EEInterface::getFingers() {
    
    return _fingers_names;
}

bool ROSEE::EEInterface::isFinger ( std::string finger_name ) {
    
    return ( _ee_description.count(finger_name) > 0 );
}



const std::vector< std::string >&  ROSEE::EEInterface::getActuatedJoints () {
    
    return _actuated_joints;
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

int ROSEE::EEInterface::getFingersNumber() {
    
    return _fingers_names.size();
}


ROSEE::EEInterface::~EEInterface() {

}
