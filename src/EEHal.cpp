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

#include <ros_end_effector/EEHal.h>

ROSEE::EEHal::EEHal ( ROSEE::EEInterface::Ptr ee_interface ) : _ee_inteface ( ee_interface ) {

    std::vector<std::string> joints = _ee_inteface->getActuatedJoints ();
    for ( auto& j: joints ) {

        _joint_postion[j] = 0;
        _joint_velocity[j] = 0;
        _joint_effort[j] = 0;
    };

}


bool ROSEE::EEHal::setJointPosition ( std::string joint_name, double joint_position ) {

    if ( _joint_postion.count ( joint_name ) ) {

        _joint_postion.at ( joint_name ) = joint_position;
        return true;
    }

    return false;
}

bool ROSEE::EEHal::setJointVelocity ( std::string joint_name, double joint_velocity ) {

    if ( _joint_velocity.count ( joint_name ) ) {

        _joint_velocity.at ( joint_name ) = joint_velocity;
        return true;
    }

    return false;
}

bool ROSEE::EEHal::setJointEffort ( std::string joint_name, double joint_effort ) {

    if ( _joint_effort.count ( joint_name ) ) {

        _joint_effort.at ( joint_name ) = joint_effort;
        return true;
    }

    return false;
}

bool ROSEE::EEHal::getJointPosition ( std::string joint_name, double& joint_position ) {

    if ( _joint_postion.count ( joint_name ) ) {

        joint_position = _joint_postion.at ( joint_name );
        return true;
    }

    return false;
    
}

bool ROSEE::EEHal::getJointVelocity ( std::string joint_name, double& joint_velocity ) {

    if ( _joint_velocity.count ( joint_name ) ) {

        joint_velocity = _joint_velocity.at ( joint_name );
        return true;
    }

    return false;
    
}

bool ROSEE::EEHal::getJointEffort ( std::string joint_name, double& joint_effort ) {

    if ( _joint_effort.count ( joint_name ) ) {

        joint_effort = _joint_effort.at ( joint_name );
        return true;
    }

    return false;

}

bool ROSEE::EEHal::getPressure(std::string sensor_name, double& sensor_value) {
    return false;
}



ROSEE::EEHal::~EEHal() {

}
