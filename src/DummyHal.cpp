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

#include <ROSEndEffector/DummyHal.h>

ROSEE::DummyHal::DummyHal ( ROSEE::EEInterface::Ptr ee_interface ) : EEHal ( ee_interface ) {

}


bool ROSEE::DummyHal::getMotorPosition ( std::string joint_name, double& motor_position ) {

    return getJointPosition(joint_name, motor_position);
}

bool ROSEE::DummyHal::getMotorVelocity ( std::string joint_name, double& motor_velocity ) {

    return getJointVelocity(joint_name, motor_velocity);
}

bool ROSEE::DummyHal::getMotorEffort ( std::string joint_name, double& motor_effort ) {

    return getJointEffort(joint_name, motor_effort);
}


bool ROSEE::DummyHal::setPositionReference ( std::string joint_name, double position_reference ) {

    setJointPosition(joint_name, position_reference);
    return true;
}


bool ROSEE::DummyHal::sense() {
    return true;
}

bool ROSEE::DummyHal::move() {
    return true;
}


ROSEE::DummyHal::~DummyHal() {

}
