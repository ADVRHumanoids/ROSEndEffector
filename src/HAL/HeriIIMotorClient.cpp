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

#include <ros_end_effector/HAL/HeriIIMotorClient.h>

using namespace XBot::Hal;

bool HeriIIMotorClient::setMotorPositionReference(double motorPositionReference){
    
    _tx.motor_position_reference = motorPositionReference;
}

bool HeriIIMotorClient::setMotorCurrentReference(double motorCurrentReference){
    
    _tx.motor_current_reference = motorCurrentReference; 
}

// double HeriIIMotorClient::getMotorPositionReference() {
//     
//     return _rx.motorPositionReference;    
// }
// 
// double HeriIIMotorClient::getMotorCurrentReference() {
//     
//     return _rx.motorCurrentReference;
// }

double HeriIIMotorClient::getMotorPosition() {
    
    return _rx.motor_position_actual;
}

double HeriIIMotorClient::getMotorCurrent() {
    
    return _rx.motor_current_actual;
}

XBOT2_REGISTER_DEVICE(HeriIIMotorClientContainer, HeriIIHand)
