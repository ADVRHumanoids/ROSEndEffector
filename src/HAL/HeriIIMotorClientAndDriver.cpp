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
#include <ros_end_effector/HAL/HeriIIMotorDriver.h>

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




HeriIIMotorDriver::HeriIIMotorDriver(Hal::DeviceInfo devinfo):
    DeviceDriverTpl(devinfo)
{
    //TODO fill finger id from deviceinfo??

}

bool HeriIIMotorDriver::move_impl()
{
    //TODO check motor limits 

    return true;
}

bool HeriIIMotorDriver::sense_impl()
{
    //nothing?
    return true;
}

HeriIIMotorDriverContainer::HeriIIMotorDriverContainer(std::vector<Hal::DeviceInfo> devinfo):
    DeviceContainer(devinfo),
    _srv_alive(false)
{
    std::vector<DeviceRt::Ptr> devs;

    std::copy(get_device_vector().begin(),
              get_device_vector().end(),
              std::back_inserter(devs));

    _cli = std::make_unique<ClientManager>(
        devs,
        "shm",
        "HeriIIHand"
        );

}

bool HeriIIMotorDriverContainer::sense_all()
{
    if(!_srv_alive)
    {
        _srv_alive = _cli->check_server_alive();
        return false;
    }

    bool ret = _cli->recv();

    return ret && DeviceContainer::sense_all();
}

bool HeriIIMotorDriverContainer::move_all()
{
    DeviceContainer::move_all();

    _cli->send();

    return true;
}

XBOT2_REGISTER_DEVICE(HeriIIMotorDriverContainer, HeriIIMotorClientContainer, HeriIIHand)
