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

#ifndef XBOT_HAL_HERIIIMOTORCLIENT_H
#define XBOT_HAL_HERIIIMOTORCLIENT_H

#include <xbot2/hal/device.h>
#include "HeriIIMotorPacket.h"

namespace XBot {
namespace Hal {

/**
 * @todo write docs
 */
class HeriIIMotorClient : public DeviceClientTpl<HeriIIMotorPacket::Rx, HeriIIMotorPacket::Tx>,
                          public virtual DeviceBase
{
    
public:
    
    using DeviceClientTpl::DeviceClientTpl;

    
    bool setMotorPositionReference(double motorPositionReference);
    bool setMotorCurrentReference(double motorCurrentReference);
    
    //double getMotorPositionReference();
    //double getMotorCurrentReference();
    double getMotorPosition();
    double getMotorCurrent();
    
private:
    
    unsigned short int finger_id;
    unsigned short int motor_in_finger_id;
    
    
};

class HeriIIMotorClientContainer : public DeviceContainer<HeriIIMotorClient>
{

public:

    using DeviceContainer::DeviceContainer;

};



}
}

#endif // XBOT_HAL_HERIIIMOTORCLIENT_H
