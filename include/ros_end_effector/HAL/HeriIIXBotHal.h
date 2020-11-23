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

#ifndef HERIIIXBOTHAL_H
#define HERIIIXBOTHAL_H

#include <xbot2/xbot2.h>
#include <xbot2/robot_interface/robot_interface_xbot_rt.h>

#include <iit_heri_ii_hand_xbot2/HeriIIMotorClient.h>
#include <ros_end_effector/HAL/EEHal.h>

namespace ROSEE {
/**
 * @todo write docs
 */
class HeriIIXBotHal : public ROSEE::EEHal
{
    
public:
    
    typedef std::shared_ptr<HeriIIXBotHal> Ptr;
    typedef std::shared_ptr<const HeriIIXBotHal> ConstPtr;
        
    HeriIIXBotHal( ros::NodeHandle* nh);
    virtual ~HeriIIXBotHal() {};
    
    bool sense() override;
        
    bool move() override;
    
private:
    
    //TODO check if both are necessary
    XBot::RobotInterface::Ptr _robot;
    std::shared_ptr<XBot::RobotInterfaceXBot2Rt> _robot2Rt;
    XBot::Hal::HeriIIMotorClientContainer* _motors;

};

}
#endif // HERIIIXBOTHAL_H
