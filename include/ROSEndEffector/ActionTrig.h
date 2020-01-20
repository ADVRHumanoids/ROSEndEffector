/*
 * Copyright (C) 2020 IIT-HHCM
 * Author: Davide Torielli
 * email:  davide.torielli@iit.it
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

#ifndef ROSEE_ACTIONTRIG_H
#define ROSEE_ACTIONTRIG_H

#include <ROSEndEffector/ActionPrimitive.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {

/**
 * @brief The action of moving a finger in a full clousure position towards the palm. 
 * The action is unique (joints involved in a certain position: the bound) so @jointStateSetMaxSize == 1
 * Described by:
 *  - a tip (@tip ): the tip of the finger that is involved in the tip. So @nLinksInvolved == 1
 *  - JointStates position: which set the joints of the finger to a bound to make the finger closes, 
 *    and all the other non-involved joints to zero
 *  - Optional info not used
 * 
 * @todo instead of @tip , use the finger name (i.e. the defined srdf group) 
 * 
 * @note We have to understand the direction of joints to make the finger full close. Because full close position
 * can be linked to both lower or upper bound of each joint involved. 
 * The method to solve this is to go in the max range of the joint, because usually a finger has more motion
 * towards the palm respect the opposite (like humans). We consider the default joint pos to 0.
 * @warning so, take care of joint limits: they must include the 0. Has it sense to have joint limits both
 * positive or both negative (not including the 0) ?
 */
class ActionTrig : public ActionPrimitive 
{
private:
    
    JointStates jointStates;
    
public:
    
    ActionTrig();
    ActionTrig (std::string, JointStates);
    
    /** Overriden set and get from the pure virtual functions of the base class @ActionPrimitive 
     The signature must be equal, even if here we have set and vector of only one element */
    std::set < std::string > getLinksInvolved() const override;
    std::vector < JointStates > getActionStates() const override;
    bool setLinksInvolved (std::set < std::string >) override;
    bool setActionStates (std::vector < JointStates > ) override;

    JointStates getActionState() const;
    bool setActionState (JointStates);

    // we are ok with the default functions of the base class ActionPrimitive
    //void printAction () const override;
    //void emitYaml ( YAML::Emitter&) override;
    //bool fillFromYaml( YAML::const_iterator yamlIt ) override;

    /** the tip involved in the action. @TODO it should be the finger?*/
    std::string tip;

};

}

#endif // ROSEE_ACTIONTRIG_H
