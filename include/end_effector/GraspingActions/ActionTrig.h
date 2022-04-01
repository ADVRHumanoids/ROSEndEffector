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

#ifndef __ROSEE_ACTIONTRIG_H
#define __ROSEE_ACTIONTRIG_H

#include <end_effector/GraspingActions/ActionPrimitive.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {

/**
 * @brief The action of moving some joints (see later) of a single finger in a full clousure position 
 * towards the palm. 
 * The action is unique (joints involved in a certain position: the bound) so \ref maxStoredActionStates == 1 always
 * Described by:
 *  - a tip (that is inside \ref fingersInvolved ): the tip of the finger that is involved in the action. So \ref nFingersInvolved == 1
 *  - JointStates position: which set the joints of the finger to a bound to make the finger closes, 
 *    and all the other non-involved joints to zero
 *  - Optional info \ref type (even if is a member of the base class, here is particular). The Trig, TipFlex
 *    and FingFlex have indentical structure, so ther is no necessity to create different classes, but necessity of only
 *    discriminate the objects using the \ref Type.
 * 
 * Actually, there are 3 types of action for this class
 *  - Trig: The action of fully closing a finger towards the parlm 
 *      (i.e. all joints of a finger set to respective bounds)
 *  - TipFlex: The action of fully closing the last part of finger, maintaining the proximal phalanges still
 *      (i.e. the last actuated joint of a finger set to its bound)
 *  - FingFlex: The action of fully closing the first part of a finger, maintaing the distal phalanges still, 
 *      like moving a human finger maintaining it right
 *      (i.e. the first actuated joint of a finger set to its bound)
 * For each tip:
 *   note that having a Trig does not mean that we have also the tip and the fing flex, because for them at least
 *   2 actuated joint (not continuos) in the finger must exist.
 *   If exist a TipFlex, also a FingFlex exist, and viceversa. If they exist, also a trig exist
 *   The "sum" of TipFlex and "FingFlex" is equal to the Trig only if the number of actuated not continuos joint
 *   in the finger is 2
 * 
 * @todo instead of tip , use the finger name for the Trig (i.e. the defined srdf group).
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

public:
    
    typedef std::map < std::string, ActionTrig > Map;

    ActionTrig (std::string actionName, ActionPrimitive::Type);
    ActionTrig (std::string actionName, ActionPrimitive::Type, std::string tip, JointPos);
    
    /** 
     * @brief Overriden get from the pure virtual function of the base class \ref ActionPrimitive 
     * The signature must be equal, even if here we have set and vector of only one element. For this class
     * this function simply return a vector which contain a single element. 
     */
    std::vector < JointPos > getAllJointPos() const override;
    
    /**
     * @brief Necessary method to know the key used by the maps which store all the Actions of one type. Used by \ref YamlWorker
     * @return for this class, it return the finger name, inserted in a single-element set because father signature say so
     */
    std::set < std::string> getKeyElements () const override;

    
    /** 
     * @brief Overriden get from the pure virtual function of the base class \ref Action 
     */
    JointPos getJointPos () const override;
    void setJointPos (JointPos);

    /** 
     * @brief Specific method of trig to simply return a string instead of the full vector \ref fingersInvolved that in this case 
     * contains only one element
     */
    std::string getFingerInvolved () const;
    void setFingerInvolved ( std::string );

    // we are ok with the default functions of the base class ActionPrimitive
    //void printAction () const override;
    //void emitYaml ( YAML::Emitter&) override;
    bool fillFromYaml( YAML::const_iterator yamlIt ) override;
    
private:
    JointPos jointPos;
};

}

#endif // __ROSEE_ACTIONTRIG_H
