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

#ifndef __ROSEE_ACTIONPRIMITIVE_H
#define __ROSEE_ACTIONPRIMITIVE_H

#include <set>
#include <string>
#include <map>
#include <vector>
#include <iostream>

#include <end_effector/GraspingActions/Action.h>

#include <yaml-cpp/yaml.h>


namespace ROSEE{

/**
 * @brief Virtual class, Base of all the primitive actions. It has some implemented functions that a 
 * derived class can use, if you don't want to override to make them more action-specific.
 * All the primitives are defined by:
 *  - A set containing a fixed number (\ref nFingersInvolved ) of links/handparts, which are involved in
 *    the action. (e.g. two fingertips for a pinch)
 *  - JointStates position: hand configuration for which the 100% action happens. This is a member that
 *    all the derived class must have, but it can be store as it is (e.g. trigger) but also in a set 
 *    (if \ref maxStoredActionStates > 1) or in a set with some other info linked (e.g. in the pinch we have a
 *    pair JointPos - Contact stored in the set). So it is not a member of this base class
 *  - Optional info about the action (e.g. a moveit Contact class for the pinch). Being optional, 
 *    we don't have a member for this in this base class
 */
class ActionPrimitive : public Action
{
    
public:
    
    typedef std::shared_ptr<ActionPrimitive> Ptr;
    typedef std::shared_ptr<const ActionPrimitive> ConstPtr;

    /** 
     * @brief Enum useful to discriminate each primitive action when, for example, we want to parse a file 
     * @remind if you change this enum, change also the ROSEEControl.msg accordingly
     */
    enum Type {PinchTight, PinchLoose, MultiplePinchTight, Trig, TipFlex, FingFlex, SingleJointMultipleTips, None};
    /* destructor of base must be virtual */
    virtual ~ActionPrimitive() {};

    /* virtual and not getters */
    Type getPrimitiveType() const;
    unsigned int getMaxStoredActionStates() const;
    unsigned int getnFingersInvolved() const;
    
    /**
     * @brief Depending on the primitive, we can use different "keys" to take info from yaml file when parsing
     * for example, trig and pinches are selected through fingersInvolved, while ActionSingleJointMultipleTips uses the joint name.
     * So each derived class must override this info, which for now is used only in \ref YamlWorker::parseYamlPrimitive() and also by map handler to get the primitive, also on send action test
     */
    virtual std::set < std::string> getKeyElements () const = 0;
    
    void setJointsInvolvedCount (ROSEE::JointsInvolvedCount jointsInvolvedCount ) ;    
    /* overridable functions, if we want to make them more action-specific*/
    virtual void emitYaml ( YAML::Emitter& ) const override;

protected:
    
    ActionPrimitive ( std::string name, unsigned int maxStoredActionStates, Type type );
 
    /**
     * @brief Protected costructor: object creable only by derived classes.
     * There is no default costructor (without arguments) because we want to set always these members
     */
    ActionPrimitive( std::string name, unsigned int nFingersInvolved, unsigned int maxStoredActionStates,
        Type type );
    
    /* e.g. two tips for the pinch*/
    unsigned int nFingersInvolved;
    
    /* the max number of action for each linksInvolved set that we want to store */
    const unsigned int maxStoredActionStates;
    
    const Type primitiveType;
        

    
};

/**
 * To print the action type enum as the real name (eg primitive) and not as the enum number
 * REmember to add here if new type are implemented
 */

//std::ostream& operator<<(std::ostream& out, const ActionPrimitive::Type type);

}

#endif // __ROSEE_ACTIONPRIMITIVE_H
