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

#ifndef ROSEE_ACTIONPINCHWEAK_H
#define ROSEE_ACTIONPINCHWEAK_H

#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionPinchGeneric.h>
#include <moveit/planning_scene/planning_scene.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {

/**
 * @brief The action of pinch with two tips. The two tips must not collide ever 
 * (otherwise we have a StrongPinch). They only need to move towards each other moving the relative joints.
 * This PinchWeak is created because also if the tips do not collide we can have a pinch (that can take an object
 * of a minimum size of @distance ).
 * All the non involved fingers are set in the default state.
 * A pinchWeak is defined by:
 *  - 2 tips (@tipsPair ), so @nLinksInvolved == 2 (members of base class @PinchGeneric )
 *  - JointStates position: where the minimum distance among two tips is found (inside @statesInfoSet )
 *  - Optional info (inside @statesInfoSet ): the minimum distance found between the two tips.
 *    The distance is used to order, for each pair of tips, the actions in the statesInfoSet 
 *    (make sense if @jointStateSetMaxSize > 1 ): 
 *    the less the distance is, the more we say the pinchWeak is good
 * 
 */
class ActionPinchWeak : public ActionPinchGeneric
{
    
public:
    
    /* A pair to "link" the jointStates with the optional info 'distance' */
    typedef std::pair <JointStates, double> StateWithDistance; 
    
    ActionPinchWeak();
    ActionPinchWeak(unsigned int);
    ActionPinchWeak(std::string, std::string);
    ActionPinchWeak (std::pair <std::string, std::string>, JointStates, double distance );
    
    /** Overriden set and get from the pure virtual functions of the base class @ActionPrimitive */
    std::vector < ROSEE::JointStates > getActionStates() const override;
    bool setActionStates (std::vector < ROSEE::JointStates > ) override;
    
    /** Specific get for this action to return the state with distance info */
    std::vector< ROSEE::ActionPinchWeak::StateWithDistance > getActionStatesWithDistance() const;

    
    /** 
     */
    bool insertActionState (JointStates, double distance);

    /* For the pinch, we override these function to print, emit and parse the optional info Contact,
     which is specific of the pinch */
    void printAction () const override;
    void emitYaml ( YAML::Emitter&) override;
    bool fillFromYaml( YAML::const_iterator yamlIt ) override;
    
private:
    
    /** struct to put in order the @statesInfoSet. The first elements are the ones with lesser distance 
     */
    struct distComp {
        bool operator() (const StateWithDistance& a, const StateWithDistance& b) const
        {return (std::abs(a.second) < std::abs(b.second) );}
    };
    
    /** 
     * @brief function to insert a single action in the setActionStates of possible action. 
     * If the action is not so good (based on distance) the action is not inserted and 
     * the function return false 
     * @param JointStates The hand configuration
     * @param collision_detection::Contact the contact associated with the action
     * @return TRUE if the action is good and is inserted in the setActionStates
     *         FALSE if the action given as param was not good as the others in the setActionStates
     *           and the set was already full (@jointStateSetMaxSize )
     */
    std::set < StateWithDistance, distComp > statesInfoSet;
    
    bool emitYamlForDistance ( double distance, YAML::Emitter& );

};

}

#endif // ROSEE_ACTIONPINCHWEAK_H
