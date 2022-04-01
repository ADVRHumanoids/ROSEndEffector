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

#ifndef __ROSEE_ACTIONPINCHLOOSE_H
#define __ROSEE_ACTIONPINCHLOOSE_H

#include <end_effector/GraspingActions/ActionPrimitive.h>
#include <end_effector/GraspingActions/ActionPinchGeneric.h>
#include <moveit/planning_scene/planning_scene.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {

/**
 * @brief The action of pinch with two tips. The two tips must not collide ever 
 * (otherwise we have a TightPinch). They only need to move towards each other moving the relative joints.
 * This PinchLoose is created because also if the tips do not collide (i.e. there is not a \ref ActionPinchTight)
 * we can have anyway a pinch at least to take object of a certain minimum size.
 * All the non involved fingers are set in the default state.
 * A pinchLoose is defined by:
 *  - 2 tips ( that are inside \ref fingersInvolved ), so \ref nFingersInvolved == 2 ( members of base class \ref Action )
 *  - JointStates position: where the collision happens (inside \ref actionStates )
 *  - Optional info (inside \ref actionStates ): the minimum distance found between the two tips.
 *    The distance is used to order, the actions in the \ref actionStates set (make sense if \ref maxStoredActionStates > 1 ): 
 *    the less the distance is, the more we say the pinchLoose is good
 */
class ActionPinchLoose : public ActionPinchGeneric
{
    
public:
    
    typedef std::map < std::pair<std::string, std::string>, ActionPinchLoose > Map;
    
    /**
     * @brief A pair to "link" the JointPos with the optional info 'distance' 
     */
    typedef std::pair <JointPos, double> StateWithDistance; 
    
    ActionPinchLoose();
    ActionPinchLoose ( unsigned int maxStoredActionStates );
    ActionPinchLoose ( std::string tip1, std::string tip2);
    ActionPinchLoose ( std::pair <std::string, std::string>, JointPos, double distance );
    
    JointPos getJointPos () const override;
    JointPos getJointPos (unsigned int index) const;
    
    std::vector < ROSEE::JointPos > getAllJointPos () const override;    
    
    /** 
     * @brief Specific get for this action to return the state with distance info 
     * @return The vector (of size \ref maxStoredActionStates) containing all the StateWithDistance objects
     */
    std::vector < ROSEE::ActionPinchLoose::StateWithDistance > getActionStates() const;

    
    /** 
     * @brief function to insert a single action in the \ref actionStates set of possible action. 
     * If the action is not so good (based on distance) the action is not inserted and 
     * the function return false 
     * @param JointPos The joints position
     * @param distance the distance from the two tips.
     * @return TRUE if the action is good and is inserted in the setActionStates
     *         FALSE if the action given as param was not good as the others in the \ref actionStates
     *           and the set was already full (\ref maxStoredActionStates )
     */
    bool insertActionState (JointPos, double distance);

    /* For the pinch, we override these function to print, emit and parse the optional info Contact,
     which is specific of the pinch */
    void print () const override;
    void emitYaml ( YAML::Emitter&) const override;
    bool fillFromYaml( YAML::const_iterator yamlIt ) override;
    
private:
    
    /**
     * @brief struct to put in order the \ref actionStates set. The first elements are the ones with lesser distance 
     */
    struct distComp {
        bool operator() (const StateWithDistance& a, const StateWithDistance& b) const
        {return (std::abs(a.second) < std::abs(b.second) );}
    };
        
    /** 
     * @brief For each pair, we want a set of action because we want to store (in general) more possible way
     * to do that action. The PinchLoose among two tips can theoretically be done in infinite ways, we store 
     * the best ways found (ordering them by the distance between fingertips)
     */
    std::set < StateWithDistance, distComp > actionStates;

};

}

#endif // __ROSEE_ACTIONPINCHLOOSE_H
