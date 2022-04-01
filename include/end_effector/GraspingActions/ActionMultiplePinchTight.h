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

#ifndef __ROSEE_ACTIONMULTIPLEPINCHTIGHT_H
#define __ROSEE_ACTIONMULTIPLEPINCHTIGHT_H

#include <end_effector/GraspingActions/ActionPinchGeneric.h>
#include <moveit/planning_scene/planning_scene.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {

/**
 * @brief Class to describe the action of "pinching" with more than 2 tips (with 2 tips there is the \ref ActionPinchTight and 
 *   \ref ActionPinchLoose
 *   The number of the finger used is fixed when the object is costructed, and it is stored in the father member \ref nFingersInvolved
 * 
 * A pinchMultipleTight is defined by:
 *  - X tips ( that are inside \ref fingersInvolved member of \ref Action ), so \ref nFingersInvolved == X ( members of base class \ref ActionPrimitive )
 *  - JointStates position: where the collision among the tips happens (inside \ref actionStates )
 *  - Optional info (inside \ref actionStates ): the sum of the depth of compenetration of all the tip pairs that collide.
 * 
 * @note When exploring the model to find this action, there are two possibility: we can look for a position where all the X fingertips
 *   collide among each other (causing EXACTLY bynomial_coeff(X, 2) collisions pair), or, with a "less strict" condition, a position where
 *   not all fingertips must collide between each other (causing AT LEAST X-1 collisions). 
 *   The default is the "more strict" condition, because it can find "better looking" mulPinches, even if obviously we find less possible 
 *   way to perform the action. With this, it seems better to order the actionState considering as "best" the position where the tips collide less,
 *   i.e. where the depthSum is lower. This is the opposite from the normal tight pinch.
 *   This cause anyway a collision, but not a ugly one where the tips compenetrate too much. This thing can change if we invert the sign in the 
 *   \ref depthComp comparison
 */
class ActionMultiplePinchTight : public ActionPinchGeneric
{
    
public:
    
    typedef std::map < std::set<std::string>, ActionMultiplePinchTight > Map;
    
    /** @brief A pair to "link" the JointPos with the depthSum info to order the StateWithDepth in the actionState set*/
    typedef std::pair <JointPos, double> StateWithDepth; 
    
    ActionMultiplePinchTight();
    ActionMultiplePinchTight(unsigned int maxStoredActionStates);
    ActionMultiplePinchTight (std::set <std::string>, JointPos, double depthSum );
    
    JointPos getJointPos () const override;
    JointPos getJointPos (unsigned int index) const;
    
    std::vector < ROSEE::JointPos > getAllJointPos () const override;
    
    /** 
     * @brief Specific get for the ActionMultiplePinchTight to return the state with the paired depthSum
     * @return The vector (of size \ref maxStoredActionStates) containing all the StateWithDepth objects
     */
    std::vector < ROSEE::ActionMultiplePinchTight::StateWithDepth > getActionStates() const;
    
    /** 
     * @brief function to insert a single action in the \ref actionStates set of possible action. 
     *   If the action is not so good (depthSum) the action is not inserted and 
     *   the function return false 
     * @param JointPos The joints position
     * @param depthSum the sum of all depth of collisions pairs
     * @return TRUE if the action is good and is inserted in the set \ref actionStates
     *         FALSE if the action given as argument was not good as the others in the set \ref actionStates
     *           and the set was already full (\ref maxStoredActionStates)
     */
    bool insertActionState (JointPos, double depthSum);

    void print () const override;
    void emitYaml ( YAML::Emitter& ) const override;
    bool fillFromYaml( YAML::const_iterator yamlIt ) override;
    
private:

    /** 
     * @brief struct to put in order the actionStates.
     *   with "<" we put as best the position that has less sumDepth.  
     */
    struct depthComp {
        bool operator() (const StateWithDepth& a, const StateWithDepth& b) const
        {return (std::abs(a.second) < std::abs(b.second) );}
    };
    
    /** 
     * @brief For each multiple pinch possible, we want a set of action because we want to store (in general) more possible way
     * to do that action with that X fingers. The pinch among X tips can theoretically be done in infinite ways, so we store 
     * the best ways found (ordering them with depthSum of fingertips compenetration)
     */
    std::set < StateWithDepth, depthComp > actionStates;
    
};

}

#endif // __ROSEE_ACTIONMULTIPLEPINCHTIGHT_H
