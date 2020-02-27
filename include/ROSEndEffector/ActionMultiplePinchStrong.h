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

#ifndef __ROSEE_ACTIONMULTIPLEPINCHSTRONG_H
#define __ROSEE_ACTIONMULTIPLEPINCHSTRONG_H

#include <ROSEndEffector/ActionPinchGeneric.h>
#include <moveit/planning_scene/planning_scene.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {

/**
 * @brief TODO CHECK ALL DOC FOR THIS
 */
class ActionMultiplePinchStrong : public ActionPinchGeneric
{
    
public:
    
    /** @brief A pair to "link" the JointPos with infos about the collision among the two tips */
    typedef std::pair <JointPos, double> StateWithDepth; 
    
    ActionMultiplePinchStrong();
    ActionMultiplePinchStrong(unsigned int maxStoredActionStates);
    ActionMultiplePinchStrong (std::set <std::string>, JointPos, double depthSum );
    
    JointPos getJointPos () const override;
    JointPos getJointPos (unsigned int index) const;
    
    std::vector < ROSEE::JointPos > getAllJointPos () const override;
    
    /** 
     * @brief Specific get for the ActionMultiplePinchStrong to return the state with contact info 
     * @return The vector (of size \ref maxStoredActionStates) containing all the StateWithContact objects
     */
    std::vector < ROSEE::ActionMultiplePinchStrong::StateWithDepth > getActionStates() const;
    
    /** 
     * @brief function to insert a single action in the \ref actionStates set of possible action. 
     * If the action is not so good (based on depth now) the action is not inserted and 
     * the function return false 
     * @param JointPos The joints position
     * @param collision_detection::Contact the contact associated with the action
     * @return TRUE if the action is good and is inserted in the setActionStates
     *         FALSE if the action given as param was not good as the others in the setActionStates
     *           and the set was already full (\ref maxStoredActionStates)
     */
    bool insertActionState (JointPos, double depthSum);

    /** For the pinch, we override these function to print, emit and parse the optional info Contact,
     which is specific of the pinch */
    void print () const override;
    void emitYaml ( YAML::Emitter& ) const override;
    bool fillFromYaml( YAML::const_iterator yamlIt ) override;
    
private:

    
    /** 
     * @brief struct to put in order the actionStates. The first elements are the ones 
     * with greater depth
     * @FIX, even if is almost impossible, two different contact with same depth will be considered equal
     * with this definition of depthComp. Theoretically they are equal only if the joint status are equal 
     * (of only joints that act for the collision). In fact, we should have the possibility to have two 
     * contact with the same depth (if joint statuses are different), they will be equally good
     */
    struct depthComp {
        bool operator() (const StateWithDepth& a, const StateWithDepth& b) const
        {return (std::abs(a.second) > std::abs(b.second) );}
    };
    
    /** 
     * @brief For each pair, we want a set of action because we want to store (in general) more possible way
     * to do that action. The pinch among two tips can theoretically be done in infinite ways, we store 
     * the best ways found (ordering them by the depth of fingertips compenetration)
     */
    std::set < StateWithDepth, depthComp > actionStates;
    
};

}

#endif // __ROSEE_ACTIONMULTIPLEPINCHSTRONG_H
