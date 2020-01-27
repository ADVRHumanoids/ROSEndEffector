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

#ifndef ROSEE_ACTIONPINCHSTRONG_H
#define ROSEE_ACTIONPINCHSTRONG_H

#include <ROSEndEffector/ActionPinchGeneric.h>
#include <moveit/planning_scene/planning_scene.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {

/**
 * @brief The action of pinch with two tips. The two tips must collide for some hand 
 * configuration to mark this configuration as a tip. All the non involved fingers are set in the 
 * default state.
 * A pinch is defined by:
 *  - 2 tips (@tipsPair ), so @nLinksInvolved == 2
 *  - JointStates position: where the collision happens (inside @statesInfoSet)
 *  - Optional info (inside @statesInfoSet ): the contact of moveit. Now only the member depth is used. 
 *    It is used to order, for each pair of tips, the actions in the statesInfoSet 
 *    (make sense if @jointStateSetMaxSize > 1 ): 
 *    the more the depth of compenetration is, the more we say the pinch is good
 * 
 * @todo If no contact between two tips, do more checks removing temporarily the joint bounds and check 
 * collisions again. Then put again the bounds and see the distance between the tips, which will be the 
 * new comparison to say if the action is better than another. 
 * This because a pinch can exist also if no collision really happen.
 * 
 */
class ActionPinchStrong : public ActionPinchGeneric
{
    
public:
    
    /* A pair to "link" the jointStates with infos about the collision among the two tips*/
    typedef std::pair <JointStates, collision_detection::Contact> StateWithContact; 
    
    ActionPinchStrong();
    ActionPinchStrong(unsigned int);
    ActionPinchStrong (std::pair <std::string, std::string>, JointStates, collision_detection::Contact );
    
    /** Overriden set and get from the pure virtual functions of the base class @ActionPrimitive */
    std::vector < ROSEE::JointStates > getActionStates() const override;
    bool setActionStates (std::vector < ROSEE::JointStates > ) override;
    
    std::vector< ROSEE::ActionPinchStrong::StateWithContact > getActionStatesWithContact() const;

    
    /** 
     * @brief function to insert a single action in the setActionStates of possible action. 
     * If the action is not so good (based on depth now) the action is not inserted and 
     * the function return false 
     * @param JointStates The hand configuration
     * @param collision_detection::Contact the contact associated with the action
     * @return TRUE if the action is good and is inserted in the setActionStates
     *         FALSE if the action given as param was not good as the others in the setActionStates
     *           and the set was already full (@jointStateSetMaxSize)
     */
    bool insertActionState (JointStates, collision_detection::Contact);

    /* For the pinch, we override these function to print, emit and parse the optional info Contact,
     which is specific of the pinch */
    void printAction () const override;
    void emitYaml ( YAML::Emitter&) override;
    bool fillFromYaml( YAML::const_iterator yamlIt ) override;
    
private:
    
    /** struct to put in order the @statesInfoSet. The first elements are the ones 
     * with greater depth
     * @FIX, even if is almost impossible, two different contact with same depth will be considered equal
     * with this definition of depthComp. Theoretically they are equal only if the joint status are equal 
     * (of only joints that act for the collision). In fact, we should have the possibility to have two 
     * contact with the same depth (if joint statuses are different), they will be equally good
     * 
     */
    struct depthComp {
        bool operator() (const StateWithContact& a, const StateWithContact& b) const
        {return (std::abs(a.second.depth) > std::abs(b.second.depth) );}
    };
    
    /** For each pair, we want a set of action because we want to store (in general) more possible way
     * to do that action. The pinch among two tips can theoretically be done in infinite ways, we store 
     * the best ways found (ordering them by the depth of fingertips compenetration)
     */
    std::set < StateWithContact, depthComp > statesInfoSet;
    
    bool emitYamlForContact ( collision_detection::Contact, YAML::Emitter& );

};

}

#endif // ROSEE_ACTIONPINCHSTRONG_H
