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

#ifndef __ROSEE_ACTIONCOMPOSED_H
#define __ROSEE_ACTIONCOMPOSED_H

#include <vector>
#include <string>
#include <ROSEndEffector/Action.h>
#include <yaml-cpp/yaml.h>
#include <memory>

/** 
 * 
 * TODO instead of use it as "Composed", use it as single action, that is a different structure respect to
 * action primitive, with a single joint state. Then can contain a single action, or more.
 * 
 * TODO una remove? non è così facile da implementare e non so se sia utile...
 */
namespace ROSEE{

/**
 * If the action has @independent primitives, each joint position is set by ONLY ONE of the primitives inside, or nothing
 * (set at a default state, i.e. not used)
 * If the action has not @independent primitives, each joint position is calculated as the mean among all the joint 
 * position of the contained primitives that uses that joint. So each mean can include different primitives, so we need
 * a @involvedJointsCount vector
 */
class ActionComposed : public Action
{
    
public: 
    ActionComposed();
    ActionComposed(std::string name);
    ActionComposed(std::string name, bool independent);
    // Copy constructor 
    ActionComposed (const ActionComposed &other);
    
    JointPos getJointPos () const override;
    
    /* getters and setters */
    unsigned int numberOfInnerActions () const;
    bool isIndependent () const;
    std::vector <std::string> getInnerActionsNames() const ;
    
    void print () const override; 
    void emitYaml ( YAML::Emitter&) const override;
    bool fillFromYaml(  YAML::const_iterator yamlIt ) override;
    
    /** 
     * @brief function to add another primitive to the composed action. 
     * @param std::shared_ptr <ROSEE::ActionPrimitive> primitive
     * @param int as : the index of the actionState that we want to insert in the composed action. 
     *  the default is the best one. This is due to the fact that a primitive can have different actionState inside 
     *  (e.g. for the pinchStrong among two tips we can choose among different Jointstates configuration to make the two
     *  tips collide)
     */
    bool sumAction ( ROSEE::Action::Ptr );
    
private:
    std::vector < std::string > innerActionsNames;
    unsigned int nInnerActions;
    
    JointPos jointPos;
    
    bool independent; //true if each primitive must set different joint states
    
};
}

#endif // __ROSEE_ACTIONCOMPOSED_H
