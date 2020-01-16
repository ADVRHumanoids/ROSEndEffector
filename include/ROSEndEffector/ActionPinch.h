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

#ifndef ROSEE_ACTIONPINCH_H
#define ROSEE_ACTIONPINCH_H

#include <ROSEndEffector/ActionPrimitive.h>
#include <moveit/planning_scene/planning_scene.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {

/**
 * @todo write docs
 */
class ActionPinch : public ActionPrimitive 
{
private:
    
    typedef std::pair <JointStates, collision_detection::Contact> StateWithContact; 
    struct depthComp {
        bool operator() (const StateWithContact& a, const StateWithContact& b) const
        {return (std::abs(a.second.depth) > std::abs(b.second.depth) );}
    };
    std::set < StateWithContact, depthComp > statesInfoSet;
    bool emitYamlForContact ( collision_detection::Contact, YAML::Emitter& );

    
public:
    
    ActionPinch();
    ActionPinch(unsigned int);
    ActionPinch (std::pair <std::string, std::string>, JointStates, collision_detection::Contact );

    ~ActionPinch();
    
    std::set < std::string > getLinksInvolved() const override;
    std::vector < ROSEE::JointStates > getActionStates() const override;
    bool setLinksInvolved (std::set < std::string >) override;
    bool setActionStates (std::vector < ROSEE::JointStates > ) override;
    bool insertActionState (JointStates, collision_detection::Contact);

    std::ostream& printAction (std::ostream &output) const override;    
    std::string emitYaml ( ) override;
    bool fillFromYaml( YAML::const_iterator yamlIt ) override;

    std::pair <std::string, std::string > tipsPair ;

};

}

#endif // ROSEE_ACTIONPINCH_H
