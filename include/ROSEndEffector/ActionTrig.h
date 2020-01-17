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

#ifndef ROSEE_ACTIONTRIG_H
#define ROSEE_ACTIONTRIG_H

#include <ROSEndEffector/ActionPrimitive.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {

/**
 * @todo write docs
 */
class ActionTrig : public ActionPrimitive 
{
private:
    
    JointStates jointStates;
    
public:
    
    ActionTrig();
    ActionTrig (std::string, JointStates);
    
    std::set < std::string > getLinksInvolved() const override;
    std::vector < JointStates > getActionStates() const override;
    bool setLinksInvolved (std::set < std::string >) override;
    bool setActionStates (std::vector < JointStates > ) override;

    JointStates getActionState() const;
    bool setActionState (JointStates);

    //void printAction () const override;
    //void emitYaml ( YAML::Emitter&) override;
    //bool fillFromYaml( YAML::const_iterator yamlIt ) override;

    std::string tip;

};

}

#endif // ROSEE_ACTIONTRIG_H
