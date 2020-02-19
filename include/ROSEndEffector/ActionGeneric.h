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

#ifndef ROSEE_ACTIONGENERIC_H
#define ROSEE_ACTIONGENERIC_H

#include <ROSEndEffector/Action.h>
#include <yaml-cpp/yaml.h>


namespace ROSEE {

/**
 * @todo write docs
 */
class ActionGeneric :  public Action {
    
public:
    /**
     * @brief default costructor. It is used when parsing a file with YamlWorker
     */
    ActionGeneric() ;
    ActionGeneric (std::string actionName, ROSEE::JointPos jointPos); 
    ActionGeneric (std::string actionName, ROSEE::JointPos jointPos, JointsInvolvedCount jic); 
    ActionGeneric (std::string actionName, ROSEE::JointPos jointPos, JointsInvolvedCount jic, std::set <std::string> fingersInvolved); 
    
    /**
     * @brief Get the joint position related to this action, overriden from \ref Action
     * @return JointsPos the map indicating the position of the joints
     */
    JointPos getJointPos () const override;
    
    /**
     * @brief Get the joint position related to this action, overriden from \ref Action
     * Necessary to not make this class abstact, even if for a composed action we store only a single JointPos, so this
     * function return a single element vector
     * @return JointsPos the map indicating the position of the joints
     */
    std::vector < ROSEE::JointPos > getAllJointPos () const override;
    
    /**

     */
    virtual void emitYaml ( YAML::Emitter& out ) const override;
    
    /**

     */
    virtual bool fillFromYaml ( YAML::const_iterator yamlIt ) override;
    
protected:
    
    /**
     * @brief this costructor is only for derived class
     */
    ActionGeneric(std::string actionName) ;
    
    JointPos jointPos;

    
};

}

#endif // ROSEE_ACTIONGENERIC_H
