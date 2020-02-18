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

#ifndef ROSEE_ACTIONTIMED_H
#define ROSEE_ACTIONTIMED_H

#include <vector>
#include <string>
#include <map>
#include <ROSEndEffector/Action.h>
#include <yaml-cpp/yaml.h>

namespace ROSEE {

/**
 * @todo write docs
 */
class ActionTimed : public Action
{
public:
    /**
     * Default constructor
     */
    ActionTimed();
    ActionTimed(std::string actionName);

    /**
     * Destructor
     */
    ~ActionTimed() {};
    
    JointPos getJointPos () const override;
        
    ROSEE::JointPos getActionJointPos (std::string actionName) const ;
    
    std::pair <double, double> getActionMargins (std::string actionName ) const ;


    /**
     * @brief Print info about this action
     */
    void print () const override;
    
    /**
     * @brief Emit info in a file with yaml format
     * @param out a YAML::Emitter& object to emit the infos
     */    
    void emitYaml ( YAML::Emitter& out) const override;
    
    /**
     * @brief Fill the internal data with infos taken from yaml file. 
     * @param yamlIt a yamlt iterator to a node which has loaded the file
     * @return false if some error happened
     */
    bool fillFromYaml(  YAML::const_iterator yamlIt ) override;
    
    bool insertAction ( ROSEE::Action::Ptr action, std::string newActionName = "" ) ;
    bool insertAction ( ROSEE::Action::Ptr action, double marginBefore, double marginAfter, std::string newActionName = "");
    
private:
    std::map <std::string, std::pair<double, double> > actionsTimeMarginsMap;
    std::map <std::string, ROSEE::JointPos> actionsJointPosMap;
    std::vector < std::string > actionsNamesOrdered;

};

}

#endif // ROSEE_ACTIONTIMED_H
