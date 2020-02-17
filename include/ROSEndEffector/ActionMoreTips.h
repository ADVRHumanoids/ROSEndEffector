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

#ifndef ACTIONMORETIPS_H
#define ACTIONMORETIPS_H

#include <ROSEndEffector/ActionPrimitive.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {
    
/**
 * @todo write docs
 */
class ActionMoreTips : public ActionPrimitive {
    
public:

    ActionMoreTips (std::string actionName, std::vector<std::string> fingers, std::string jointName, JointPos jpFurther, JointPos jpNearer);
    
    /** 
     * @brief Overriden get from the pure virtual function of the base class \ref ActionPrimitive 
     * The signature must be equal, even if here we have set and vector of only one element. For this class
     * this function simply return a vector which contain a single element. 
     */
    std::vector < JointPos > getAllJointPos() const override;
    
    /** 
    * @brief Overriden get from the pure virtual function of the base class \ref Action 
    * Default return the jointPosFurther 
    */
    JointPos getJointPos () const override;
    
    JointPos getJointPosFurther () const;
    JointPos getJointPosNearer () const;
    std::string getJointName() const;
    
    void print () const override;
    void emitYaml ( YAML::Emitter& out ) const override;
    bool fillFromYaml( YAML::const_iterator yamlIt ) override;
    
private:
    
    JointPos jointPosFurther;
    JointPos jointPosNearer;
    std::string jointInvolved;
};
}

#endif // ACTIONMORETIPS_H
