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
#include <end_effector/GraspingActions/ActionGeneric.h>
#include <end_effector/Utils.h>
#include <yaml-cpp/yaml.h>
#include <memory>


namespace ROSEE{

/**
 * @brief A ActionComposed, which is formed by one or more Primitives (or even other composed).
 * It is useful for example to create an action that grasp only with bending the tips (e.g. to take a dish from above) 
 * If the ActionComposed has the boolean value \ref independent to true, it means that include indipendent sub-actions, 
 * so, each joint is used by at maximum by ONLY ONE of the sub-action inside.
 * In this case the \ref jointsInvolvedCount will contain only 0 or 1 values.
 * If the ActionComposed is not \ref independent, each joint position is calculated as the mean of all the joint 
 * position of the contained sub-actions that uses that joint. So each mean can include different primitives, so we used the
 * \ref jointsInvolvedCount vector to store the number of sub action that use each joint.
 * 
 * @todo A removeAction function? difficult to implement, and useless?
 */
class ActionComposed : public ActionGeneric
{
    
public: 
    typedef std::shared_ptr<ActionComposed> Ptr;
    typedef std::shared_ptr<const ActionComposed> ConstPtr;
    
    ActionComposed();
    ActionComposed(std::string name);
    ActionComposed(std::string name, bool independent);
    /** @brief Copy costructor 
     */
    ActionComposed (const ActionComposed &other);

    
    /**
     * @brief
     * @return unsigned int the number of the actions that compose this one
     */
    unsigned int numberOfInnerActions () const;
    
    /**
     * @brief
     * @return bool true if this action must contain only independent primitives
     */
    bool isIndependent () const;
    
    /**
     * @brief
     * @return std::vector <std::string> all the names that compose this action
     */
    std::vector <std::string> getInnerActionsNames() const ;
    
    /**
     * @brief Print info about this action (name, jointpos, inner actions names, and other)
     */
    virtual void print () const override;
    
    /**
     * @brief Emit info in a file with yaml format
     * @param out a YAML::Emitter& object to emit the infos
     */    
    virtual void emitYaml ( YAML::Emitter& out) const override;
    
    /**
     * @brief Fill the internal data with infos taken from yaml file. 
     * @param yamlIt a yamlt iterator to a node which has loaded the file
     * @return false if some error happened
     */
    virtual bool fillFromYaml(  YAML::const_iterator yamlIt ) override;
    
    /** 
     * @brief Function to add another action to this one. 
     * @param action The action to be added to the ActionComposed
     * @param jointPosScaleFactor How much scale the joint position values for the action to be inserted. Default to 1
     * @param jointPosIndex (default == 0) the wanted jointPos or \p action to insert. Error the index is greater than the number
     *      of joint pos in the \p action. First element has index 0. 
     * @return False if the ActionComposed is \ref independent and we try to add an action that is dependent from one of the already present
     */
    virtual bool sumAction ( ROSEE::Action::Ptr action , double jointPosScaleFactor = 1.0, unsigned int jointPosIndex = 0 );
    
    /**
     * @brief Check if the action composed is empty
     * 
     * @return true if empty, false otherwise
     */
    bool empty();
    
protected:
    std::vector < std::string > innerActionsNames;
    unsigned int nInnerActions;
        
    bool independent; //true if each primitive must set different joint states

    bool checkIndependency ( ROSEE::Action::Ptr action );

    
};
}

#endif // __ROSEE_ACTIONCOMPOSED_H
