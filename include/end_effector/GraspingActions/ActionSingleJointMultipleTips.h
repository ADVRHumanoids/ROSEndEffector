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

#ifndef ACTIONSINGLEJOINTMULTIPLETIPS_H
#define ACTIONSINGLEJOINTMULTIPLETIPS_H

#include <end_effector/GraspingActions/ActionPrimitive.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {
    
/**
 * @brief Primitive which indicate a motion of n fingers moving ONLY ONE joint. 
 * For example, this primitive is necessary for the hands that have one joint that close all the fingers to do a grasp. But 
 * it can also useful to detect other multiple finger motions (like a "spread finger")
 */
class ActionSingleJointMultipleTips : public ActionPrimitive {
    
public:
    
    typedef std::map < std::string, ActionSingleJointMultipleTips > Map;
    
    ActionSingleJointMultipleTips();

    /**
     * @brief NOT ANYOMORE ? used (for now) by yaml worker only. Before parsing we cant now the info that the other costructor need. So all these infos
     * are set in the \ref fillFromYaml
     * @param actionName name of the action
     * @param nFingers number of the fingers involved in the action (i.e. number of finger that the \ref jointInvolved moves)
     */
    ActionSingleJointMultipleTips(std::string actionName, unsigned int nFingers);
    /**
     * @brief "Standard" costructor, which fill all the necessary infos
     * @param actionName name of the action
     * @param fingers vector containing all the fingers involved
     * @param jointName the name of the joint involved in the action (one joint for definition)
     * @param jpFurther JointPosition of ALL joints of the hand, with only the \p jointName set, in a way that the position is to the bound
     *        which is further from the default position (which is 0)
     * @param jpNearer JointPosition of ALL joints of the hand, with only the \p jointName set, in a way that the position is to the bound
     *        which is nearer from the default position
     * @warning If 0 position is not included between the bounds (eg 0.4 | 5.1) , jpNearer is the one nearer
     *       to 0 (0.4) and jpFurther the further (5.1). CHECK THIS FACT TO BE SURE
     */
    ActionSingleJointMultipleTips (std::string actionName, std::vector<std::string> fingers, std::string jointName, JointPos jpFurther, JointPos jpNearer);
    
    /** 
     * @brief Overriden get from the pure virtual function of the base class \ref Action
     * The signature must be equal, even if here we have set and vector of only one element. 
     * For this class this function return the vector which contain \ref jointPosFurther and \ref jointPosNearer
     */
    std::vector < JointPos > getAllJointPos() const override;
    
    /** 
     * @brief Overriden get from the pure virtual function of the base class \ref Action 
     * @return the jointPosFurther 
     */
    JointPos getJointPos () const override;
    
    /**
     * @brief Necessary method to know the key used by the maps which store all the Actions of one type. Used by \ref YamlWorker
     * @return for this class, it return the jointName, inserted in a single-element set because father signature say so
     */
    std::set < std::string> getKeyElements () const override;

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

#endif // ACTIONSINGLEJOINTMULTIPLETIPS_H
