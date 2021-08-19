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

#include <ros_end_effector/GraspingActions/Action.h>
#include <yaml-cpp/yaml.h>
#include <ros_end_effector/Utils.h>


namespace ROSEE {

/**
 * @brief Class to handle a generic, simple action. Differently from other class, this is easily creable manually,
 * for ex giving a name and a \ref JointPos map to the costructor. It contains essential infos about the action,
 * and override the necessary pure virtual functions of base class \ref Action
 */
class ActionGeneric :  public Action {
    
public:
    typedef std::shared_ptr<ActionGeneric> Ptr;
    typedef std::shared_ptr<const ActionGeneric> ConstPtr;
    
    /**
     * @brief default costructor. It is used when parsing a file with YamlWorker,
     * @warning If you use this costructor, then you must fill internal structures with \ref fillFromYaml 
     *      (or using \ref YamlWorker support class), it is the only way to set the internal infos (e.g. there is not a
     *      setName( name ) function ). If you have not a Yaml file to parse, use other costructors.
     */
    ActionGeneric() ;
    /**
     * @brief Simpliest costructor, need only essential infos
     * @note This costructor create \ref jointsInvolvedCount map. To do this, it considers a "not set" joint (count == 0) 
     *      a joint which pos is 0. If you do not want this, pass also your \ref JointsInvolvedCount map to the costructor
     */
    ActionGeneric (std::string actionName, ROSEE::JointPos jointPos); 
    
    /**
     * @brief Another costructor
     * @param actionName the name of this action
     * @param jointPos map containing position of ALL joints of your robot
     * @param jic map of counters of times of joints involved (e.g. joint not used --> 0 ; joint used for the action --> 1) 
     * 
     * @throw Be sure that keys of jointPos and jic are the same, otherwise exception is throw
     */
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
