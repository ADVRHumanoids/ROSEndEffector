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

#ifndef __ROSEE_ACTION_H
#define __ROSEE_ACTION_H

#include <vector>
#include <string>
#include <map>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <iostream>

#include <end_effector/Utils.h>

/** 
 * 
 */
namespace ROSEE {
    
/** 
 * @brief The map to describe the position of all actuated joints. The key is the name of the string,
 * the value is a vector of joint positions (because in general a joint can have more DOFs
 */
typedef std::map <std::string, std::vector <double> > JointPos;

/** operator overload for JointPos so it is easier to print */
std::ostream& operator << (std::ostream& output, const JointPos jp) ;

JointPos operator * ( double multiplier,  JointPos jp) ;

JointPos operator * ( JointPos jp,  double multiplier ) ;

JointPos& operator *= ( JointPos& jp, double multiplier ) ;

JointPos operator + ( JointPos jp1, JointPos jp2) ;

JointPos& operator += ( JointPos& jp1, ROSEE::JointPos jp2);


/** 
 * @brief The map to describe, how many times a joint is set by the action. 
 *  An \ref ActionPrimitive and an \ref ActionComposed (indipendent) have as values only 0 or 1.
 *  \ref ActionComposed (not independet) can have values > 1.
 *  This map is also useful to understand if a joint is used or not by the action (0 == not used) so
 *  we can control only the necessary joints.
 */
typedef std::map <std::string, unsigned int> JointsInvolvedCount;

std::ostream& operator << (std::ostream& output, const JointsInvolvedCount jic);


/**
 * @brief The pure virtual class representing an Action. It has members that are in common to all derived class
 */
class Action
{

public:
    typedef std::shared_ptr<Action> Ptr;
    typedef std::shared_ptr<const Action> ConstPtr;
    
    /** 
     * @brief Enum useful to discriminate each  action when, for example, we want to parse a file 
     * @remind if you change this enum, change also the ROSEEControl.msg accordingly
     */
    enum Type {Primitive, Generic, Composed, Timed, None};

    /* destructor of base must be virtual */
    virtual ~Action() {};
    
    /**
     * @brief Get the name of the action
     * @return std::string the name of the action
     */
    std::string getName () const ;
    
    Type getType() const;
    
    /**
     * @brief Get for \ref fingersInvolved
     * @return std::set<std::string> the set containing all the hand's fingers involved in the action
     */
    std::set <std::string> getFingersInvolved () const ;
    
    /**
     * @brief Get for \ref jointsInvolvedCount
     * @return JointsInvolvedCount the map indicating how many times the joint is set by the action
     */
    JointsInvolvedCount getJointsInvolvedCount () const ;
    
    /**
     * @brief Get the position related to this action. Pure Virtual function: the derived class
     * store this info differently so they are in charge of providing the read.
     * @return JointsPos the map indicating how the position of the joint
     */
    //TODO rename getJointsPos
    virtual JointPos getJointPos () const = 0;
    
    /**
     * @brief Return all the joint position stored. If the concrete (derived from \ref Action) has only one joint position info,
     * this function is equal to \ref getJointPos.
     * @return vector containing all the joint pos of the action
     */
    //TODO rename getAllJointsPos
    virtual std::vector < ROSEE::JointPos > getAllJointPos () const = 0;

    /** @brief Overridable functions, if we want to make them more action-specific */
    virtual void print () const ;
    
    /**
     * @brief Function to fill the argument passed with info about the action. Pure virtual because each derived
     * class has different infos and stored differently.
     * check \ref YamlWorker to correctly emit and parse the file
     * @param out the yaml-cpp emitter which store infos about the action
     * @note this function does not print in a file, but simply fill a YAML::Emitter.
     */
    virtual void emitYaml ( YAML::Emitter& out ) const = 0;
    /**
     * @brief function to fill members of the Action with infos taken from yaml files
     * @param yamlIt a YAML::const_iterator to the node that is loaded with YAML::LoadFile(dirPath + filename).
     * check \ref YamlWorker to correctly parse and emit the file
     */
    virtual bool fillFromYaml ( YAML::const_iterator yamlIt ) = 0;
    
protected:
    // Only derived class can create this class
    Action();
    Action(std::string actionName, Action::Type type);
    
    std::string name;
    Action::Type type;
    std::set <std::string> fingersInvolved;
    JointsInvolvedCount jointsInvolvedCount;

};

/**
 * To print the action type enum as the real name (eg primitive) and not as the enum number
 * REmember to add here if new type are implemented
 */
std::ostream& operator <<(std::ostream& out, const ROSEE::Action::Type type);

}

#endif // __ROSEE_ACTION_H
