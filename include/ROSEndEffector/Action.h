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

/** 
 * 
 */
namespace ROSEE {
    
/** 
 * @brief The map to describe the position of all actuated joints. The key is the name of the string,
 * the value is a vector of joint positions (because in general a joint can have more DOFs
 */
typedef std::map <std::string, std::vector <double> > JointPos;

/** operator overload for JointPos so it is easy to print */
std::ostream& operator << (std::ostream& output, const JointPos js) {
    for (const auto &jsEl : js) {
        output << "\t\t"<<jsEl.first << " : "; //joint name
        for(const auto &jValue : jsEl.second){
            output << jValue << ", "; //joint position (vector because can have multiple dof)
        }
        output.seekp (-2, output.cur); //to remove the last comma (and space)
        output << std::endl;       
    }
    return output;
}

typedef std::map <std::string, unsigned int> JointsInvolvedCount;

std::ostream& operator << (std::ostream& output, const JointsInvolvedCount jic) {
    for (const auto &jicEl : jic) {
        output << "\t"<< jicEl.first << " : " << jicEl.second;
        output << std::endl;       
    }
    return output;
}

/**
 * @brief The pure virtual class representing an Action. It has members that are in common to all derived class
 */
class Action
{

public:
    typedef std::shared_ptr<Action> Ptr;
    typedef std::shared_ptr<const Action> ConstPtr;


    /* destructor of base must be virtual */
    virtual ~Action() {};
    
    std::string getName () const ;
    std::set <std::string> getFingersInvolved () const ;
    JointsInvolvedCount getJointsInvolvedCount () const ;
    
    virtual JointPos getJointPos () const = 0;
    
    /* overridable functions, if we want to make them more action-specific*/
    virtual void print () const ;
    virtual void emitYaml ( YAML::Emitter& ) const = 0;
    virtual bool fillFromYaml ( YAML::const_iterator yamlIt ) = 0;
    
protected:
    //Only derived class can create this one
    Action();
    Action(std::string);
    
    std::string name;
    std::set <std::string> fingersInvolved;
    
    JointsInvolvedCount jointsInvolvedCount;

};
}

#endif // __ROSEE_ACTION_H
