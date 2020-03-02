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

#include <ROSEndEffector/Utils.h>

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
std::ostream& operator << (std::ostream& output, const JointPos jp) {
    for (const auto &jsEl : jp) {
        output << "\t\t"<<jsEl.first << " : "; //joint name
        for(const auto &jValue : jsEl.second){
            output << jValue << ", "; //joint position (vector because can have multiple dof)
        }
        output.seekp (-2, output.cur); //to remove the last comma (and space)
        output << std::endl;       
    }
    return output;
}

JointPos operator * (const double multiplier, const JointPos jp) {
    
    JointPos jpNew;
    for (const auto &jsEl : jp) {
        std::vector<double> newPos;
       // std::cout << jsEl.first << std::endl;
        for (const double pos : jsEl.second) {
            //std::cout << "old " << pos << "   new: " << pos*multiplier << std::endl;
            newPos.push_back (pos*multiplier);
        }
        jpNew.insert ( std::make_pair (jsEl.first, newPos) );
    }
    
    return jpNew;
}

JointPos operator * (const JointPos jp, const double multiplier ) {
    return (multiplier * jp );
}

JointPos operator + (const JointPos jp1, const JointPos jp2) {
    
    if ( ! ROSEE::Utils::keys_equal(jp1, jp2) ) {
        throw ROSEE::Utils::DifferentKeysException<ROSEE::JointPos, ROSEE::JointPos>(&jp1, &jp2);
    }
    
    JointPos jpNew;
    for (const auto &jsEl : jp1) {
        if (jsEl.second.size() != jp2.at(jsEl.first).size() ) {
            throw "Dofs not same";
        }

        std::vector<double> newPos;
        for (int i = 0; i < jsEl.second.size(); i++) {
            newPos.push_back (jsEl.second.at(i) +  jp2.at(jsEl.first).at(i));
        }
        jpNew.insert ( std::make_pair (jsEl.first, newPos) );
    }
    
    return jpNew;
}

/** 
 * @brief The map to describe, how many times a joint is set by the action. 
 *  An \ref ActionPrimitive and an \ref ActionComposed (indipendent) have as values only 0 or 1.
 *  \ref ActionComposed (not independet) can have values > 1.
 *  This map is also useful to understand if a joint is used or not by the action (0 == not used) so
 *  we can control only the necessary joints.
 */
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
    
    /** 
     * @brief Enum useful to discriminate each  action when, for example, we want to parse a file 
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
    virtual JointPos getJointPos () const = 0;
    
    /**
     * @brief Return all the joint position stored. If the concrete (derived from \ref Action) has only one joint position info,
     * this function is equal to \ref getJointPos.
     * @return vector containing all the joint pos of the action
     */
    virtual std::vector < ROSEE::JointPos > getAllJointPos () const = 0;

    /** @brief Overridable functions, if we want to make them more action-specific */
    virtual void print () const ;
    
    /**
     * @brief Function to fill the argument passed with info about the action. Pure virtual because each derived
     * class has different infos and stored differently.
     * check \ref YamlParser to correctly emit and parse the file
     * @param out the yaml-cpp emitter which store infos about the action
     * @note this function does not print in a file, but simply fill a YAML::Emitter.
     */
    virtual void emitYaml ( YAML::Emitter& out ) const = 0;
    /**
     * @brief function to fill members of the Action with infos taken from yaml files
     * @param yamlIt a YAML::const_iterator to the node that is loaded with YAML::LoadFile(dirPath + filename).
     * check \ref YamlParser to correctly parse and emit the file
     */
    virtual bool fillFromYaml ( YAML::const_iterator yamlIt ) = 0;
    
protected:
    // Only derived class can create this class
    Action();
    Action(std::string actionName, Action::Type type);
    
    std::string name;
    Type type;
    std::set <std::string> fingersInvolved;
    JointsInvolvedCount jointsInvolvedCount;

};
}

#endif // __ROSEE_ACTION_H
