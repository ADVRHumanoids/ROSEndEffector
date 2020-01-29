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

#ifndef ACTIONPRIMITIVE_H
#define ACTIONPRIMITIVE_H

#include <set>
#include <string>
#include <map>
#include <vector>
#include <iostream>

#include <yaml-cpp/yaml.h>


namespace ROSEE{
/** The map to store the couple jointName (key) --- jointPositions (value), multiple dof in general
 */
typedef std::map <std::string, std::vector <double> > JointStates;

std::ostream& operator << (std::ostream& output, const JointStates js) {
    for (const auto &jsEl : js) {
        output << "\t\t\t"<<jsEl.first << " : "; //joint name
        for(const auto &jValue : jsEl.second){
            output << jValue << ", "; //joint position (vector because can have multiple dof)
        }
        output << std::endl;       
    }
    return output;
}

enum ActionType {Pinch, PinchStrong, PinchWeak, Trig, TipFlex, FingFlex, None};

/**
 * @brief Virtual class, Base of all the primitive action. It has some implemented functions that a 
 * derived class can use, if you don't want to override to make them more action-specific.
 * All the primitives are defined by:
 *  - A set containing a fixed number (@nLinksInvolved ) of links/handparts, which are involved in
 *    the action. (e.g. two fingertips for a pinch)
 *  - JointStates position: hand configuration for which the 100% action happens. This is a member that
 *    all the derived class must have, but it can be store as it is (e.g. trigger) but also in a set 
 *    (if jointStateSetMaxSize > 1) or in a set with some other info linked (e.g. in the pinch we have a
 *    pair JointStates - Contact stored in the set). So it is not a member of this base class
 *  - Optional info about the action (e.g. a moveit Contact class for the pinch). Being optional, 
 *    we don't have a member for this in this base class
 */
class ActionPrimitive
{
protected:
 
    /* Protected costructor: object creable only by derived classes. 
     No default costructo (without arguments) because we want to set always these three member */
    ActionPrimitive( std::string name, unsigned int nLinksInvolved, unsigned int jointStateSetMaxSize,
        ActionType actionType );
    
    const std::string name;
    
    /* e.g. two tips for the pinch*/
    const unsigned int nLinksInvolved;
    
    /* the max number of action for each linksInvolved set that we want to store */
    const unsigned int jointStateSetMaxSize;
    
    const ActionType actionType;
    
    std::vector < bool > jointsInvolved ;
    
public:
    
    /* destructor of base must be virtual */
    virtual ~ActionPrimitive() {};

    /* virtual and not getters */
    std::string getName () const;
    ROSEE::ActionType getActionType() const;
    unsigned int getJointStatesSetMaxSize() const;
    unsigned int getnLinksInvolved() const;
    std::vector < bool > getIfJointsInvolved() const;
    virtual std::set < std::string > getLinksInvolved() const = 0;
    virtual std::vector < ROSEE::JointStates > getActionStates() const = 0;
    
    /* virtual and not setters */
    void setJointsInvolved ( std::vector < bool > );
    virtual bool setLinksInvolved (std::set < std::string >) = 0;
    virtual bool setActionStates (std::vector < ROSEE::JointStates > ) = 0;
    
    /* overridable functions, if we want to make them more action-specific*/
    virtual void printAction () const ;
    std::stringstream streamJointsInvolved ( std::stringstream & output) const;

    virtual void emitYaml ( YAML::Emitter& ) ;
    void emitYamlForJointsInvolved ( YAML::Emitter& );
    virtual bool fillFromYaml( YAML::const_iterator yamlIt );
    void fillYamlJointsInvolved ( YAML::const_iterator yamlIt );

    
};
}

#endif // ACTIONPRIMITIVE_H
