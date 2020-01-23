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

#ifndef ROSEE_ACTIONPINCHWEAK_H
#define ROSEE_ACTIONPINCHWEAK_H

#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionPinchGeneric.h>
#include <moveit/planning_scene/planning_scene.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {

/**
 */
class ActionPinchWeak : public ActionPinchGeneric
{
    
public:
    
    /* A pair to "link" the jointStates with infos about the collision among the two tips*/
    typedef std::pair <JointStates, double> StateWithDistance; 
    
    ActionPinchWeak();
    ActionPinchWeak(unsigned int);
    ActionPinchWeak(std::string, std::string);
    ActionPinchWeak (std::pair <std::string, std::string>, JointStates, double distance );
    
    /** Overriden set and get from the pure virtual functions of the base class @ActionPrimitive */
    std::vector < ROSEE::JointStates > getActionStates() const override;
    bool setActionStates (std::vector < ROSEE::JointStates > ) override;
    
    std::vector< ROSEE::ActionPinchWeak::StateWithDistance > getActionStatesWithDistance() const;

    
    /** 
     */
    bool insertActionState (JointStates, double distance);

    /* For the pinch, we override these function to print, emit and parse the optional info Contact,
     which is specific of the pinch */
    void printAction () const override;
    void emitYaml ( YAML::Emitter&) override;
    bool fillFromYaml( YAML::const_iterator yamlIt ) override;
    
private:
    
    /** struct to put in order the @statesInfoSet. The first elements are the ones 
     */
    struct distComp {
        bool operator() (const StateWithDistance& a, const StateWithDistance& b) const
        {return (std::abs(a.second) < std::abs(b.second) );}
    };
    
    /** 
     */
    std::set < StateWithDistance, distComp > statesInfoSet;
    
    bool emitYamlForDistance ( double distance, YAML::Emitter& );

};

}

#endif // ROSEE_ACTIONPINCHWEAK_H
