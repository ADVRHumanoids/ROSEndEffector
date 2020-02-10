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

#include <ROSEndEffector/ActionPinchWeak.h>

ROSEE::ActionPinchWeak::ActionPinchWeak() : 
    ActionPinchGeneric ("pinchWeak", 2, 3, ActionPrimitive::Type::PinchWeak) { }

ROSEE::ActionPinchWeak::ActionPinchWeak (unsigned int maxStoredActionStates) : 
    ActionPinchGeneric ("pinchWeak", 2, maxStoredActionStates, ActionPrimitive::Type::PinchWeak) { }

ROSEE::ActionPinchWeak::ActionPinchWeak (std::string tip1, std::string tip2) : 
    ActionPinchGeneric ("pinchWeak", 2, 3, ActionPrimitive::Type::PinchWeak) {
        fingersInvolved.insert (tip1);
        fingersInvolved.insert (tip2);
}
    
ROSEE::ActionPinchWeak::ActionPinchWeak (std::pair <std::string, std::string> tipNames, 
    JointPos jp, double distance) :
    ActionPinchGeneric ("pinchWeak", 2, 3, ActionPrimitive::Type::PinchWeak )  {

    fingersInvolved.insert (tipNames.first);
    fingersInvolved.insert (tipNames.second);
    
    //different from insertState, here we are sure the set is empty (we are in costructor)
    actionStates.insert (std::make_pair (jp, distance) );
}

std::vector < ROSEE::ActionPinchWeak::StateWithDistance > ROSEE::ActionPinchWeak::getActionStates() const {
    
    std::vector < ROSEE::ActionPinchWeak::StateWithDistance > retVect;
    retVect.reserve ( actionStates.size() );
    
    for (auto it : actionStates ) {
        retVect.push_back(it);
    }
    
    return retVect;
}

ROSEE::JointPos ROSEE::ActionPinchWeak::getJointPos() const {
    return (actionStates.begin()->first);
}

ROSEE::JointPos ROSEE::ActionPinchWeak::getJointPos(unsigned int index) const {
    auto it = actionStates.begin();
    unsigned int i = 1;
    while (i < index ) {
        ++ it;
        ++ i;
    }
    return (it->first);
}

std::vector < ROSEE::JointPos > ROSEE::ActionPinchWeak::getAllJointPos() const {
    
    std::vector < JointPos > retVect;
    retVect.reserve(actionStates.size());
    
    for (auto it : actionStates ) {
        retVect.push_back(it.first);
    }
    
    return retVect;
}

bool ROSEE::ActionPinchWeak::insertActionState (ROSEE::JointPos jp, double dist) {

    auto pairRet = actionStates.insert ( std::make_pair (jp, dist) ) ;
    
    if (actionStates.size() > maxStoredActionStates) { 
        //max capacity reached, we have to delete the last one
        auto it = pairRet.first;        
        
        if (++(it) == actionStates.end() ){
           // the new inserted is the last one and has to be erased
            actionStates.erase(pairRet.first);
            return false;
        }
        
        // the new inserted is not the last one that has to be erased
        auto lastElem = actionStates.end();
        --lastElem;
        actionStates.erase(lastElem);
    }
    
    return true;
}


void ROSEE::ActionPinchWeak::print () const {
    
    std::stringstream output;
    
    output << "ActionName: " << name << std::endl;
    
    output << "FingersInvolved: [";
    for (auto fingName : fingersInvolved){
        output << fingName << ", " ;
    }
    output.seekp (-2, output.cur); //to remove the last comma (and space)
    output << "]" << std::endl;
    
    output << "JointsInvolvedCount: " << std::endl;;
    output << jointsInvolvedCount << std::endl;

    unsigned int nActState = 1;
    for (auto itemSet : actionStates) {  //the element in the set
        output << "Action_State_" << nActState << " :" << std::endl;

        output << "\t" << "JointStates:" << std::endl;
        output << itemSet.first;
        output << "\t" << "Distance:" << std::endl;
        output << "\t\tdistance " << itemSet.second << std::endl;
            
        nActState++;
    }
    output << std::endl;
    
    std::cout << output.str();

}


void ROSEE::ActionPinchWeak::emitYaml ( YAML::Emitter& out ) const {
    
    out << YAML::Key << YAML::Flow << fingersInvolved;

    unsigned int nCont = 1;
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "ActionName" << YAML::Value << name;
    out << YAML::Key << "JointsInvolvedCount" << YAML::Value << YAML::BeginMap;
    for (const auto &jointCount : jointsInvolvedCount ) {
        out << YAML::Key << jointCount.first;
        out << YAML::Value << jointCount.second;
    }
    out << YAML::EndMap;
    
    for (const auto & actionState : actionStates) { //.second is the set of ActionState
        
        std::string contSeq = "ActionState_" + std::to_string(nCont);
        out << YAML::Key << contSeq; 
        
        out << YAML::Value << YAML::BeginMap;
            //actionState.first, the jointstates map
            out << YAML::Key << "JointStates" << YAML::Value << YAML::BeginMap;
            for (const auto &joint : actionState.first) {
                out << YAML::Key << joint.first;
                out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
            }
            out << YAML::EndMap;
            
            //actionState.second, the optional
            out << YAML::Key << "Optional" << YAML::Value << YAML::BeginMap;
                out << YAML::Key << "distance" << YAML::Value << actionState.second;
            out << YAML::EndMap;
            
        out << YAML::EndMap;
        nCont++;
    }
    out << YAML::EndMap;

}


bool ROSEE::ActionPinchWeak::fillFromYaml ( YAML::const_iterator yamlIt ) {
        
    std::vector <std::string> fingInvolvedVect = yamlIt->first.as <std::vector < std::string >> ();
    for (const auto &it : fingInvolvedVect) {
        fingersInvolved.insert(it);
    }

    for ( YAML::const_iterator actionState = yamlIt->second.begin(); actionState != yamlIt->second.end(); ++actionState) {        
        // actionState->first is the key ActionState_x OR JointsInvolved
        
        std::string key = actionState->first.as<std::string>();
        if ( key.compare("JointsInvolvedCount") == 0 ) {
            jointsInvolvedCount = actionState->second.as < JointsInvolvedCount > ();
            
        } else if (key.compare ("ActionName") == 0 ) {
            name = actionState->second.as <std::string> ();
            
        } else if (key.compare(0, 12, "ActionState_") == 0) { //compare 12 caracters from index 0 of key

            JointPos jointPos;
            double distance;
            for(YAML::const_iterator asEl = actionState->second.begin(); asEl != actionState->second.end(); ++asEl) {

                //asEl can be the map JointStates or the map Optional
                if (asEl->first.as<std::string>().compare ("JointStates") == 0 ) {
                    jointPos = asEl->second.as < JointPos >(); 
                    
                } else if (asEl->first.as<std::string>().compare ("Optional") == 0 ) {
                    distance = asEl->second["distance"].as < double >();
                    
                } else {
                    //ERRROr, only joinstates and optional at this level
                    return false;
                }
            }  
            actionStates.insert ( std::make_pair (jointPos, distance));
            
        } else {
            //TODO print errorrr
        }
    }
    
    return true;
}


