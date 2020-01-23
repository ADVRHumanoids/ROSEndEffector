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
    ActionPinchGeneric ("pinchWeak", 2, 3, ActionType::Pinch) { }

ROSEE::ActionPinchWeak::ActionPinchWeak (unsigned int jointStateSetMaxSize) : 
    ActionPinchGeneric ("pinchWeak", 2, jointStateSetMaxSize, ActionType::Pinch) { }

ROSEE::ActionPinchWeak::ActionPinchWeak (std::string tip1, std::string tip2) : 
    ActionPinchGeneric ("pinchWeak", 2, 3, ActionType::Pinch) {
        this->tipsPair.first = tip1;
        this->tipsPair.second = tip2;
    }
    
ROSEE::ActionPinchWeak::ActionPinchWeak (std::pair <std::string, std::string> tipNames, 
    JointStates js, double distance) :
    ActionPinchGeneric ("pinchWeak", 2, 3, ActionType::Pinch )  {

    //different from insertState, here we are sure the set is empty (we are in costructor)
    this->tipsPair = tipNames;
    statesInfoSet.insert (std::make_pair (js, distance) );
}


std::vector < ROSEE::JointStates > ROSEE::ActionPinchWeak::getActionStates() const{
    
    std::vector < JointStates > retVect;
    retVect.reserve(statesInfoSet.size());
    
    for (auto it : statesInfoSet ) {
        retVect.push_back(it.first);
    }
    
    return retVect;
}


std::vector < ROSEE::ActionPinchWeak::StateWithDistance > ROSEE::ActionPinchWeak::getActionStatesWithDistance() const {
    
    std::vector < ROSEE::ActionPinchWeak::StateWithDistance > retVect;
    retVect.reserve ( statesInfoSet.size() );
    
    for (auto it : statesInfoSet ) {
        retVect.push_back(it);
    }
    
    return retVect;
    
}


bool ROSEE::ActionPinchWeak::setActionStates (std::vector < ROSEE::JointStates > jsVect) {
    
    double dist = 0.0; //we need to initialize it because it is used in the set comparator
    for (auto it : jsVect) {
        if (! insertActionState (it, dist)) {
            return false;
        }
    }
    return true;
}

bool ROSEE::ActionPinchWeak::insertActionState (ROSEE::JointStates js, double dist) {

    auto pairRet = statesInfoSet.insert ( std::make_pair (js, dist) ) ;
    
    if (statesInfoSet.size() > jointStateSetMaxSize) { 
        //max capacity reached, we have to delete the last one
        auto it = pairRet.first;        
        
        if (++(it) == statesInfoSet.end() ){
           // the new inserted is the last one and has to be erased
            statesInfoSet.erase(pairRet.first);
            return false;
        }
        
        // the new inserted is not the last one that has to be erased
        auto lastElem = statesInfoSet.end();
        --lastElem;
        statesInfoSet.erase(lastElem);
    }
    
    return true;
}


void ROSEE::ActionPinchWeak::printAction () const {
    
    std::stringstream output;
    
    output << tipsPair.first << ", " << tipsPair.second << std::endl ;
    
    unsigned int nActState = 1;
    for (auto itemSet : statesInfoSet) {  //the element in the set
        output << "\tAction_State_" << nActState << " :" << std::endl;

        output << "\t\t" << "JointStates:" << std::endl;
        output << itemSet.first;
        output << "\t\t" << "Distance:" << std::endl;
        output << "\t\t\tdistance " << itemSet.second << std::endl;
            
        nActState++;
    }
    output << std::endl;
    
    std::cout << output.str();

}

bool ROSEE::ActionPinchWeak::emitYamlForDistance (double distance, YAML::Emitter& out) {

    out << YAML::BeginMap;
        out << YAML::Key << "Distance" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "distance";
            out << YAML::Value << distance;
        out << YAML::EndMap;
    out << YAML::EndMap;
    
    return true;
}

void ROSEE::ActionPinchWeak::emitYaml ( YAML::Emitter& out ) {
    
    // YAML << not valid for pair, we have to "convert" into vector
    std::vector <std::string> vectKeys {tipsPair.first, tipsPair.second};
    out << YAML::Key << YAML::Flow << vectKeys;
    
    unsigned int nCont = 1;
    out << YAML::Value << YAML::BeginMap;
    for (const auto & actionState : statesInfoSet) { //.second is the set of ActionState
        
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
            out << YAML::Key << "Optional" << YAML::Value;
            emitYamlForDistance(actionState.second, out);
            
        out << YAML::EndMap;
        nCont++;
    }
    out << YAML::EndMap;

}


bool ROSEE::ActionPinchWeak::fillFromYaml ( YAML::const_iterator yamlIt ) {
        
    tipsPair = yamlIt->first.as<std::pair < std::string, std::string >> ();

    for ( YAML::const_iterator actionState = yamlIt->second.begin(); actionState != yamlIt->second.end(); ++actionState) {        
        // actionState->first is the key ActionState_x

        JointStates jointStates;
        double distance;
        for(YAML::const_iterator asEl = actionState->second.begin(); asEl != actionState->second.end(); ++asEl) {

            //asEl can be the map JointStates or the map Optional
            if (asEl->first.as<std::string>().compare ("JointStates") == 0 ) {
                jointStates = asEl->second.as < JointStates >(); 
            } else if (asEl->first.as<std::string>().compare ("Optional") == 0 ) {
                
                YAML::Node node =  asEl->second["Distance"];
                distance = node["distance"].as < double >();
                
            } else {
                //ERRROr, only joinstates and optional at this level
                return false;
            }
        }  
        statesInfoSet.insert ( std::make_pair (jointStates, distance));
    }
    
    return true;
}


