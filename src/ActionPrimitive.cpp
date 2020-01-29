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

#include <ROSEndEffector/ActionPrimitive.h>

ROSEE::ActionPrimitive::ActionPrimitive(
    std::string name, unsigned int nLinksInvolved, unsigned int jointStateSetMaxSize,
        ActionType actionType) :
        name(name), nLinksInvolved(nLinksInvolved), jointStateSetMaxSize(jointStateSetMaxSize),
        actionType(actionType) {}

std::string ROSEE::ActionPrimitive::getName () const{
    return name;
}

ROSEE::ActionType ROSEE::ActionPrimitive::getActionType() const {
    return actionType;
}

unsigned int ROSEE::ActionPrimitive::getJointStatesSetMaxSize() const {
    return jointStateSetMaxSize;
}

unsigned int ROSEE::ActionPrimitive::getnLinksInvolved() const {
    return nLinksInvolved;
}

std::vector<bool> ROSEE::ActionPrimitive::getJointsInvolved() const {
    return jointsInvolved;
}

void ROSEE::ActionPrimitive::setJointsInvolved(std::vector<bool> jointsInvolved ) {
    this->jointsInvolved = jointsInvolved;
}



void ROSEE::ActionPrimitive::printAction () const {

    std::stringstream output;
    for (auto names : getLinksInvolved()){
        output << names << ", " ;
    }
    output.seekp (-2, output.cur); //to remove the last comma (and space)
    output << std::endl;
    
    streamJointsInvolved ( output );

    unsigned int nActState = 1;
    for (auto item : getActionStates()) {  //the element in the vector
        output << "\tAction_State_" << nActState << " :" << std::endl;
        output << "\t\t" << "JointStates:" << std::endl;
        output << item;
        output << std::endl;
        nActState++;
    }
    output << std::endl;

    std::cout << output.str();
}

std::stringstream ROSEE::ActionPrimitive::streamJointsInvolved ( std::stringstream & output) const {
    std::stringstream outputTemp;
    outputTemp << "\tJointsInvolved: [" ;
    for (auto it : jointsInvolved) {
        outputTemp << it << ", "; 
    }
    outputTemp.seekp (-2, output.cur); //to remove the last comma (and space)
    outputTemp << "]" << std::endl;
    output << outputTemp.str();
    return outputTemp;
}

void ROSEE::ActionPrimitive::emitYaml ( YAML::Emitter& out )
{

    // key: set of string (eg two tip names)
    out << YAML::Key << YAML::Flow << getLinksInvolved();

    unsigned int nCont = 1;
    out << YAML::Value << YAML::BeginMap;
    emitYamlForJointsInvolved ( out);
    for (const auto & actionState : getActionStates()) {

        std::string contSeq = "ActionState_" + std::to_string(nCont);
        out << YAML::Key << contSeq;

        out << YAML::Value << YAML::BeginMap;
            //actionState.first, the jointstates map
            out << YAML::Key << "JointStates" << YAML::Value << YAML::BeginMap;
            for (const auto &joint : actionState) {
                out << YAML::Key << joint.first;
                out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
            }
            out << YAML::EndMap;

        out << YAML::EndMap;
        nCont++;
    }
    out << YAML::EndMap;
}

void ROSEE::ActionPrimitive::emitYamlForJointsInvolved ( YAML::Emitter& out) {
    
    out << YAML::Key << "JointsInvolved" 
        << YAML::Value << YAML::Flow << jointsInvolved ;
    
}


bool ROSEE::ActionPrimitive::fillFromYaml ( YAML::const_iterator yamlIt ) {

    std::vector <std::string> linksInvolvedVect = yamlIt->first.as <std::vector < std::string >> ();
    std::set <std::string> linksInvolvedSet;
        for (const auto &it : linksInvolvedVect) {
        linksInvolvedSet.insert(it);
    }
    setLinksInvolved(linksInvolvedSet);

    std::vector < ROSEE::JointStates > jointStateVect;
    for ( YAML::const_iterator actionState = yamlIt->second.begin(); actionState != yamlIt->second.end(); ++actionState) {
                
        if (actionState->first.as<std::string>().compare("JointsInvolved") == 0) {
            fillYamlJointsInvolved(actionState);
            
        } else {

            JointStates jointStates;
            for(YAML::const_iterator asEl = actionState->second.begin(); asEl != actionState->second.end(); ++asEl) {
                //asEl can be the map JointStates or the map Optional

                if (asEl->first.as<std::string>().compare ("JointStates") == 0 ) {
                    jointStates = asEl->second.as < JointStates >();
                } else if (asEl->first.as<std::string>().compare ("Optional") == 0 ) {

                    //an optional is present, override for your class!

                } else {
                    //ERRROr
                    return false;
                }
            }
            jointStateVect.push_back ( jointStates) ;
        }
    }

    setActionStates(jointStateVect);
    return true;
}

void ROSEE::ActionPrimitive::fillYamlJointsInvolved ( YAML::const_iterator yamlIt ) {
    
    if (yamlIt->first.as<std::string>().compare("JointsInvolved") == 0) {
        jointsInvolved = yamlIt->second.as<std::vector <bool> > ();
    } 
    
}

