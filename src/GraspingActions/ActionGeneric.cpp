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

#include <end_effector/GraspingActions/ActionGeneric.h>


ROSEE::ActionGeneric::ActionGeneric () {}

ROSEE::ActionGeneric::ActionGeneric(std::string actionName) : Action (actionName, Action::Type::Generic) {}

ROSEE::ActionGeneric::ActionGeneric(std::string actionName, ROSEE::JointPos jointPos) : Action(actionName, Action::Type::Generic) {
    
    //HACK TODO now consider the position 0 as not used joint
    for (auto jp : jointPos) {
        bool zeros = std::all_of(jp.second.begin(), jp.second.end(), [](double i) { return i==0.0; });
        if (zeros) {
            jointsInvolvedCount.insert (std::make_pair (jp.first, 0) );
        } else {
            jointsInvolvedCount.insert (std::make_pair (jp.first, 1) );

        }
    } 
    
    this->jointPos = jointPos;

}

ROSEE::ActionGeneric::ActionGeneric(std::string actionName, ROSEE::JointPos jointPos, JointsInvolvedCount jic) : 
                            Action(actionName, Action::Type::Generic) {
                      
    if (jic.empty()) {
        //HACK TODO now consider the position 0 as not used joint
        for (auto jp : jointPos) {
            bool zeros = std::all_of(jp.second.begin(), jp.second.end(), [](double i) { return i==0.0; });
            if (zeros) {
                jointsInvolvedCount.insert (std::make_pair (jp.first, 0) );
            } else {
                jointsInvolvedCount.insert (std::make_pair (jp.first, 1) );

            }
        }
        
    } else {
        if ( ! ROSEE::Utils::keys_equal(jointPos, jic) ) {
            throw ROSEE::Utils::DifferentKeysException<ROSEE::JointPos, ROSEE::JointsInvolvedCount>(&jointPos, &jic);
        }     
        this->jointsInvolvedCount = jic;
    }
    
    this->jointPos = jointPos;

}


ROSEE::ActionGeneric::ActionGeneric(std::string actionName, ROSEE::JointPos jointPos, JointsInvolvedCount jic, 
                                  std::set<std::string> fingersInvolved) : Action(actionName, Action::Type::Generic) {
    
    if (jic.empty()) {
        //HACK TODO now consider the position 0 as not used joint
        for (auto jp : jointPos) {
            bool zeros = std::all_of(jp.second.begin(), jp.second.end(), [](double i) { return i==0.0; });
            if (zeros) {
                jointsInvolvedCount.insert (std::make_pair (jp.first, 0) );
            } else {
                jointsInvolvedCount.insert (std::make_pair (jp.first, 1) );

            }
        } 
        
    } else {
        
        if ( ! ROSEE::Utils::keys_equal(jointPos, jic) ) {
            throw ROSEE::Utils::DifferentKeysException<ROSEE::JointPos, ROSEE::JointsInvolvedCount>(&jointPos, &jic);
        }
        
        this->jointsInvolvedCount = jic;
    }
    
    this->jointPos = jointPos;

    this->fingersInvolved = fingersInvolved;
}

ROSEE::JointPos ROSEE::ActionGeneric::getJointPos() const {
    return jointPos;
}

std::vector<ROSEE::JointPos> ROSEE::ActionGeneric::getAllJointPos() const {
    std::vector < JointPos> vect;
    vect.push_back ( jointPos ) ;
    return vect;
}

void ROSEE::ActionGeneric::emitYaml(YAML::Emitter& out) const {
    
    out << YAML::BeginMap << YAML::Key << name << YAML::Value << YAML::BeginMap ;
        out << YAML::Key << "Type" << YAML::Value << type;
        out << YAML::Key << "FingersInvolved" << YAML::Value << YAML::Flow << fingersInvolved;
        out << YAML::Key << "JointsInvolvedCount" << YAML::Value << YAML::BeginMap;
        for (const auto &jointCount : jointsInvolvedCount ) {
            out << YAML::Key << jointCount.first;
            out << YAML::Value << jointCount.second;
        } 
        out << YAML::EndMap;

        out << YAML::Key << "JointPos" << YAML::Value << YAML::BeginMap;
            for (const auto &joint : jointPos) {
                out << YAML::Key << joint.first;
                out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
            }
        out << YAML::EndMap;
    out << YAML::EndMap;
    out << YAML::EndMap;
}


bool ROSEE::ActionGeneric::fillFromYaml(YAML::const_iterator yamlIt) {
    
    name = yamlIt->first.as<std::string>();
    type = ROSEE::Action::Type::Generic;
            
    for (auto keyValue = yamlIt->second.begin(); keyValue != yamlIt->second.end(); ++keyValue ) {

        std::string key = keyValue->first.as<std::string>();

        if ( key.compare ("FingersInvolved") == 0 ) { 
            // if <not_inserted> tempVect is a empty vector
            auto tempVect = keyValue->second.as <std::vector <std::string> > ();
            fingersInvolved.insert ( tempVect.begin(), tempVect.end() );
            
        } else if ( key.compare ("Type") == 0 ) {
            if (ROSEE::Action::Type::Generic != static_cast<ROSEE::Action::Type> ( keyValue->second.as <unsigned int>() )) {
                std::cout << "[GENERIC ACTION::" << __func__ << "] Error, found type  " << keyValue->second.as <unsigned int>()
                << "instead of generic type (" << ROSEE::Action::Type::Generic << ")" << std::endl;
                return false;
            }
            type = ROSEE::Action::Type::Generic;
            
        } else if ( key.compare ("JointsInvolvedCount") == 0 ) {
            jointsInvolvedCount = keyValue->second.as < JointsInvolvedCount >(); 
            
        } else if ( key.compare ("JointPos") == 0 ) {
            jointPos = keyValue->second.as < JointPos >();
            
        } else {
            std::cout << "[GENERIC ACTION::" << __func__ << "] Error, not known key " << key << std::endl;
            return false;
        }
    } 
    
    if ( ! ROSEE::Utils::keys_equal(jointPos, jointsInvolvedCount) ) {
        throw ROSEE::Utils::DifferentKeysException<ROSEE::JointPos, ROSEE::JointsInvolvedCount>(&jointPos, &jointsInvolvedCount);
    }
    
    return true;
}





