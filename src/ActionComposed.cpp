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

#include <ROSEndEffector/ActionComposed.h>

ROSEE::ActionComposed::ActionComposed(){
    name = "ComposedGeneric";
    independent = true;
    nPrimitives = 0;
}

ROSEE::ActionComposed::ActionComposed(std::string name) {
    this->name = name;
    this->independent = true;
    nPrimitives = 0;
}

ROSEE::ActionComposed::ActionComposed(std::string name, bool independent) {
    this->name = name;
    this->independent = independent;
    nPrimitives = 0;
}


std::string ROSEE::ActionComposed::getName() const {
    return name;
}


std::vector<std::string> ROSEE::ActionComposed::getPrimitiveNames() const {
    return primitiveNames;
}

std::vector < std::shared_ptr <ROSEE::ActionPrimitive> > ROSEE::ActionComposed::getPrimitiveObjects() const {
    return primitiveObjects;
}

ROSEE::JointStates ROSEE::ActionComposed::getJointStates() const {
    return jointStates;
}


bool ROSEE::ActionComposed::sumPrimitive ( std::shared_ptr <ROSEE::ActionPrimitive> primitive, int as )
{
    
    if (nPrimitives == 0) { //first primitive inserted, add anyway
        jointStates = primitive->getActionStates().at(as);
        
    } else {
    
        if (independent) {
            JointStates js = primitive->getActionStates().at(as);
            for (auto joint : js ){
                //HACK single dof joint now
                if ( joint.second.at(0) != 0.0 &&
                    jointStates.at(joint.first).at(0) != 0.0 ) {
                    return false; //both setted, cant add this primitive
                }
            }
            
            for (auto joint : js ){ 
                if ( jointStates.at(joint.first).at(0) == 0.0 ){
                    jointStates.at(joint.first).at(0) = joint.second.at(0);
                }       
            }
        } else {
            //TODO, or not?
        }
    }
    
    primitiveNames.push_back ( primitive->getName() );
    primitiveObjects.push_back ( primitive );
    nPrimitives ++;
    
    
    return true;
}

void ROSEE::ActionComposed::printAction() const {
    
    std::stringstream output;
    
    output << "Composed Action " << name;
    independent ? output << " (independent)" : output << " (not independent)" ;
    output << std::endl;
    output << "Composed by " << nPrimitives << " primitives: [" ;
    for (auto it : primitiveNames) {
        output << it << ", ";
    }
    output.seekp (-2, output.cur); //to remove the last comma (and space)
    output << "]" << std::endl;
    
    output << "\t" << "JointStates:" << std::endl;
    output << jointStates << std::endl;
    
    std::cout << output.str();
    
}


void ROSEE::ActionComposed::emitYaml ( YAML::Emitter& out) const {
    
    out << YAML::BeginMap;
        out << YAML::Key << "Name" << YAML::Value << name;
        out << YAML::Key << "Independent" << YAML::Value << independent;
        out << YAML::Key << "nPrimitives" << YAML::Value << nPrimitives;
        out << YAML::Key << "Primitives" << YAML::Value << YAML::Flow << primitiveNames;
        out << YAML::Key << "JointStates" << YAML::Value << YAML::BeginMap;
            for (const auto &joint : jointStates) {
                out << YAML::Key << joint.first;
                out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
            }
        out << YAML::EndMap;
    out << YAML::EndMap;
}

// if parsed, we dont have the link with the primitives... so primitiveObjects is empty
bool ROSEE::ActionComposed::fillFromYaml ( YAML::Node node ) {
    
    for (auto keyValue = node.begin(); keyValue != node.end(); ++keyValue ){

        std::string key = keyValue->first.as<std::string>();
        if ( key.compare ("Name") == 0 ) {
            name = keyValue->second.as<std::string>();
            
        } else if ( key.compare ("Independent") == 0 ) {
            independent = keyValue->second.as<bool>();
            
        } else if ( key.compare ("nPrimitives") == 0 ) {
            nPrimitives = keyValue->second.as <unsigned int>();
            
        } else if ( key.compare ("Primitives") == 0 ) {
            primitiveNames = keyValue->second.as <std::vector <std::string> >();
            
        } else if ( key.compare ("JointStates") == 0 ) {
            jointStates = keyValue->second.as < JointStates >();
            
        } else {
            std::cout << "[COMPOSED ACTION PARSER] Error, not known key " << key << std::endl;
            return false;
        }
    } 
    return true;
}



