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

#include <ROSEndEffector/ActionPinch.h>

ROSEE::ActionPinch::ActionPinch()
{
    actionName = "pinch";
    actionStateSetDim = 3;
    optUsed = true;
}

std::ostream& ROSEE::ActionPinch::OptPinch::printOpt (std::ostream &output) const {
    
    output << "MoveitContact: " << std::endl ;
    output << "\t\t\t\tbody_name_1: " << moveitContact.body_name_1 << std::endl;
    output << "\t\t\t\tbody_name_2: " << moveitContact.body_name_2 << std::endl;
    output << "\t\t\t\tbody_type_1: " << moveitContact.body_type_1 << std::endl;
    output << "\t\t\t\tbody_type_2: " << moveitContact.body_type_2 << std::endl;
    output << "\t\t\t\tdepth: " << moveitContact.depth << std::endl;
    //TODO print normal and pos

    return output;
}

bool ROSEE::ActionPinch::OptPinch::emitYaml ( YAML::Emitter& out) const {

    out << YAML::BeginMap;
        out << YAML::Key << "MoveItContact" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "body_name_1";
            out << YAML::Value << moveitContact.body_name_1;
            out << YAML::Key << "body_name_2";
            out << YAML::Value << moveitContact.body_name_2;
            out << YAML::Key << "body_type_1";
            out << YAML::Value << moveitContact.body_type_1;
            out << YAML::Key << "body_type_2";
            out << YAML::Value << moveitContact.body_type_2;
            out << YAML::Key << "depth";
            out << YAML::Value << moveitContact.depth;
            out << YAML::Key << "normal";
            std::vector < double > normal ( moveitContact.normal.data(), moveitContact.normal.data() +  moveitContact.normal.rows());  
            out << YAML::Value << YAML::Flow << normal;
            out << YAML::Key << "pos";
            std::vector < double > pos ( moveitContact.pos.data(), moveitContact.pos.data() +  moveitContact.pos.rows());
            out << YAML::Value << YAML::Flow << pos;
            out << YAML::EndMap;
    out << YAML::EndMap;
    
    return true;
}

bool ROSEE::ActionPinch::OptPinch::parseYaml (YAML::const_iterator it) {
    
    if (it->first.as<std::string>().compare ("Optional") != 0 ) {
        return false;
    }

    moveitContact.body_name_1 = it->second["body_name_1"].as<std::string>();
    moveitContact.body_name_2 = it->second["body_name_2"].as<std::string>();
    moveitContact.depth = it->second["depth"].as<double>();
    //TODO pos normal, body type
    
    return true;
    
}



std::pair < ROSEE::ActionPrimitive::ActionMap::iterator, bool> ROSEE::ActionPinch::insertInMap (std::string link1, std::string link2, JointStates jstates, collision_detection::Contact contact) {
    
    std::set<std::string> keys;
    keys.insert(link1);
    keys.insert(link2);
    
    std::shared_ptr<OptPrimitive> optPointer;
    optPointer = std::make_shared < OptPinch >(contact);
    
    
    return ActionPrimitive::insertInMap(keys, std::make_pair(jstates, optPointer));
    
}  

std::pair < ROSEE::ActionPrimitive::ActionMap::iterator, bool> ROSEE::ActionPinch::insertInMap (std::pair < std::string, std::string> keys, JointStates jstates, collision_detection::Contact contact) {
    
    return insertInMap(keys.first, keys.second, jstates, contact);
    
}

