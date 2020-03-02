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

#include <ROSEndEffector/ActionMoreTips.h>

ROSEE::ActionMoreTips::ActionMoreTips() : 
    ActionPrimitive ( "moreTips", 1, ROSEE::ActionPrimitive::Type::MoreTips ) {}

ROSEE::ActionMoreTips::ActionMoreTips(std::string actionName, unsigned int nFingers ) : 
    ActionPrimitive ( actionName, nFingers, 1, ROSEE::ActionPrimitive::Type::MoreTips ) {}

ROSEE::ActionMoreTips::ActionMoreTips (std::string actionName, std::vector<std::string> fingers, std::string jointName, 
                                       JointPos jpFurther, JointPos jpNearer) : 
    ActionPrimitive ( actionName, fingers.size(), 1, ROSEE::ActionPrimitive::Type::MoreTips ) {
        
    this->jointInvolved = jointName;
    fingersInvolved.insert (fingers.begin(), fingers.end());
    
    this->jointPosFurther = jpFurther;
    this->jointPosNearer = jpNearer;
    
    // still need to do this, that can be done in costructor because we know that only one joint will be used, per definition of this action
    for (auto it : jpFurther ) {
        jointsInvolvedCount.insert (std::make_pair (it.first, 0) );
    }
    jointsInvolvedCount.at (jointName) = 1;
        
}

std::vector<ROSEE::JointPos> ROSEE::ActionMoreTips::getAllJointPos() const {
    
    std::vector<JointPos> vect;
    vect.push_back (jointPosFurther); 
    vect.push_back (jointPosNearer);
    return vect;
}

ROSEE::JointPos ROSEE::ActionMoreTips::getJointPos() const {
    return jointPosFurther;
}

ROSEE::JointPos ROSEE::ActionMoreTips::getJointPosFurther() const {
    return jointPosFurther;
}

ROSEE::JointPos ROSEE::ActionMoreTips::getJointPosNearer() const {
    return jointPosNearer;
}

std::string ROSEE::ActionMoreTips::getJointName() const {
    return jointInvolved;
}

std::set<std::string> ROSEE::ActionMoreTips::getKeyForYamlMap() const {
    std::set <std::string> set;
    set.insert(jointInvolved);
    return set;
}

void ROSEE::ActionMoreTips::print() const {
    
    std::stringstream output;
    output << "ActionName: " << name << std::endl;
    
    output << "FingersInvolved: [";
    for (auto fingName : fingersInvolved){
        output << fingName << ", " ;
    }
    output.seekp (-2, output.cur); //to remove the last comma (and space)
    output << "]" << std::endl;
    
    output << "Joint which moves the tips: " << jointInvolved << std::endl;
    
    output << "JointPos Further from 0:" << std::endl;
    output << jointPosFurther;
    
    output << "JointPos Nearer to 0:" << std::endl;
    output << jointPosNearer;

    output << std::endl;

    std::cout << output.str();
    
}

void ROSEE::ActionMoreTips::emitYaml(YAML::Emitter& out) const {
    
    out << YAML::Key << jointInvolved;
    out << YAML::Value << YAML::BeginMap;
    
        out << YAML::Key << "PrimitiveType" << YAML::Value << primitiveType;
        out << YAML::Key << "ActionName" << YAML::Value << name;
        out << YAML::Key << "FingersInvolved" << YAML::Value << YAML::Flow << fingersInvolved;
        
        out << YAML::Key << "JointPosFurther" << YAML::Value << YAML::BeginMap;
        for (const auto &joint : jointPosFurther) {
            out << YAML::Key << joint.first;
            out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
        }
        out << YAML::EndMap;
        
        out << YAML::Key << "JointPosNearer" << YAML::Value << YAML::BeginMap;
        for (const auto &joint : jointPosNearer) {
            out << YAML::Key << joint.first;
            out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
        }
        out << YAML::EndMap;
    
    out << YAML::EndMap;
    
}



bool ROSEE::ActionMoreTips::fillFromYaml(YAML::const_iterator yamlIt) {
    
    jointInvolved = yamlIt->first.as < std::string > ();
    
    for ( YAML::const_iterator element = yamlIt->second.begin(); element != yamlIt->second.end(); ++element) {
        std::string key = element->first.as<std::string>();
        
         if ( key.compare ("ActionName") == 0 ){
            name = element->second.as < std::string > ();
            
        } else if (key.compare("FingersInvolved") == 0) {
            std::vector <std::string> fingInvolvedVect = element->second.as <std::vector < std::string >> ();
            for (const auto &it : fingInvolvedVect) {
                fingersInvolved.insert(it);
            }
            
        } else if (key.compare ("JointPosNearer") == 0) {
            jointPosNearer = element->second.as <JointPos>();
            
        } else if (key.compare ("JointPosFurther") == 0) {
            jointPosFurther = element->second.as <JointPos>();
        } else {
            //todo strange error
            return false;
        }
    }
    
    // we have also to fill this structure, that is not present on yaml file because redundant 
    for (auto it : jointPosFurther ) {
        jointsInvolvedCount.insert (std::make_pair (it.first, 0) );
    }
    jointsInvolvedCount.at (jointInvolved) = 1;
    
    nFingersInvolved = fingersInvolved.size();

    return true;
    
}




