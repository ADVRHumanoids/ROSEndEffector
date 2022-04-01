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

#include <end_effector/GraspingActions/ActionTrig.h>

ROSEE::ActionTrig::ActionTrig (std::string actionName, ActionPrimitive::Type actionType) :
    ActionPrimitive ( actionName, 1, 1, actionType ) { }

ROSEE::ActionTrig::ActionTrig (std::string actionName, ActionPrimitive::Type actionType, std::string tip, JointPos jp) :
    ActionPrimitive ( actionName, 1, 1, actionType ) {
        
    fingersInvolved.insert(tip);
    jointPos = jp;   
}

std::string ROSEE::ActionTrig::getFingerInvolved () const {
    return *( fingersInvolved.begin() );
}

ROSEE::JointPos ROSEE::ActionTrig::getJointPos() const {
    return jointPos;
}

std::set<std::string> ROSEE::ActionTrig::getKeyElements() const {
    return fingersInvolved;
}

void ROSEE::ActionTrig::setJointPos(ROSEE::JointPos jointPos) {
    this->jointPos = jointPos;
}

std::vector < ROSEE::JointPos > ROSEE::ActionTrig::getAllJointPos() const{
    
    std::vector < JointPos > retVect {jointPos};
    return retVect;
}


void ROSEE::ActionTrig::setFingerInvolved (std::string fingName ) {
    fingersInvolved.clear();
    fingersInvolved.insert(fingName);
}

bool ROSEE::ActionTrig::fillFromYaml (  YAML::const_iterator yamlIt  ) {


    std::vector <std::string> fingInvolvedVect = yamlIt->first.as <std::vector < std::string >> ();
    for (const auto &it : fingInvolvedVect) {
        fingersInvolved.insert(it);
    }

    for ( YAML::const_iterator element = yamlIt->second.begin(); element != yamlIt->second.end(); ++element) {
        
        std::string key = element->first.as<std::string>();
        if ( key.compare ("ActionName") == 0 ){
            name = element->second.as < std::string > ();
            
        } else  if (key.compare("JointsInvolvedCount") == 0) {
            jointsInvolvedCount = element->second.as < JointsInvolvedCount > ();
        
        } else if (key.compare ("PrimitiveType") == 0) {
            ROSEE::ActionPrimitive::Type parsedType = static_cast<ROSEE::ActionPrimitive::Type> ( 
                element->second.as <unsigned int>() );
            if (parsedType != primitiveType ) {
                std::cerr << "[ERROR ActionTrig::" << __func__ << " parsed a type " << parsedType << 
                    " but this object has primitive type " << primitiveType << std::endl; 
                return false;
            }
            
        } else if (key.compare(0, 12, "ActionState_") == 0) { //compare 12 caracters from index 0 of key
            for(YAML::const_iterator asEl = element->second.begin(); asEl != element->second.end(); ++asEl) {
                //asEl can be the map JointStates or the map Optional

                if (asEl->first.as<std::string>().compare ("JointPos") == 0 ) {
                    jointPos =  asEl->second.as < JointPos >()  ;
                    
                } else {
                    //ERRROr, only JointPos at this level (optional is not for trig)
                    std::cerr << "[ERROR ActionTrig::" << __func__ << " not know key " 
                        << asEl->first.as<std::string>() << 
                        " found in the yaml file at this level" << std::endl; 
                    return false;
                }
            }
            
        } else {
            std::cerr << "[ERROR ActionTrig::" << __func__ << "not know key " << key << 
                " found in the yaml file" << std::endl; 
        }
    }
    return true;
}
