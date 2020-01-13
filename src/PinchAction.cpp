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

#include <ROSEndEffector/PinchAction.h>

#include <string> //TODO levalo da qua

ROSEE::PinchAction::PinchAction()
{

}

bool ROSEE::PinchAction::insertMap(
    std::pair < std::string, std::string > tipsNames, ContactWithJointStates contactJstates){
    
    auto it = pinchMap.find(tipsNames);
    if (it == pinchMap.end()) { //new pair
        
        std::set<ContactWithJointStates, depthComp> newSet;
        newSet.insert(contactJstates);
        
        pinchMap.insert(std::make_pair(tipsNames, newSet));
        
    } else if (it->second.size() < MAX_CONTACT_STORED){
            
        it->second.insert(contactJstates); // the set will insert in order for us
            
    } else if (contactJstates.first.depth > it->second.rbegin()->first.depth) {

        it->second.insert(contactJstates);
        //delete the last element
        std::set<ContactWithJointStates, depthComp>::iterator lastElem = it->second.end();
        --lastElem;
        it->second.erase(lastElem);
        
    } else {
        return false; //no new added
    }
    return true;
}

void ROSEE::PinchAction::printMap(){
    std::stringstream logStream;
    logStream << "Contact list: " << std::endl ;
    
    for (const auto &item : pinchMap){
        logStream << "\t" << item.first.first << ", " << item.first.second << ": "<< std::endl;
            
        for (auto contact : item.second) {  //the element in the set
            logStream << "\tWith depth of: " << contact.first.depth << " and joint states:" << std::endl;
            
            for (const auto &jointState : contact.second) {
                logStream << "\t\t" << jointState.first << " : "; //joint name
                for(const auto &jointValue : jointState.second){
                    logStream << jointValue << ", "; //joint position (vector because can have multiple dof)
                }
            logStream << std::endl;       
            }
        }
        logStream << std::endl;
    }    
    std::cout << logStream.str() << std::endl;
}
