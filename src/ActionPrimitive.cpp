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

#include <ROSEndEffector/ActionPrimitive.h>

ROSEE::ActionPrimitive::ActionPrimitive(){
    actionName = "NONE";
    actionStateSetDim = 1;
    optUsed = false;
}

bool ROSEE::ActionPrimitive::insertInMap ( std::set<std::string> keys, ActionState actState) {
    
    auto it = actionMap.find(keys);
    if (it == actionMap.end()) { //new pair, we have to create the set
        
        std::set < ActionState, cmp> newSet;
        newSet.insert(actState);
        actionMap.insert(std::make_pair(keys, newSet));
        
    } else if (it->second.size() < actionStateSetDim){
            
        it->second.insert(actState); // the set will insert in order for us
            
    } else if (actState.second > it->second.rbegin()->second) {

        it->second.insert(actState);
        //delete the last element
        std::set < ActionState, cmp >::iterator lastElem = it->second.end();
        --lastElem;
        it->second.erase(lastElem);
        
    } else {
        return false; //no new added
    }
    return true;
}

std::ostream& ROSEE::ActionPrimitive::printMap (std::ostream &output) const {
    
    output << "ACTION " << actionName << std::endl ;
    
    for (const auto &item : actionMap){
        for (auto keyNames : item.first){
            output << keyNames << ", " ;
        }
        output << std::endl;
        unsigned int nActState = 1;
        for (auto itemSet : item.second) {  //the element in the set
            output << "\tAction_State_" << nActState << " :" << std::endl;

            output << "\t\t" << "Joint States:" << std::endl;
            for (const auto &jointState : itemSet.first) {
                output << "\t\t\t" << jointState.first << " : "; //joint name
                for(const auto &jointValue : jointState.second){
                    output << jointValue << ", "; //joint position (vector because can have multiple dof)
                }
                output << std::endl;       
            }
            if (optUsed) {
                output << "\t\t" << "Optional:" << std::endl << "\t\t\t";
                itemSet.second->printOpt(output);
                
            }
            output << std::endl;
            nActState++;
        }
        output << std::endl;
    }
    output << std::endl << std::endl;
    
    return output;
}

unsigned int ROSEE::ActionPrimitive::getMapSize(){    
    return actionMap.size();   
}

ROSEE::ActionPrimitive::ActionMap ROSEE::ActionPrimitive::getActionMap() const { return actionMap; }


