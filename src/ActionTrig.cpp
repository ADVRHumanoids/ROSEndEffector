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

#include <ROSEndEffector/ActionTrig.h>

ROSEE::ActionTrig::ActionTrig (std::string actionName, ActionType actionType) :
    ActionPrimitive ( actionName, 1, 1, actionType ) { }

ROSEE::ActionTrig::ActionTrig (std::string actionName, ActionType actionType, std::string tip, JointStates js) :
    ActionPrimitive ( actionName, 1, 1, actionType ) {
        
    this->tip = tip;
    jointStates = js;    
}


std::set < std::string > ROSEE::ActionTrig::getLinksInvolved() const {
 
    std::set < std::string> tempSet;
    tempSet.insert (tip);
    
    return tempSet;    
}

std::string ROSEE::ActionTrig::getLinkInvolved () const {
    return tip;
}


std::vector < ROSEE::JointStates > ROSEE::ActionTrig::getActionStates() const{
    
    std::vector < JointStates> retVect {jointStates};
    return retVect;
}

ROSEE::JointStates ROSEE::ActionTrig::getActionState() const{    
    return jointStates;
}


bool ROSEE::ActionTrig::setLinksInvolved (std::set < std::string > setTips) {
    
    if (setTips.size() != 1 ) {
        return false;
    } else {
        std::set<std::string>::iterator it = setTips.begin();
        tip = *it;
    }
    return true;
}

void ROSEE::ActionTrig::setLinkInvolved (std::string linkName ) {
    tip = linkName;
}

bool ROSEE::ActionTrig::setActionStates (std::vector < ROSEE::JointStates > jsVect) {
    
    if (jsVect.size() != 1){
        return false;
    }
    jointStates = jsVect.at(0);
    return true;
}

bool ROSEE::ActionTrig::setActionState (ROSEE::JointStates js) {
    
    jointStates = js;
    return true;
}
