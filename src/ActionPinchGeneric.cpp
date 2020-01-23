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

#include <ROSEndEffector/ActionPinchGeneric.h>

ROSEE::ActionPinchGeneric::ActionPinchGeneric() : 
    ActionPrimitive ("pinch", 2, 3, ActionType::Pinch) { }

ROSEE::ActionPinchGeneric::ActionPinchGeneric(unsigned int jointStateSetMaxSize) : 
    ActionPrimitive ("pinch", 2, jointStateSetMaxSize, ActionType::Pinch) { }
    
ROSEE::ActionPinchGeneric::ActionPinchGeneric(
    std::string name, unsigned int nLinksInvolved, unsigned int jointStateSetMaxSize, ROSEE::ActionType actionType) :
        ActionPrimitive (name, nLinksInvolved, jointStateSetMaxSize, actionType) { }


std::set < std::string > ROSEE::ActionPinchGeneric::getLinksInvolved() const {
 
    std::set < std::string> tempSet;
    tempSet.insert (tipsPair.first);
    tempSet.insert (tipsPair.second);
    
    return tempSet;    
}


bool ROSEE::ActionPinchGeneric::setLinksInvolved (std::set < std::string > setTips) {
    
    if (setTips.size() != 2 ) {
        return false;
    } else {
        std::set<std::string>::iterator it = setTips.begin();
        tipsPair.first = *it;
        std::advance(it,1);
        tipsPair.second = *it;
    }
    return true;
    
}


