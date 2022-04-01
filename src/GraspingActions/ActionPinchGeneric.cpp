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

/** @TODO KNOW the distance at a certain percentage? so se volgio prendere un oggetto largo 3 cm il programma
 * sa quale è la percentage per avere una distanza di 3cm  
 *
 */

#include <end_effector/GraspingActions/ActionPinchGeneric.h>

ROSEE::ActionPinchGeneric::ActionPinchGeneric(std::string name, ActionPrimitive::Type type) : 
    ActionPrimitive (name, 3, type) { }

ROSEE::ActionPinchGeneric::ActionPinchGeneric(std::string name, unsigned int maxStoredActionStates, ActionPrimitive::Type type) : 
    ActionPrimitive (name, maxStoredActionStates, type) { }
    
ROSEE::ActionPinchGeneric::ActionPinchGeneric(
    std::string name, unsigned int nFingerInvolved, unsigned int maxStoredActionStates, ActionPrimitive::Type type) :
        ActionPrimitive (name, nFingerInvolved, maxStoredActionStates, type) { }

std::set<std::string> ROSEE::ActionPinchGeneric::getKeyElements() const {
    return fingersInvolved;
}

