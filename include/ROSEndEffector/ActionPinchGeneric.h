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

#ifndef __ROSEE_ACTIONPINCHGENERIC_H
#define __ROSEE_ACTIONPINCHGENERIC_H

#include <ROSEndEffector/ActionPrimitive.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ROSEE {
/**
 * @brief A base virtual class for the PinchStrong and PinchWeak classes. It includes member and method that are
 * in common between the two type of pinches. It derives the more generic @ActionPrimitive
 */
class ActionPinchGeneric : public ActionPrimitive
{
    
public:
    
    ActionPinchGeneric();
    ActionPinchGeneric(unsigned int);
    ActionPinchGeneric(std::string name, unsigned int nLinksInvolved, unsigned int jointStateSetMaxSize,
        ActionType actionType);
    
    /** Overriden set and get from the pure virtual functions of the base class @ActionPrimitive */
    std::set < std::string > getLinksInvolved() const override;
    bool setLinksInvolved (std::set < std::string >) override;    

    /** the two tips that are involved in the action */
    std::pair <std::string, std::string > tipsPair ;

};

}

#endif // __ROSEE_ACTIONPINCHGENERIC_H
