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

#ifndef ROSEE_ACTIONPINCH_H
#define ROSEE_ACTIONPINCH_H

#include <iostream>

#include <ROSEndEffector/ActionPrimitive.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>





/** Max contact stored in the set for each pair */
#define MAX_CONTACT_STORED 3

namespace ROSEE {

/**
 * @todo write docs
 */
class ActionPinch : public ActionPrimitive
{
    
private:
    
    class OptPinch : public ActionPrimitive::OptPrimitive {
    public:
        
        OptPinch ( collision_detection::Contact contact) {moveitContact = contact;};
        
        /** @FIX, even if is almost impossible, two different contact with same depth will be considered equal
        * with this definition of operator >. Theoretically they are equal only if the joint status are equal 
        * (of only joints that act for the collision). In fact, we should have the possibility to have two contact
        * with the same depth (if joint statuses are different), they will be equally good
        */
        bool operator > (const OptPrimitive &b) const override {
             if (const OptPinch* point = dynamic_cast < const OptPinch* >(&b)) {
                return std::abs(moveitContact.depth) > std::abs(point->moveitContact.depth);
            } else {
                return false;
            }
        };
        
        std::ostream& printOpt ( std::ostream &output ) const override;
        bool emitYaml ( YAML::Emitter& ) const override;
        bool parseYaml (YAML::const_iterator) override ;
        
        collision_detection::Contact moveitContact;
        
    };
    
    
public:
    
    /**
     * Default constructor
     */
    ActionPinch();
    
    std::pair < ActionMap::iterator, bool> insertInMap (std::string, std::string, JointStates, collision_detection::Contact);    
    std::pair < ActionMap::iterator, bool> insertInMap (std::pair < std::string, std::string>, JointStates , collision_detection::Contact);
 

};

}

#endif // ROSEE_ACTIONPINCH_H
