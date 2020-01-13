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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>


/** Max contact stored in the set for each pair */
#define MAX_CONTACT_STORED 3

namespace ROSEE {

/**
 * @todo write docs
 */
class ActionPinch
{
public:
    
    //*****************************************TODO put this in the generic move class
    
    /** a map containing pairs jointNames-jointValues. vector of double because a joint can have more than 1 dof @NOTE: being a map the order (given by joint names) is assured*/
    typedef std::map <std::string, std::vector <double> > JointStates;

    /**Contact informations for a contact that happens with a particular joint states*/
    typedef std::pair <collision_detection::Contact, JointStates> ContactWithJointStates;     
        
    /** struct to put in order the set of ContactWithJointStates. The first elements are the ones 
     * with greater depth
     * @FIX, even if is almost impossible, two different contact with same depth will be considered equal
     * with this definition of depthComp. Theoretically they are equal only if the joint status are equal 
     * (of only joints that act for the collision). In fact, we should have the possibility to have two contact
     * with the same depth (if joint statuses are different), they will be equally good
     * @TODO put this in the generic move class
     */
    struct depthComp {
        bool operator() (const ContactWithJointStates& a, const ContactWithJointStates& b) const
        {return (std::abs(a.first.depth) > std::abs(b.first.depth) );}
    };
    /** The object that contains all the "best" MAX_CONTACT_STORED contact for each possible pair. 
     It is a map with as key the pair of the two tips colliding, 
     and as value a fixed size set of ContactWithJointStates object*/
    std::map < std::pair < std::string, std::string >, std::set<ContactWithJointStates, depthComp>> pinchMap; 
    
    //***************************************************
    
    
    /**
     * Default constructor
     */
    ActionPinch();
    
        /** 
     * @brief insert the new contact in the map, if it is among the best ones
     */
    bool insertMap( std::pair < std::string, std::string > tipsNames, ContactWithJointStates contactJstates);
    void printMap();
    
    std::string name;
    
    
    

};

}

#endif // ROSEE_ACTIONPINCH_H
