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

#ifndef ACTIONCOMPOSED_H
#define ACTIONCOMPOSED_H

#include <vector>
#include <string>
#include <ROSEndEffector/ActionPrimitive.h>
#include <yaml-cpp/yaml.h>
#include <memory>

/** 
 * TODO, for the independance of primitives, now we check if they have setter same joints.
 * Now this is done check if a joint pos is zero; but also zero pos can be a setted pos.
 * idea is to add a boolJointStates in ActionPrimitive where each joint is true or false depend
 * if is setted or not.
 * TODO independance: how to mean the jointvalues? we can mean always dividing per the number of 
 * primitives (nPrimitives). But with this method, we add in the mean also a lot of zero values which
 * should be the non setted state, so should not influence the mean. Possible solution is to introduce a vector
 * of counter which says, for each joint, how much are the primitive that use that particular joint and
 * so we can divide each joint for the right sum of primitive. DONEEEEEEEE write docs
 * 
 * TODO instead of use it as "Composed", use it as single action, that is a different structure respect to
 * action primitive, with a single joint state. Then can contain a single action, or more.
 * 
 * TODO add un member "fingerInvolved" ? set of string che contiene tutti i diti coinvolti
 * 
 * TODO una remove? non è così facile da implementare e non so se sia utile...
 */
namespace ROSEE{

class ActionComposed
{
private:
    std::string name;
    std::vector < std::string > primitiveNames;
    std::set < std::string > linksInvolved;
    ROSEE::JointStates jointStates;
    std::vector < std::shared_ptr <ROSEE::ActionPrimitive> > primitiveObjects;
    unsigned int nPrimitives;
    
    bool independent; //true if each primitive must set different joint states
    std::vector <unsigned int> involvedJointsCount;
    
public: 
    ActionComposed();
    ActionComposed(std::string name);
    ActionComposed(std::string name, bool independent);
    // Copy constructor 
    ActionComposed (const ActionComposed &other);
    
    std::string getName () const;
    unsigned int getnPrimitives () const;
    bool getIndependent () const;
    std::vector <std::string> getPrimitiveNames() const ;
    std::set <std::string> getLinksInvolved() const ;
    ROSEE::JointStates getJointStates() const;
    std::vector < std::shared_ptr <ROSEE::ActionPrimitive> > getPrimitiveObjects() const;
    std::vector <unsigned int> getInvolvedJointsCount () const;
    
    void printAction () const ; 
    void emitYaml ( YAML::Emitter&) const;
    bool fillFromYaml( YAML::Node node );
    
    bool sumPrimitive ( std::shared_ptr <ROSEE::ActionPrimitive>, int as = 0 );
    
};
}

#endif // ACTIONCOMPOSED_H
