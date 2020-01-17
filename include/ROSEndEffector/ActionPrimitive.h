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

#ifndef ACTIONPRIMITIVE_H
#define ACTIONPRIMITIVE_H

#include <set>
#include <string>
#include <map>
#include <vector>
#include <iostream>

#include <yaml-cpp/yaml.h>


namespace ROSEE{
    
typedef std::map <std::string, std::vector <double> > JointStates;
std::ostream& operator << (std::ostream& output, const JointStates js) {
    for (const auto &jsEl : js) {
        output << "\t\t\t"<<jsEl.first << " : "; //joint name
        for(const auto &jValue : jsEl.second){
            output << jValue << ", "; //joint position (vector because can have multiple dof)
        }
        output << std::endl;       
    }
}

/**
 * @todo write docs
 */
class ActionPrimitive
{
protected:
 
    ActionPrimitive();
    
    //TODO make these constant (and initialize them with initializer list)
    std::string name;
    unsigned int nLinksInvolved;
    unsigned int jointStateSetMaxSize;
    
public:
    
    virtual ~ActionPrimitive() {};

    
    std::string getName () const;
    unsigned int getJointStatesSetMaxSize() const;
    unsigned int getnLinksInvolved() const;
    
    virtual std::set < std::string > getLinksInvolved() const = 0;
    virtual std::vector < ROSEE::JointStates > getActionStates() const = 0;
    virtual bool setLinksInvolved (std::set < std::string >) = 0;
    virtual bool setActionStates (std::vector < ROSEE::JointStates > ) = 0;
    
    virtual void printAction () const ;
    virtual void emitYaml ( YAML::Emitter& ) ;
    virtual bool fillFromYaml( YAML::const_iterator yamlIt );

    
    
    

};
}

#endif // ACTIONPRIMITIVE_H
