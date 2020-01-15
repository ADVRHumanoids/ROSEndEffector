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

#include <string>
#include <vector>
#include <tuple>
#include <map>
#include <set>
#include <functional>
#include <iostream>
#include <memory>

//yaml
#include <yaml-cpp/yaml.h>

namespace ROSEE {
    
/** a map containing pairs jointNames-jointValues. vector of double because a joint can have more than 1 dof 
 * @NOTE: being a map the order (given by joint names) is assured
 */
typedef std::map < std::string, std::vector <double> > JointStates;
    
/**
 * @todo write docs
 */
class ActionPrimitive
{
    
protected:
    class OptPrimitive {
        public:
            virtual ~OptPrimitive(){};
            virtual bool operator > (const OptPrimitive &b) const {
                return true;
            };
            
            virtual std::ostream& printOpt (std::ostream &output) const {return output;};
            virtual bool emitYaml (YAML::Emitter&) const {return false;};
            //virtual bool parseYaml (OptPrimitive*, YAML::const_iterator) const {return false;};

        protected :
            OptPrimitive(){};

    };
    
private:
    std::ostream& printMap ( std::ostream &output ) const;    
    
public:
    
    typedef std::pair <JointStates, std::shared_ptr<OptPrimitive> > ActionState;

    /** struct to put in order the set of ActionState. */
    struct cmp {
        bool operator() ( const ActionState& a ,  const ActionState& b) const
        {return (*(a.second) > *(b.second);}
    };
    typedef std::map < std::set<std::string>, std::set<ActionState, cmp> > ActionMap; 
        
    virtual ~ActionPrimitive(){}; //important to have in parent class
    
    std::string actionName;
    unsigned int actionStateSetDim;
    bool optUsed; //true if any optional info is present in the set
    
    bool insertInMap ( std::set<std::string>, ActionState );   
    ActionMap getActionMap() const ;
    unsigned int getMapSize();
    
    friend std::ostream& operator << (std::ostream &output, const ActionPrimitive &a){
        return a.printMap(output);
    }

protected:
    ActionPrimitive();
    ActionMap actionMap;
    OptPrimitive* optPointer;
};


}

#endif // ACTIONPRIMITIVE_H
