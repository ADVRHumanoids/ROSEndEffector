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

#ifndef MAPACTIONHANDLER_H
#define MAPACTIONHANDLER_H

#include <string>
#include <map>
#include <set>
#include <memory>

#include <ROSEndEffector/YamlWorker.h>
#include <ROSEndEffector/Utils.h>

#include <ROSEndEffector/Action.h>
#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionTimed.h>
#include <ROSEndEffector/ActionGeneric.h>



namespace ROSEE {
/**
 * @todo write docs
 */
class MapActionHandler {

public:

    MapActionHandler();
    MapActionHandler(std::string handName);

    typedef std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > ActionPrimitiveMap;

    /**
     * @param folder where the action are. the action will be look in (<pkg_path> + pathFolder + "/" + handName + "/") ;
     */
    bool parseAllPrimitives(std::string pathFolder = "");
    bool parseAllGenerics(std::string pathFolder = "");
    bool parseAllTimeds(std::string pathFolder = "");
    bool parseAllActions(std::string pathFolder = "");

    std::vector<ActionPrimitiveMap> getPrimitive( ROSEE::ActionPrimitive::Type ) const;
    ActionPrimitiveMap getPrimitive( std::string primitiveName )  const;
    std::map <std::string, ActionPrimitiveMap> getAllPrimitives () const;
    
    std::shared_ptr<ROSEE::ActionGeneric> getGeneric (std::string name) const;
    std::map <std::string, std::shared_ptr<ROSEE::ActionGeneric>> getAllGenerics () const;
    
    ROSEE::ActionTimed getTimed (std::string name) const;
    std::map <std::string, ROSEE::ActionTimed> getAllTimed () const;
    
    
    std::set<std::string> getFingertipsForPinch ( std::string finger, unsigned int choice = 0  ) const;
    std::map <std::string, std::set<std::string> > getPinchStrongPairsMap ( ) const;
    std::map <std::string, std::set<std::string> > getPinchWeakPairsMap ( ) const;

private:
    
    void findPinchPairsMap();
    
    std::string handName;

    std::map <std::string, ActionPrimitiveMap> primitives;
    std::map <std::string, std::shared_ptr<ROSEE::ActionGeneric>> generics;
    std::map <std::string, ROSEE::ActionTimed> timeds;
    
    std::map <std::string, std::set<std::string> > pinchStrongPairsMap;
    std::map <std::string, std::set<std::string> > pinchWeakPairsMap;


};

} //namespace rosee

#endif // MAPACTIONHANDLER_H
