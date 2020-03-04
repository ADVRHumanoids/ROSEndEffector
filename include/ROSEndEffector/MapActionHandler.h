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

    typedef std::map < std::set < std::string >, ROSEE::ActionPrimitive::Ptr > ActionPrimitiveMap;

    /**
     * @param folder where the action are. the action will be look in (<pkg_path> + pathFolder + "/" + handName + "/") ;
     */
    bool parseAllPrimitives(std::string pathFolder);
    bool parseAllGenerics(std::string pathFolder);
    bool parseAllTimeds(std::string pathFolder);
    bool parseAllActions(std::string pathFolder);

    /************************************* Generic Functions to get actions *************************/
    std::vector<ActionPrimitiveMap> getPrimitiveMap( ROSEE::ActionPrimitive::Type ) const;
    ActionPrimitiveMap getPrimitiveMap( std::string primitiveName )  const;
    std::map <std::string, ActionPrimitiveMap> getAllPrimitiveMaps () const;
    ROSEE::ActionPrimitive::Ptr getPrimitive (std::string primitiveName, std::set<std::string> key) const;
    std::vector<ROSEE::ActionPrimitive::Ptr> getPrimitive (ROSEE::ActionPrimitive::Type, std::set<std::string> key) const;
    ROSEE::ActionPrimitive::Ptr getPrimitive (std::string primitiveName, std::pair<std::string, std::string> key) const;
    std::vector<ROSEE::ActionPrimitive::Ptr> getPrimitive (ROSEE::ActionPrimitive::Type, std::pair<std::string, std::string> key) const;
    ROSEE::ActionPrimitive::Ptr getPrimitive (std::string primitiveName, std::string key) const;
    std::vector<ROSEE::ActionPrimitive::Ptr> getPrimitive (ROSEE::ActionPrimitive::Type, std::string key) const;
    
    std::shared_ptr<ROSEE::ActionGeneric> getGeneric (std::string name) const;
    std::map <std::string, std::shared_ptr<ROSEE::ActionGeneric>> getAllGenerics () const;
    
    ROSEE::ActionTimed getTimed (std::string name) const;
    std::map <std::string, ROSEE::ActionTimed> getAllTimeds () const;
    
    /************************************* Specific functions for specific actions *************************/


    /**
     * @brief function to return the map that contains all the moretips primitive with that moves the specific number
     *    of fingers \param nFingers. This is in practice, the file moreTips_*nFingers*.yaml
     * 
     * It return only one map because per definition we have only one ActionPrimitiveMap of type moretips with a defined number of 
     * nFinger. (Then inside it we can have more primitives ( when we have a hand with more joints that moves more than 1 finger), but
     * always with the same number of nFinger)
     * 
     * @param nFingers the number of the finger that the moreTips primitives moves
     * @return A map with key the joint that moves the \param nFingers and as value the primitive with all the info to command it to the hand.
     *      Obvioulsy we can have more joints that move a certain number (\param nFingers) of fingers
     * @todo return with the key as set instead??? or leave as it is?
     */
    std::map <std::string, ROSEE::ActionPrimitive::Ptr> getPrimitiveMoreTipsMap ( unsigned int nFingers ) const;   
    
    /**
     * @brief This function try to get an action that should be a grasp
     * 
     * A real grasp, until now, can be two things: 
     *  - a generic/composed action, composed by trig with all the fingers
     *  - a MoreTips primitive action, where the number of fingers moved is obviously the number of finger of the hand
     * In the second case, if we have more joints that moves all the fingers, we return no action because we do not know which one is
     * "the grasp". If we have only one joint that move all fingers, we return the moretips action associated with this joint, even if
     * there is the possibility that this is not a grasp (but which hand has only one joint that moves all fingers and it is not for a grasp?)
     * 
     * This function look first for a generic (or composed, that is a derived class of generic) action with name \param graspName, 
     * that should have been created and emitted before calling the \ref parseAllActions (or \ref parseAllGenerics) 
     * (e.g. in UniversalFindActions putting all the trigs togheter, but we can also define the grasp in another way).
     *   
     * @param nFingers the number of the finger of the hand, that is an information that this class hasn't
     * @param graspName name given to the action grasp, default is "grasp"
     * @return pointer to Action, because the pointed object can be a primitive or a generic action.
     * @todo If not found, it tries to create a composed action, done with all trigs, and store this new action??   
     * @warning Generic and moretips are different ros msg: moretips want a element to be set (the joint) so this can cause problems..
     *   so maybe this function should not exist
     */
    ROSEE::Action::Ptr getGrasp ( unsigned int nFingers, std::string graspName = "grasp" ) ;
    
    /************************************* Other functions not returning actions themselves *************************/
    std::set<std::string> getFingertipsForPinch ( std::string finger, ROSEE::ActionPrimitive::Type pinchType) const;
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
