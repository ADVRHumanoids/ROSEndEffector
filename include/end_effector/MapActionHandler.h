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

#include <end_effector/YamlWorker.h>
#include <end_effector/Utils.h>

#include <end_effector/GraspingActions/Action.h>
#include <end_effector/GraspingActions/ActionPrimitive.h>
#include <end_effector/GraspingActions/ActionTimed.h>
#include <end_effector/GraspingActions/ActionGeneric.h>



namespace ROSEE {
/**
 * @todo write docs
 */
class MapActionHandler {

public:
    
    typedef std::shared_ptr<MapActionHandler> Ptr;

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
    /**
     * @brief getter to take all the primitives maps of one type (\param type) 
     *      A primitive map is a map where key is the element involved (joints or fingers) and values the action primitive itself
     *      It returns a vector (differently from version with string as argument) because we can have more primitives
     *      with same type (e.g. a singleJointMultipleTips primitive, which are differente between them because of the joint they move 
     *      (singleJointMultipleTips_5 , singleJointMultipleTips_4 ... ) , or also multipinch)). This consideration is valid for all the other getPrimitive
     * @param type the primitive type of the action that we want
     * @return vector of all the primitives that are of type \param type
     * @warning Be sure to call parsing functions (parseAll*** ) otherwise no actions are returned never
     */
    std::vector<ActionPrimitiveMap> getPrimitiveMap( ROSEE::ActionPrimitive::Type type ) const;
    
    ActionPrimitiveMap getPrimitiveMap( std::string primitiveName )  const;
    std::map <std::string, ActionPrimitiveMap> getAllPrimitiveMaps () const;
    
    /**
     * @brief getter to take a single primitive, identified by the name and by the key (e.g. finger involved or joint involved)
     * 
     * There are various version of this get, if you pass the type a vector is returned. For some type, can be  
     * easier to use pair or single string as second argument instead of the set of string, when the type is associated with these
     * particular keys (e.g. trig has a single string (the finger used) as key, pinch has a pair (the 2 fingers used for the pinch) ... )
     *
     * @param primitiveName the name of the primitive that we want
     * @param key a set of strings to identify the primitive that we want among all the primitives that are called \param primitiveName
     * @example to get a pinch with thumb and index, we will pass "pinch" and a set of two element, "thumb" and "index". Note that 
     *      for this case we can also pass a pair (and not a set) as second argument
     * 
     * @return pointer to the primitive that we were looking for. nullptr if action not found.
     * 
     */
    ROSEE::ActionPrimitive::Ptr getPrimitive (std::string primitiveName, std::set<std::string> key) const;
    std::vector<ROSEE::ActionPrimitive::Ptr> getPrimitive (ROSEE::ActionPrimitive::Type, std::set<std::string> key) const;
    ROSEE::ActionPrimitive::Ptr getPrimitive (std::string primitiveName, std::vector<std::string> key) const;
    std::vector<ROSEE::ActionPrimitive::Ptr> getPrimitive (ROSEE::ActionPrimitive::Type, std::vector<std::string> key) const;
    ROSEE::ActionPrimitive::Ptr getPrimitive (std::string primitiveName, std::pair<std::string, std::string> key) const;
    std::vector<ROSEE::ActionPrimitive::Ptr> getPrimitive (ROSEE::ActionPrimitive::Type, std::pair<std::string, std::string> key) const;
    ROSEE::ActionPrimitive::Ptr getPrimitive (std::string primitiveName, std::string key) const;
    std::vector<ROSEE::ActionPrimitive::Ptr> getPrimitive (ROSEE::ActionPrimitive::Type, std::string key) const;
    
    /**
     * @note these functions return generics and composed (because composed are a derived class of generic)
     */
    std::shared_ptr<ROSEE::ActionGeneric> getGeneric (std::string name, bool verbose = true) const;
    std::map <std::string, std::shared_ptr<ROSEE::ActionGeneric>> getAllGenerics () const;
    
    std::shared_ptr<ROSEE::ActionTimed> getTimed (std::string name) const;
    std::map <std::string, std::shared_ptr<ROSEE::ActionTimed>> getAllTimeds () const;
    
    /************************************* Specific functions for specific actions *************************/


    /**
     * @brief function to return the map that contains all the singleJointMultipleTips primitive with that moves the specific number
     *    of fingers \p nFingers . This is in practice, the file singleJointMultipleTips_*nFingers*.yaml
     * 
     * It return only one map because per definition we have only one ActionPrimitiveMap of type singleJointMultipleTips with a defined number of 
     * nFinger. (Then inside it we can have more primitives ( when we have a hand with more joints that moves more than 1 finger), but
     * always with the same number of nFinger)
     * 
     * @param nFingers the number of the finger that the singleJointMultipleTips primitives moves
     * @return A map with key the joint that moves the \p nFingers and as value the primitive with all the info to command it to the hand.
     *      Obvioulsy we can have more joints that move a certain number (\p nFingers) of fingers
     * @todo return with the key as set instead??? or leave as it is?
     */
    std::map <std::string, ROSEE::ActionPrimitive::Ptr> getPrimitiveSingleJointMultipleTipsMap ( unsigned int nFingers ) const;   
    
    /**
     * @brief This function try to get an action that should be a grasp
     * 
     * A real grasp, until now, can be two things: 
     *  - a generic/composed action, composed by trig with all the fingers
     *  - a SingleJointMultipleTips primitive action, where the number of fingers moved is obviously the number of finger of the hand
     * In the second case, if we have more joints that moves all the fingers, we return no action because we do not know which one is
     * "the grasp". If we have only one joint that move all fingers, we return the singleJointMultipleTips action associated with this joint, even if
     * there is the possibility that this is not a grasp (but which hand has only one joint that moves all fingers and it is not for a grasp?)
     * 
     * This function look first for a generic (or composed, that is a derived class of generic) action with name \p graspName , 
     * that should have been created and emitted before calling the \ref parseAllActions (or \ref parseAllGenerics) 
     * (e.g. in UniversalFindActions putting all the trigs togheter, but we can also define the grasp in another way).
     *   
     * @param nFingers the number of the finger of the hand, that is an information that this class hasn't
     * @param graspName name given to the action grasp, default is "grasp"
     * @return pointer to Action, because the pointed object can be a primitive or a generic action.
     * @todo If not found, it tries to create a composed action, done with all trigs, and store this new action??   
     * @warning Generic and singleJointMultipleTips are different ros msg: singleJointMultipleTips want a element to be set (the joint) so this can cause problems..
     *   so maybe this function should not exist
     */
    ROSEE::Action::Ptr getGrasp ( unsigned int nFingers, std::string graspName = "grasp" ) ;
    
    /************************************* Other functions not returning actions themselves *************************/
    std::set<std::string> getFingertipsForPinch ( std::string finger, ROSEE::ActionPrimitive::Type pinchType) const;
    std::map <std::string, std::set<std::string> > getPinchTightPairsMap ( ) const;
    std::map <std::string, std::set<std::string> > getPinchLoosePairsMap ( ) const;
    
    /***************************** Other *******************************************/
    
    /** 
     * @brief This is needed by rosservicehandler which has to include a new doable action, if received the service
     * 
     */
    bool insertSingleGeneric(ROSEE::ActionGeneric::Ptr generic);

private:
    
    void findPinchPairsMap();
    
    std::string handName;

    std::map <std::string, ActionPrimitiveMap> primitives;
    std::map <std::string, std::shared_ptr<ROSEE::ActionGeneric>> generics;
    std::map <std::string, std::shared_ptr<ROSEE::ActionTimed>> timeds;
    
    std::map <std::string, std::set<std::string> > pinchTightPairsMap;
    std::map <std::string, std::set<std::string> > pinchLoosePairsMap;


};

} //namespace rosee

#endif // MAPACTIONHANDLER_H
