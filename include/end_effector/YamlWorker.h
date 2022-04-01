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

#ifndef __ROSEE_YAMLWORKER_H
#define __ROSEE_YAMLWORKER_H

#define DEFAULT_ACTION_FOLDER "/configs/actions/"

#include <memory>

//yaml
#include <yaml-cpp/yaml.h>

//to create directories
#include <end_effector/Utils.h>

#include <end_effector/GraspingActions/Action.h>
#include <end_effector/GraspingActions/ActionPrimitive.h>
#include <end_effector/GraspingActions/ActionPinchTight.h>
#include <end_effector/GraspingActions/ActionPinchLoose.h>
#include <end_effector/GraspingActions/ActionTrig.h>
#include <end_effector/GraspingActions/ActionSingleJointMultipleTips.h>
#include <end_effector/GraspingActions/ActionMultiplePinchTight.h>

#include <end_effector/GraspingActions/ActionComposed.h>
#include <end_effector/GraspingActions/ActionTimed.h>

namespace ROSEE
{

/**
 * @todo PUT this in Parser?
 */
class YamlWorker
{
    
public:
    
    /**
     * @brief Costructor default
     */
    YamlWorker ( );

    /**
     * @brief Create/overwrite yaml file and emit info on it about each \ref ActionPrimitive inside the given \p mapOfActions.
     * @param mapOfActions container of actions which infos will be emitted on file
     * @param actionName the name of the action inside the map (that will have all same name)
     * @return std::string the filename (with the path) of the file created/overwritten
     */
    std::string createYamlFile ( const std::map < std::set <std::string> , ActionPrimitive* > mapOfActions, 
                                 const std::string actionName, std::string pathFolder  ) ;
                                 
    /**
     * @brief Create/overwrite yaml file and emit info on it about the given ActionComposed \p action
     * @param action [in] pointer to ActionComposed
     * @param pathFolder folder where to store the file. It will be created if does not exist
     * @return std::string the filename (with the path) of the file created/overwritten
     */                             
    std::string createYamlFile ( const Action* action, std::string pathFolder) ;
    std::string createYamlFile ( const ActionGeneric::Ptr, std::string pathFolder) ;
        
    /**
     * @brief Parse a yaml file and return the map with all the actions present in the file. 
     * For the moment, a \p actionType argument must be passed to create the right Action object
     * @param fileWithPath the path of the file to be parsed
     */
    std::map < std::set < std::string>, ROSEE::ActionPrimitive::Ptr > parseYamlPrimitive (std::string fileWithPath);
    
    std::map < std::set < std::string>, ROSEE::ActionPrimitive::Ptr > parseYamlPrimitive (std::string fileWithPath, ROSEE::ActionPrimitive::Type);
        
    /**
     * @brief Parse a composed Action
     * @param fileWithPath the path of the file to be parsed
     * @return the ActionComposed parsed 
     */
    ROSEE::ActionComposed parseYamlComposed (std::string fileWithPath);
    
    /**
     * @brief Parse a timed Action
     * @param fileWithPath the path of the file to be parsed
     * @return the ActionTimed parsed 
     */
    std::shared_ptr < ROSEE::ActionTimed > parseYamlTimed ( std::string fileWithPath);

    ROSEE::ActionGeneric::Ptr parseYamlGeneric ( std::string fileWithPath );
    
private:
    /**
     * @brief support functions for \ref createYamlFile
     * @param mapOfActions the map where take info from
     * @return std::string a string formatted as yaml file, ready to be put in the file
     */
    std::string emitYaml ( const std::map < std::set <std::string> , ActionPrimitive* > ) ;
    
    /** 
     * @brief support functions for \ref createYamlFile
     * @param action the action which infos must be emitted
     * @return std::string a string formatted as yaml file, ready to be put in the file
     */
    std::string emitYaml  ( const Action* action) ;
    std::string emitYaml  ( const ActionGeneric::Ptr action) ;



};

} //namespace ROSEE

#endif // __ROSEE_YAMLWORKER_H
