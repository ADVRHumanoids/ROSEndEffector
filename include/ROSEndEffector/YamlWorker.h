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
#include <ROSEndEffector/Utils.h>

#include <ROSEndEffector/Action.h>
#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionPinchStrong.h>
#include <ROSEndEffector/ActionPinchWeak.h>
#include <ROSEndEffector/ActionTrig.h>
#include <ROSEndEffector/ActionMoreTips.h>
#include <ROSEndEffector/ActionMultiplePinchStrong.h>

#include <ROSEndEffector/ActionComposed.h>
#include <ROSEndEffector/ActionTimed.h>

namespace ROSEE
{

/**
 * @todo PUT this in Parser?
 */
class YamlWorker
{
    
public:
    
    /**
     * @brief Costructor, with argument some info about where to save/take the yaml files: 
     * @param handName the name of the robot/hand (used to create file in a named folder)
     * @param path2saveYaml string containing the path where to save/overwrite the yaml files.
     *      Nothing can be passed, in this case a default location is used (see code) 
     */
    YamlWorker ( std::string handName, std::string path2saveYaml = "");

    /**
     * @brief Create/overwrite yaml file and emit info on it about each \ref ActionPrimitive inside the given \p mapOfActions.
     * @param mapOfActions container of actions which infos will be emitted on file
     * @param actionName the name of the action inside the map (that will have all same name)
     * @return std::string the filename (with the path) of the file created/overwritten
     */
    std::string createYamlFile ( const std::map < std::set <std::string> , ActionPrimitive* > mapOfActions, 
                                 const std::string actionName  ) ;
                                 
    /**
     * @brief Create/overwrite yaml file and emit info on it about the given ActionComposed \p action
     * @param action [in] pointer to ActionComposed
     * @return std::string the filename (with the path) of the file created/overwritten
     */                             
    std::string createYamlFile ( const Action* action) ;
        
    /**
     * @brief Parse a yaml file and return the map with all the actions present in the file. 
     * For the moment, a \p actionType argument must be passed to create the right Action object
     * @param filename the path of the file to be parsed
     * @param actionType the type of the action that is in the file
     * @param map of action parsed. This map contains pointer to ActionPrimitive, each pointer will point to a specific derived class
     */
    std::map < std::set < std::string>, ROSEE::ActionPrimitive::Ptr > parseYamlPrimitive ( 
                                    std::string filename, ROSEE::ActionPrimitive::Type actionType);
    
    /**
     * @brief Parse a composed Action
     * @param filename the path of the file to be parsed
     * @return the ActionComposed parsed
     * @deprecated use \ref parseYamlAction instead
     */
    ROSEE::ActionComposed parseYamlComposed (std::string filename);
    
    /**
     * 
     * @deprecated use \ref parseYamlAction instead
     */
    ROSEE::ActionTimed parseYamlTimed ( std::string filename );
    
    bool parseYamlAction (std::string filename, Action::Ptr ptrAction);

    std::string dirPath;
    
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



};

} //namespace ROSEE

#endif // __ROSEE_YAMLWORKER_H
