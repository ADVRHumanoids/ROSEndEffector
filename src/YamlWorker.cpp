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

#include <ROSEndEffector/YamlWorker.h>

ROSEE::YamlWorker::YamlWorker ( std::string handName)
{
    dirPath = ROSEE::Utils::getPackagePath() + COLLIDER_REL_PATH 
        + "/" + handName + "/" ;
}


std::string ROSEE::YamlWorker::createYamlFile ( const ROSEE::ActionPrimitive action ) {

    ROSEE::Utils::create_directory ( dirPath );
    std::string output = emitYaml ( action);

    ROSEE::Utils::out2file(dirPath + action.actionName + ".yaml", output);
    return (dirPath + action.actionName + ".yaml");
    
}


std::string ROSEE::YamlWorker::emitYaml ( const ROSEE::ActionPrimitive action )
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto & mapEl : action.getActionMap() ) {
    
        // key: set of string (eg two tip names)
        out << YAML::Key << YAML::Flow << mapEl.first;
        
        unsigned int nCont = 1;
        out << YAML::Value << YAML::BeginMap;
        for (const auto & actionState : mapEl.second) { //.second is the set of ActionState
            
            std::string contSeq = "ActionState_" + std::to_string(nCont);
            out << YAML::Key << contSeq; 
            
            out << YAML::Value << YAML::BeginMap;
                //actionState.first, the jointstates map
                out << YAML::Key << "JointStates" << YAML::Value << YAML::BeginMap;
                for (const auto &joint : actionState.first) {
                    out << YAML::Key << joint.first;
                    out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
                }
                out << YAML::EndMap;
                
                //actionState.second, the optional
                out << YAML::Key << "Optional" << YAML::Value;
                actionState.second->emitYaml(out);
            out << YAML::EndMap;
            nCont++;
        }
        out << YAML::EndMap;
        out << YAML::Newline << YAML::Newline; //double to insert a blanck line between tips pair
    }
    out << YAML::EndMap;
    return out.c_str();    
}


//TODO, parse same order for action state, see comparator
ROSEE::ActionPrimitive ROSEE::YamlWorker::parseYaml ( std::string filename, std::string actionName ){
    
    ROSEE::ActionPrimitive genericParsedAction; 
    genericParsedAction.actionStateSetDim = 3;
    YAML::Node node = YAML::LoadFile(dirPath + filename);
        
    for(YAML::const_iterator mapEl = node.begin(); mapEl != node.end(); ++mapEl) {
        std::vector <std::string> keyMapVect = mapEl->first.as<std::vector<std::string>>();
        std::set <std::string> keyMap;
        for (const auto &it : keyMapVect) {
            keyMap.insert(it);
        }
    
        for ( YAML::const_iterator actionState = mapEl->second.begin(); actionState != mapEl->second.end(); ++actionState) {
            for(YAML::const_iterator  asEl= actionState->second.begin(); asEl != actionState->second.end(); ++asEl) {
                //asEl can be the map JointStates or the map Optional
                if (asEl->first.as<std::string>().compare ("JointStates") == 0 ) {
                    
                    JointStates jointMap = asEl->second.as < JointStates >(); 
                    std::pair < ActionPrimitive::ActionMap::iterator, bool> insResult = genericParsedAction.insertInMap ( keyMap, jointMap ) ;
                }
            }
        }
    }

    
    return genericParsedAction;
}
