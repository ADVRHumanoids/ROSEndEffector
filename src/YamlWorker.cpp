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
