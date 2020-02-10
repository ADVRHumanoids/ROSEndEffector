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

ROSEE::YamlWorker::YamlWorker ( std::string handName, std::string path2saveYaml)
{
    if ( path2saveYaml.compare("") == 0 ){
        dirPath = ROSEE::Utils::getPackagePath() + COLLIDER_REL_PATH 
            + "/" + handName + "/" ;
    } else {
        dirPath = ROSEE::Utils::getPackagePath() + path2saveYaml + "/" + handName + "/" ;
    }

}

std::string ROSEE::YamlWorker::createYamlFile( const ROSEE::ActionComposed* action) {
    
    ROSEE::Utils::create_directory ( dirPath );
    std::string output = emitYaml ( action );
    ROSEE::Utils::out2file(dirPath + action->getName() + ".yaml", output);
    return (dirPath + action->getName() + ".yaml");
    
}

std::string ROSEE::YamlWorker::createYamlFile(
    const std::map < std::set <std::string> , ActionPrimitive* > mapOfActions,
    const std::string actionName) {
    

    ROSEE::Utils::create_directory ( dirPath );
    std::string output = emitYaml ( mapOfActions );
    ROSEE::Utils::out2file(dirPath + actionName + ".yaml", output);
    return (dirPath + actionName + ".yaml");
    
    
}


std::string ROSEE::YamlWorker::emitYaml ( 
    const std::map < std::set <std::string> , ActionPrimitive* > mapOfActions ) {
    
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto & mapEl : mapOfActions) {
        mapEl.second->emitYaml(out);
        out << YAML::Newline << YAML::Newline; //double to insert a blanck line between tips pair
    }
    out << YAML::EndMap;
    return out.c_str();
    
}

std::string ROSEE::YamlWorker::emitYaml  ( const ROSEE::ActionComposed* action ) {
    
    YAML::Emitter out;
    action->emitYaml(out);
    out << YAML::Newline << YAML::Newline; //double to insert a blanck line
    return out.c_str();
    
}

std::map < std::set < std::string>, ROSEE::ActionPrimitive::Ptr > ROSEE::YamlWorker::parseYamlPrimitive ( 
                                                    std::string filename, ROSEE::ActionPrimitive::Type actionType){
    
    std::map < std::set < std::string>, ROSEE::ActionPrimitive::Ptr > parsedMap; 

    //TODO check elsewhere if file exist or not?
    std::ifstream ifile(dirPath + filename);
    if (! ifile) {
        std::cout << "[ERROR YAMLPARSER]: file " << dirPath + filename << " not found. "  << std::endl;
            return parsedMap;
    }
    
    YAML::Node node = YAML::LoadFile(dirPath + filename);
    
    for(YAML::const_iterator it4Action = node.begin(); it4Action != node.end(); ++it4Action) {
        ActionPrimitive::Ptr ptr;
        switch (actionType) {
        case ActionPrimitive::Type::PinchStrong: {
            ptr = std::make_shared <ActionPinchStrong>();
            break;
        }
        case ActionPrimitive::Type::PinchWeak: {
            ptr = std::make_shared <ActionPinchWeak> ();
            break;
        }
        case ActionPrimitive::Type::Trig: {
            ptr = std::make_shared <ActionTrig>("trig", ActionPrimitive::Type::Trig);
            break;
        }
        case ActionPrimitive::Type::TipFlex: {
            ptr = std::make_shared <ActionTrig>("tipFlex", ActionPrimitive::Type::TipFlex);
            break;
        }
        case ActionPrimitive::Type::FingFlex: {
            ptr = std::make_shared <ActionTrig>("fingFlex", ActionPrimitive::Type::FingFlex);
            break;
        }

        default : {
            std::cout << "[ERROR YAMLPARSER]: " << actionType << " : type not found" << std::endl;
        }
        }
        
        ptr->fillFromYaml ( it4Action );
        
        parsedMap.insert ( std::make_pair ( ptr->getFingersInvolved(), ptr) );
    }
        
    return parsedMap;
}

ROSEE::ActionComposed ROSEE::YamlWorker::parseYamlComposed (std::string filename){
    
    ROSEE::ActionComposed parsedAction; 

    //TODO check elsewhere if file exist or not?
    std::ifstream ifile(dirPath + filename);
    if (! ifile) {
        std::cout << "[ERROR YAMLPARSER]: file " << dirPath + filename << " not found. "  << std::endl;
            return parsedAction;
    }
    
    YAML::Node node = YAML::LoadFile(dirPath + filename);
    YAML::const_iterator yamlIt = node.begin();

    parsedAction.fillFromYaml ( yamlIt );
    
    return parsedAction;
    
}

