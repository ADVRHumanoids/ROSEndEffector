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


std::string ROSEE::YamlWorker::createYamlFile(
    std::map < std::set <std::string> , ActionPrimitive* > mapOfActions ) {

    ROSEE::Utils::create_directory ( dirPath );
    std::string output = emitYaml ( mapOfActions );

    ROSEE::Utils::out2file(dirPath + mapOfActions.begin()->second->getName() + ".yaml", output);
    return (dirPath + mapOfActions.begin()->second->getName() + ".yaml");
    
    
}


std::string ROSEE::YamlWorker::emitYaml ( 
    std::map < std::set <std::string> , ActionPrimitive* > mapOfActions ) {
    
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto & mapEl : mapOfActions) {
        mapEl.second->emitYaml(out);
        out << YAML::Newline << YAML::Newline; //double to insert a blanck line between tips pair
    }
    out << YAML::EndMap;
    return out.c_str();
    
}

std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > ROSEE::YamlWorker::parseYaml ( std::string filename, ROSEE::ActionType actionType){
    
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > parsedMap; 

    //TODO check elsewhere if file exist or not?
    std::ifstream ifile(dirPath + filename);
    if (! ifile) {
        std::cout << "YAMLPARSER: file " << dirPath + filename << " not found. " << 
            "Is this action possible with this hand?" << std::endl;
            return parsedMap;
    }
    
    YAML::Node node = YAML::LoadFile(dirPath + filename);
    
    for(YAML::const_iterator it4Action = node.begin(); it4Action != node.end(); ++it4Action) {
        std::shared_ptr <ActionPrimitive> ptr;
        switch (actionType) {
        case PinchStrong: {
            ptr = std::make_shared <ActionPinchStrong>();
            break;
        }
        case PinchWeak: {
            ptr = std::make_shared <ActionPinchWeak> ();
            break;
        }
        case Trig: {
            ptr = std::make_shared <ActionTrig>("trig", ActionType::Trig);
            break;
        }
        case TipFlex: {
            ptr = std::make_shared <ActionTrig>("tipFlex", ActionType::TipFlex);
            break;
        }
        case FingFlex: {
            ptr = std::make_shared <ActionTrig>("fingFlex", ActionType::FingFlex);
            break;
        }

        default : {
            std::cout << "YAML PARSER: " << actionType << " : type not found" << std::endl;
        }
        }
        
        ptr->fillFromYaml ( it4Action );
        
        parsedMap.insert ( std::make_pair ( ptr->getLinksInvolved(), ptr) );
    }
        
    return parsedMap;
}
