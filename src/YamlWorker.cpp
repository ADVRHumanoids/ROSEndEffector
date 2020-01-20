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
        std::cout << "YAMLPARSER: file" << dirPath + filename << "not found. " << 
            "Is this action possible with this hand?" << std::endl;
            return parsedMap;
    }
    
    YAML::Node node = YAML::LoadFile(dirPath + filename);
    
    for(YAML::const_iterator it4Action = node.begin(); it4Action != node.end(); ++it4Action) {
        std::shared_ptr <ActionPrimitive> ptr;
        switch (actionType) {
        case Pinch: {
            ptr = std::make_shared <ActionPinch>();
            break;
        }
        case Trig: {
            ptr = std::make_shared <ActionTrig>();
            break;
        }
        default : {
            //ERROR STOP ALL
        }
        }
        
        ptr->fillFromYaml ( it4Action );
        
        parsedMap.insert ( std::make_pair ( ptr->getLinksInvolved(), ptr) );
    }
        
    return parsedMap;
}
