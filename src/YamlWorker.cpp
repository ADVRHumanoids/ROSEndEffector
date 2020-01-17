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

/**
//TODO return the generic class movement
std::map < std::pair < std::string, std::string >, std::map < std::string, ROSEE::ActionPinch::JointStates> >
    ROSEE::YamlWorker::parseYaml ( std::string filename ){
    
    std::map < std::pair < std::string, std::string >, std::map < std::string, ActionPinch::JointStates> > pinchParsedMap; 
    YAML::Node node = YAML::LoadFile(dirPath + filename);
        
    for(YAML::const_iterator tipPair = node.begin(); tipPair != node.end(); ++tipPair) {
        std::pair <std::string, std::string> tipNames = tipPair->first.as<std::pair<std::string, std::string>>();
        auto insResult = pinchParsedMap.insert ( std::make_pair( tipNames, std::map<std::string, ActionPinch::JointStates> () ) );
        
        //TODO check if new insertion, for security reason
        if (!insResult.second) {
            //PAIR already present, some error with the yaml file
        }
        
        for ( YAML::const_iterator setElem = tipPair->second.begin(); setElem != tipPair->second.end(); ++setElem) {
            
            for(YAML::const_iterator cont = setElem->second.begin(); cont != setElem->second.end(); ++cont) {
                //cont can be the map MoveItContact or JointStates
                
                if (cont->first.as<std::string>().compare ("JointStates") == 0 ) {
                    
                    ActionPinch::JointStates jointMap = cont->second.as < ActionPinch::JointStates >(); 
                    insResult.first->second.insert(
                        std::make_pair (setElem->first.as<std::string>(), jointMap)); //map insert return also the iterator to the added element
                }
            }
        }
    }

    
    return pinchParsedMap;
}

*/
