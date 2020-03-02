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

#include <ROSEndEffector/MapActionHandler.h>

ROSEE::MapActionHandler::MapActionHandler() {}

ROSEE::MapActionHandler::MapActionHandler(std::string handName) {
    
    this->handName = handName;
}

ROSEE::MapActionHandler::ActionPrimitiveMap ROSEE::MapActionHandler::getPrimitive(std::string primitiveName) const{
    
    auto it = primitives.find (primitiveName);
    if (it == primitives.end() ){
        std::cerr << "[ERROR MapActionHandler::" << __func__ << "] Not found any action with name " << primitiveName << std::endl;
        return ActionPrimitiveMap();
    }
    
    return it->second;
}

std::vector<ROSEE::MapActionHandler::ActionPrimitiveMap> ROSEE::MapActionHandler::getPrimitive(ROSEE::ActionPrimitive::Type type) const {
    
    std::vector<ROSEE::MapActionHandler::ActionPrimitiveMap> vectRet;
    
    for (auto it : primitives) {
        
        if (it.second.begin()->second->getPrimitiveType() == type ){
            vectRet.push_back(it.second);

        }
    }
    
    return vectRet;
    
}

std::map<std::string, ROSEE::MapActionHandler::ActionPrimitiveMap> ROSEE::MapActionHandler::getAllPrimitives() const {
    return primitives;
}


std::map<std::string, std::set<std::string> > ROSEE::MapActionHandler::getPinchStrongPairsMap() const {
    return pinchStrongPairsMap;
}

std::map<std::string, std::set<std::string> > ROSEE::MapActionHandler::getPinchWeakPairsMap() const {
    return pinchWeakPairsMap;
}

std::shared_ptr<ROSEE::ActionGeneric> ROSEE::MapActionHandler::getGeneric(std::string name) const {
    
    auto it = generics.find(name);
    if (it == generics.end() ) {
         std::cerr << "[ERROR MapActionHandler " << __func__ << "] No generic function named" << name << std::endl;    
        return nullptr;
    }
    
    return it->second;
}

std::map<std::string, std::shared_ptr<ROSEE::ActionGeneric> > ROSEE::MapActionHandler::getAllGenerics() const {
    return generics;
}

ROSEE::ActionTimed ROSEE::MapActionHandler::getTimed(std::string name) const {
    
    auto it = timeds.find(name);
    if (it == timeds.end() ) {
         std::cerr << "[ERROR MapActionHandler " << __func__ << "] No timed function named" << name << std::endl;    
        return ActionTimed();
    }
    
    return it->second;
}

std::map<std::string, ROSEE::ActionTimed> ROSEE::MapActionHandler::getAllTimed() const {
    return timeds;
}

std::set<std::string> ROSEE::MapActionHandler::getFingertipsForPinch(std::string finger, unsigned int choice) const {
    
    std::set <std::string> pairedFinger;
    
    if (choice == 0) {
        
        auto it = pinchStrongPairsMap.find(finger);
        
        if ( it != pinchStrongPairsMap.end() ) {
            pairedFinger = it->second;
            
        } else {
            std::cerr << "[WARNING MapActionHandler " << __func__ << "] No companions found to make a strong pinch with " << finger << " finger" 
            << std::endl;
        }
        
    } else if (choice == 1 ) {
        
        auto it = pinchWeakPairsMap.find(finger);
        
        if ( it != pinchWeakPairsMap.end() ) {
            pairedFinger = it->second;
            
        } else {
            std::cerr << "[WARNING MapActionHandler " << __func__ << "] No companions found to make a weak pinch with " << finger << " finger" 
            << std::endl;
        } 
        
    } else { //both strong a weak companions
       
        auto it = pinchStrongPairsMap.find(finger);
        if ( it != pinchStrongPairsMap.end() ) {
            pairedFinger.insert ( it->second.begin(), it->second.end() );
        }
        
        it = pinchWeakPairsMap.find(finger);
        if ( it!= pinchWeakPairsMap.end() ) { 
            pairedFinger.insert ( it->second.begin(), it->second.end() );
        }
        
        if (pairedFinger.size() == 0 ){
            std::cerr << "[WARNING MapActionHandler " << __func__ << "] No companions found to make a strong or weak pinch with " 
            << finger << " finger" << std::endl;    
        }
    }
    
    return pairedFinger;
}


bool ROSEE::MapActionHandler::parseAllActions(std::string pathFolder) {
    
    bool flag = true;
    if (! parseAllPrimitives(pathFolder + "/primitives/") ) {
        flag = false;
    }
    if (! parseAllGenerics(pathFolder + + "/generics/") ) {
        flag = false;
    }
    if (! parseAllTimeds(pathFolder + + "/timeds/") ) {
        flag = false;
    }
    
    return flag;
    
}

bool ROSEE::MapActionHandler::parseAllPrimitives(std::string pathFolder) {
    
    std::vector <std::string> filenames = ROSEE::Utils::getFilesInDir(pathFolder);
    YamlWorker yamlWorker;
    
     for (auto file : filenames) {
        ActionPrimitiveMap primitive = yamlWorker.parseYamlPrimitive(pathFolder+file);
        primitives.insert (std::make_pair( primitive.begin()->second->getName(), primitive) ) ;
    }
    
    findPinchPairsMap();
    
    return true;
    
}

bool ROSEE::MapActionHandler::parseAllGenerics(std::string pathFolder) {

    std::vector <std::string> filenames = ROSEE::Utils::getFilesInDir(pathFolder);
    YamlWorker yamlWorker;
    for (auto file : filenames) {
        
        std::shared_ptr<ROSEE::ActionGeneric> genericPointer = yamlWorker.parseYamlGeneric(pathFolder+file);
        generics.insert (std::make_pair( genericPointer->getName(), genericPointer) ) ;
    }
    
    
    return true;
    
    
}

bool ROSEE::MapActionHandler::parseAllTimeds(std::string pathFolder) {
    
    std::vector <std::string> filenames = ROSEE::Utils::getFilesInDir(pathFolder);
    YamlWorker yamlWorker;
    for (auto file : filenames) {
        
        ROSEE::ActionTimed timed = yamlWorker.parseYamlTimed(pathFolder+file);
        timeds.insert (std::make_pair( timed.getName(), timed) ) ;
    }
    
    
    return true;
    
}




//******************************** PRIVATE FUNCT ********************************************************************/

void ROSEE::MapActionHandler::findPinchPairsMap() {
    
    //Assume only one pinch strong, it should be like this now.
    auto maps = getPrimitive(ROSEE::ActionPrimitive::Type::PinchStrong);
    
    if (maps.size() != 0 ){
        for (ActionPrimitiveMap map : maps) {
            for (auto mapEl : map) {
        
                for (auto fing : mapEl.first) { //.first is a set
                    
                    //we will insert all the set as value, this means that also will include the key itself,
                    // we remove the key from the values later
                    if (pinchStrongPairsMap.count(fing) == 0 ) {
                        pinchStrongPairsMap.insert(std::make_pair(fing, mapEl.first)); 
                    } else {
                        pinchStrongPairsMap.at(fing).insert (mapEl.first.begin(), mapEl.first.end());
                    }
                }
            }
        }  
        //remove the string key from the values.
        for (auto it : pinchStrongPairsMap) {
            it.second.erase(it.first);
        }
    }
    
    //now do the same for the weak pinches
    auto mapsweak = getPrimitive(ROSEE::ActionPrimitive::Type::PinchWeak);
    
    if (mapsweak.size() != 0 ) {
        for (ActionPrimitiveMap map : mapsweak) {
            for (auto mapEl : map) {
        
                for (auto fing : mapEl.first) { //.first is a set
                    
                    //we will insert all the set as value, this means that also will include the key itself,
                    // we remove the key from the values later
                    if (pinchWeakPairsMap.count(fing) == 0 ) {
                        pinchWeakPairsMap.insert(std::make_pair(fing, mapEl.first)); 
                    } else {
                        pinchWeakPairsMap.at(fing).insert (mapEl.first.begin(), mapEl.first.end());
                    }
                }
            }
        }
        //remove the string key from the values.
        for (auto it : pinchWeakPairsMap) {
            it.second.erase(it.first);
        }
    }
}


