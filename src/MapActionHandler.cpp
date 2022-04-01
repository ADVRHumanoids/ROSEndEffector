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

#include <end_effector/MapActionHandler.h>

ROSEE::MapActionHandler::MapActionHandler() {}

ROSEE::MapActionHandler::ActionPrimitiveMap ROSEE::MapActionHandler::getPrimitiveMap(std::string primitiveName) const{
    
    auto it = primitives.find (primitiveName);
    if (it == primitives.end() ){
        std::cerr << "[ERROR MapActionHandler::" << __func__ << "] Not found any primitive action with name " << primitiveName << std::endl;
        return ActionPrimitiveMap();
    }
    
    return it->second;
}

std::vector<ROSEE::MapActionHandler::ActionPrimitiveMap> ROSEE::MapActionHandler::getPrimitiveMap(ROSEE::ActionPrimitive::Type type) const {
    
    std::vector<ROSEE::MapActionHandler::ActionPrimitiveMap> vectRet;
    
    for (auto it : primitives) {
        
        if (it.second.begin()->second->getPrimitiveType() == type ){
            vectRet.push_back(it.second);

        }
    }
    
    return vectRet;
    
}

std::map<std::string, ROSEE::MapActionHandler::ActionPrimitiveMap> ROSEE::MapActionHandler::getAllPrimitiveMaps() const {
    return primitives;
}

ROSEE::ActionPrimitive::Ptr ROSEE::MapActionHandler::getPrimitive(std::string primitiveName, std::set<std::string> key) const {
    
    auto map = getPrimitiveMap(primitiveName);
    
    if (map.size() == 0 ) { 
        return nullptr; //error message already printed in getPrimitiveMap
    }
    
    if (map.begin()->second->getKeyElements().size() != key.size()) {
        std::cerr << "[ERROR MapActionHandler::" << __func__ << "] The action '" 
        << primitiveName << "' has as key a set of dimension " <<
        map.begin()->second->getKeyElements().size() <<
        " and not dimension of passed 2nd argument " << key.size() << std::endl;
        return nullptr;
    }
    
    auto it = map.find(key);
    
    if (it == map.end()) {
        std::cerr << "[ERROR MapActionHandler::" << __func__ << "] Not found any action '" 
            << primitiveName << "' with key [ " ;
        for (auto keyEl : key) {
            std::cerr << keyEl << ", ";
        }
        std::cerr << "] " << std::endl;
        return nullptr;
    }
    
    return it->second;
}

ROSEE::ActionPrimitive::Ptr ROSEE::MapActionHandler::getPrimitive ( std::string primitiveName, std::vector<std::string> key) const {
    
    std::set <std::string> keySet (key.begin(), key.end());
    return getPrimitive(primitiveName, keySet);
}

ROSEE::ActionPrimitive::Ptr ROSEE::MapActionHandler::getPrimitive(std::string primitiveName, std::string key) const {
    
    std::set <std::string> keySet {key};
    return getPrimitive(primitiveName, keySet);
}

ROSEE::ActionPrimitive::Ptr ROSEE::MapActionHandler::getPrimitive(std::string primitiveName, std::pair<std::string,std::string> key) const {
    
    std::set <std::string> keySet {key.first, key.second};
    return getPrimitive(primitiveName, keySet);
}

std::vector<ROSEE::ActionPrimitive::Ptr> ROSEE::MapActionHandler::getPrimitive(ROSEE::ActionPrimitive::Type type, std::set<std::string> key) const {
    
    std::vector <ActionPrimitiveMap> maps = getPrimitiveMap(type);
    
    //now we look among the maps, for all the maps that has key size as the size of key passed
    std::vector <ActionPrimitiveMap> theMaps;
    for (int i =0; i<maps.size(); i++) {
        if (maps.at(i).begin()->second->getKeyElements().size() == key.size()) {
            theMaps.push_back(maps.at(i));
        }
    }
    
    if (theMaps.size() == 0 ) {
        std::cerr << "[ERROR MapActionHandler::" << __func__ << "] No primitive action of type '" 
        << type << "' has as key a set of dimension " <<
        key.size() << " (passed 2nd argument)" << std::endl;
        return std::vector<ROSEE::ActionPrimitive::Ptr>();
    }
    
    //now we look, among all the themaps (where key size is the same as the passed arg key)
    // for an action that as effectively the wanted key. We can have more than one action
    // with the wanted key because in theMaps vector we have different primitives (altought 
    // of same type). This is not possible now (because singleJointMultipleTips have as key a joint, so
    // a joint cant move X fingers and ALSO Y fingers) (and multiplePinch action have all 
    // different key set size ( the number of fing used for multpinch). Anyway a vect is
    // returned because we do not know if in future we will have new type of primitives.
    std::vector<ROSEE::ActionPrimitive::Ptr> returnVect;
    for (auto action : theMaps) {
        auto it = action.find(key);
        if (it != action.end()) {
            returnVect.push_back(it->second);
        }
    }
    
    if (returnVect.size() == 0) {
        std::cerr << "[ERROR MapActionHandler::" << __func__ << "] Not found any primitive action of type '" << type << "' with key [ " ;
        for (auto keyEl : key) {
            std::cerr << keyEl << ", ";
        }
        std::cerr << "] " << std::endl;
        return std::vector<ROSEE::ActionPrimitive::Ptr>();
    }
    
    return returnVect;
    
}

std::vector<ROSEE::ActionPrimitive::Ptr> ROSEE::MapActionHandler::getPrimitive ( ROSEE::ActionPrimitive::Type type, std::vector<std::string> key) const {
    
    std::set <std::string> keySet (key.begin(), key.end());
    return getPrimitive(type, keySet);
}

std::vector<ROSEE::ActionPrimitive::Ptr> ROSEE::MapActionHandler::getPrimitive(ROSEE::ActionPrimitive::Type type, std::string key) const {
    
    std::set <std::string> keySet {key};
    return getPrimitive(type, keySet);
}

std::vector<ROSEE::ActionPrimitive::Ptr> ROSEE::MapActionHandler::getPrimitive(ROSEE::ActionPrimitive::Type type, std::pair<std::string,std::string> key) const {
    
    std::set <std::string> keySet {key.first, key.second};
    return getPrimitive(type, keySet);
}

std::map<std::string, std::set<std::string> > ROSEE::MapActionHandler::getPinchTightPairsMap() const {
    return pinchTightPairsMap;
}

std::map<std::string, std::set<std::string> > ROSEE::MapActionHandler::getPinchLoosePairsMap() const {
    return pinchLoosePairsMap;
}

std::shared_ptr<ROSEE::ActionGeneric> ROSEE::MapActionHandler::getGeneric(std::string name, bool verbose) const {
    
    auto it = generics.find(name);
    if (it == generics.end() ) {
        if (verbose) {
            std::cerr << "[ERROR MapActionHandler " << __func__ << "] No generic function named '" << name << "'" << std::endl;
        }
        return nullptr;
    }
    
    return it->second;
}

std::map<std::string, std::shared_ptr<ROSEE::ActionGeneric> > ROSEE::MapActionHandler::getAllGenerics() const {
    return generics;
}

std::shared_ptr<ROSEE::ActionTimed> ROSEE::MapActionHandler::getTimed(std::string name) const {
    
    auto it = timeds.find(name);
    if (it == timeds.end() ) {
         std::cerr << "[ERROR MapActionHandler " << __func__ << "] No timed function named '" << name << "'" << std::endl;    
        return nullptr;
    }
    
    return it->second;
}

std::map<std::string, std::shared_ptr<ROSEE::ActionTimed>> ROSEE::MapActionHandler::getAllTimeds() const {
    return timeds;
}


std::map<std::string, ROSEE::ActionPrimitive::Ptr> ROSEE::MapActionHandler::getPrimitiveSingleJointMultipleTipsMap(unsigned int nFingers) const {
    
    std::map<std::string, ROSEE::ActionPrimitive::Ptr> ret;
    
    for (auto it : primitives) {
        
        if (it.second.begin()->second->getPrimitiveType() ==
            ROSEE::ActionPrimitive::Type::SingleJointMultipleTips && 
            it.second.begin()->second->getnFingersInvolved() == nFingers){
            
            //copy the map into one similar but with as key a strign and not a set
            for (auto itt : it.second) {
                //itt.first is the set of one element
                std::string key = *(itt.first.begin());
                ret.insert(std::make_pair(key, itt.second));
            }

        }
    }
    
    if (ret.size() == 0) {
        std::cerr << "[WARNING MapActionHandler::" << __func__ << "] Not found any singleJointMultipleTips action that moves " << nFingers << " fingers " << std::endl;
    }
    
    return ret;
}

ROSEE::Action::Ptr ROSEE::MapActionHandler::getGrasp(unsigned int nFingers, std::string graspName) {
    
    auto it = generics.find(graspName);
    if (it != generics.end()) {
        return it->second;
    }
    
    auto moreTip = getPrimitiveSingleJointMultipleTipsMap(nFingers);
    if (moreTip.size() == 1) { //if more than 1 I do not know how to choose the one that effectively "grasp"
        return moreTip.begin()->second;
    }
    
    std::cerr << "[WARNING MapActionHandler::" << __func__ << "] Not found any grasp named " << graspName << " neither a singleJointMultipleTips primitive " 
        << "that move all fingers with a single joint, you should create one action for grasp before calling parseAllActions/parseAllGenerics()"
        << std::endl;

    
    return nullptr;
    
}


std::set<std::string> ROSEE::MapActionHandler::getFingertipsForPinch(std::string finger, ROSEE::ActionPrimitive::Type pinchType) const {
    
    std::set <std::string> pairedFinger;
    
    switch (pinchType) {
        
    case ROSEE::ActionPrimitive::Type::PinchTight : {
        
        auto it = pinchTightPairsMap.find(finger);
        
        if ( it != pinchTightPairsMap.end() ) {
            pairedFinger = it->second;
            
        } else {
            std::cerr << "[WARNING MapActionHandler " << __func__ << "] No companions found to make a tight pinch with " << finger << " finger" 
            << std::endl;
        }
        break;
    }
    
    case ROSEE::ActionPrimitive::Type::PinchLoose : {
        
        auto it = pinchLoosePairsMap.find(finger);
        
        if ( it != pinchLoosePairsMap.end() ) {
            pairedFinger = it->second;
            
        } else {
            std::cerr << "[WARNING MapActionHandler " << __func__ << "] No companions found to make a loose pinch with " << finger << " finger" 
            << std::endl;
        } 
        break;
    }
    
    default: {
       
        std::cerr << "[WARNING MapActionHandler " << __func__ << "] Type " <<
        pinchType << " is not a type to look for companions " << std::endl;    
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
        
        ROSEE::ActionGeneric::Ptr genericPointer =
            yamlWorker.parseYamlGeneric(pathFolder+file);
        generics.insert (std::make_pair( genericPointer->getName(), genericPointer) ) ;
    }
    
    
    return true;
    
    
}

bool ROSEE::MapActionHandler::parseAllTimeds(std::string pathFolder) {
    
    std::vector <std::string> filenames = ROSEE::Utils::getFilesInDir(pathFolder);
    YamlWorker yamlWorker;
    for (auto file : filenames) {
        
        std::shared_ptr < ROSEE::ActionTimed > timed = yamlWorker.parseYamlTimed(pathFolder+file);
        timeds.insert (std::make_pair( timed->getName(), timed) ) ;
    }
    
    return true;
}


bool ROSEE::MapActionHandler::insertSingleGeneric(ROSEE::ActionGeneric::Ptr generic) {
    
    auto it = generics.find(generic->getName());
    
    if (it != generics.end()){
       
         std::cerr << "[ERROR MapActionHandler " << __func__ << "] Trying to insert generic action with name " <<
            generic->getName() << "which already exists" << std::endl;
        
        return false;
    }
    
    //it as hint beause we already did the lookup in the find above
    generics.insert(it, std::make_pair(generic->getName(), generic));
    
    return true;
    
}




//******************************** PRIVATE FUNCT ********************************************************************/

void ROSEE::MapActionHandler::findPinchPairsMap() {
    
    //Assume only one pinch tight, it should be like this now.
    auto maps = getPrimitiveMap(ROSEE::ActionPrimitive::Type::PinchTight);
    
    if (maps.size() != 0 ){
        for (ActionPrimitiveMap map : maps) {
            for (auto mapEl : map) {
        
                for (auto fing : mapEl.first) { //.first is a set
                    
                    //we will insert all the set as value, this means that also will include the key itself,
                    // we remove the key from the values later
                    if (pinchTightPairsMap.count(fing) == 0 ) {
                        pinchTightPairsMap.insert(std::make_pair(fing, mapEl.first)); 
                    } else {
                        pinchTightPairsMap.at(fing).insert (mapEl.first.begin(), mapEl.first.end());
                    }
                }
            }
        }  
        //remove the string key from the values.
        for (auto it : pinchTightPairsMap) {
            it.second.erase(it.first);
        }
    }
    
    //now do the same for the loose pinches
    auto mapsloose = getPrimitiveMap(ROSEE::ActionPrimitive::Type::PinchLoose);
    
    if (mapsloose.size() != 0 ) {
        for (ActionPrimitiveMap map : mapsloose) {
            for (auto mapEl : map) {
        
                for (auto fing : mapEl.first) { //.first is a set
                    
                    //we will insert all the set as value, this means that also will include the key itself,
                    // we remove the key from the values later
                    if (pinchLoosePairsMap.count(fing) == 0 ) {
                        pinchLoosePairsMap.insert(std::make_pair(fing, mapEl.first)); 
                    } else {
                        pinchLoosePairsMap.at(fing).insert (mapEl.first.begin(), mapEl.first.end());
                    }
                }
            }
        }
        //remove the string key from the values.
        for (auto it : pinchLoosePairsMap) {
            it.second.erase(it.first);
        }
    }
}


