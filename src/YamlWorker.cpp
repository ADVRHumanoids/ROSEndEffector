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

#include <end_effector/YamlWorker.h>

ROSEE::YamlWorker::YamlWorker ( ) {}

std::string ROSEE::YamlWorker::createYamlFile( const ROSEE::Action* action, std::string pathFolder) {
    
    ROSEE::Utils::create_directory ( pathFolder );
    std::string output = emitYaml ( action );
    ROSEE::Utils::out2file(pathFolder + action->getName() + ".yaml", output);
    return (pathFolder + action->getName() + ".yaml");
    
}

std::string ROSEE::YamlWorker::createYamlFile( const ROSEE::ActionGeneric::Ptr action, std::string pathFolder) {
    
    ROSEE::Utils::create_directory ( pathFolder );
    std::string output = emitYaml ( action );
    ROSEE::Utils::out2file(pathFolder + action->getName() + ".yaml", output);
    return (pathFolder + action->getName() + ".yaml");
    
}

std::string ROSEE::YamlWorker::createYamlFile(
    const std::map < std::set <std::string> , ActionPrimitive* > mapOfActions,
    const std::string actionName, std::string pathFolder) {
    
    ROSEE::Utils::create_directory ( pathFolder );
    std::string output = emitYaml ( mapOfActions );
    ROSEE::Utils::out2file( pathFolder + actionName + ".yaml", output);
    return (pathFolder + actionName + ".yaml");
    
    
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

std::string ROSEE::YamlWorker::emitYaml  ( const ROSEE::Action* action ) {
    
    YAML::Emitter out;
    action->emitYaml(out);
    out << YAML::Newline << YAML::Newline; //double to insert a blanck line
    return out.c_str();
    
}

std::string ROSEE::YamlWorker::emitYaml  ( const ROSEE::ActionGeneric::Ptr action ) {
    
    YAML::Emitter out;
    action->emitYaml(out);
    out << YAML::Newline << YAML::Newline; //double to insert a blanck line
    return out.c_str();
    
}

std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> ROSEE::YamlWorker::parseYamlPrimitive(
        std::string fileWithPath, ROSEE::ActionPrimitive::Type actionType) {
    
    std::map < std::set < std::string>, ROSEE::ActionPrimitive::Ptr > parsedMap; 

    //TODO check elsewhere if file exist or not?
    std::ifstream ifile(fileWithPath);
    if (! ifile) {
        std::cout << "[ERROR YAMLPARSER:: " << __func__ << "]: file " << fileWithPath << " not found. "  << std::endl;
            return parsedMap;
    }
    
    YAML::Node node = YAML::LoadFile(fileWithPath);
    
    for(YAML::const_iterator it4Action = node.begin(); it4Action != node.end(); ++it4Action) {
        ActionPrimitive::Ptr ptr;
        switch (actionType) {
        case ActionPrimitive::Type::PinchTight: {
            ptr = std::make_shared <ActionPinchTight>();
            break;
        }
        case ActionPrimitive::Type::PinchLoose: {
            ptr = std::make_shared <ActionPinchLoose> ();
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
        case ActionPrimitive::Type::SingleJointMultipleTips: {
            ptr = std::make_shared <ActionSingleJointMultipleTips>();
            break;
        }
        case ActionPrimitive::Type::MultiplePinchTight: {
            ptr = std::make_shared <ActionMultiplePinchTight> ();
            break;
        }

        default : {
            std::cout << "[ERROR YAMLPARSER]: " << actionType << " : type not found" << std::endl;
        }
        }
        
        ptr->fillFromYaml ( it4Action );
        
        parsedMap.insert ( std::make_pair ( ptr->getKeyElements(), ptr) );
    }
        
    return parsedMap;
}


std::map < std::set < std::string>, ROSEE::ActionPrimitive::Ptr > ROSEE::YamlWorker::parseYamlPrimitive (std::string fileWithPath) {
    
    std::map < std::set < std::string>, ROSEE::ActionPrimitive::Ptr > parsedMap; 

    //TODO check elsewhere if file exist or not?
    std::ifstream ifile(fileWithPath);
    if (! ifile) {
        std::cout << "[ERROR YAMLPARSER:: " << __func__ << "]: file " << fileWithPath << " not found. "  << std::endl;
            return parsedMap;
    }
    
    YAML::Node node = YAML::LoadFile(fileWithPath);

    for(YAML::const_iterator it4Action = node.begin(); it4Action != node.end(); ++it4Action) {
        
        std::string actionName;
        ROSEE::ActionPrimitive::Type type = ROSEE::ActionPrimitive::Type::None;
        
        //now look for the type of the action and for the name
        for ( YAML::const_iterator actionState = it4Action->second.begin(); actionState != it4Action->second.end(); ++actionState) {        
        
            if (actionState->first.as<std::string>().compare("PrimitiveType") == 0) {
                type = static_cast<ROSEE::ActionPrimitive::Type> ( actionState->second.as < int > () );
                break; //we only need the type
            }
        }
        
        //now use emit of specific action
        ActionPrimitive::Ptr ptr;
        switch (type) {
        case ActionPrimitive::Type::PinchTight: {
            ptr = std::make_shared <ActionPinchTight>();
            break;
        }
        case ActionPrimitive::Type::PinchLoose: {
            ptr = std::make_shared <ActionPinchLoose> ();
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
        case ActionPrimitive::Type::SingleJointMultipleTips: {
            ptr = std::make_shared <ActionSingleJointMultipleTips>();
            break;
        }
        case ActionPrimitive::Type::MultiplePinchTight: {
            ptr = std::make_shared <ActionMultiplePinchTight> ();
            break;
        }

        default : {
            std::cout << "[ERROR YAMLPARSER]: " << type << " : type not found" << std::endl;
        }
        }
        
        ptr->fillFromYaml ( it4Action );
        
        parsedMap.insert ( std::make_pair ( ptr->getKeyElements(), ptr) );
    }
        
    return parsedMap;
    
}


ROSEE::ActionComposed ROSEE::YamlWorker::parseYamlComposed (std::string fileWithPath){
    
    ROSEE::ActionComposed parsedAction; 

    //TODO check elsewhere if file exist or not?
    std::ifstream ifile(fileWithPath);
    if (! ifile) {
        std::cout << "[ERROR YAMLPARSER:: " << __func__ << "]: file " << fileWithPath << " not found. "  << std::endl;
            return parsedAction;
    }
    
    YAML::Node node = YAML::LoadFile(fileWithPath);
    YAML::const_iterator yamlIt = node.begin();

    parsedAction.fillFromYaml ( yamlIt );
    
    return parsedAction;
    
}

ROSEE::ActionGeneric::Ptr ROSEE::YamlWorker::parseYamlGeneric(std::string fileWithPath) {
    
    ActionGeneric::Ptr ptrAction;
    
    //TODO check elsewhere if file exist or not?
    std::ifstream ifile(fileWithPath);
    if (! ifile) {
        std::cout << "[ERROR YAMLPARSER:: " << __func__ << "]: file " << fileWithPath << " not found. "  << std::endl;
        return ptrAction;
    }
    
    YAML::Node node = YAML::LoadFile(fileWithPath);
    
    YAML::const_iterator it4Action = node.begin(); //for not primitive, only one action is inside the file
        
    ROSEE::Action::Type type = ROSEE::Action::Type::None;
    
    //now look for the type of the action
    for ( YAML::const_iterator el = it4Action->second.begin(); el != it4Action->second.end(); ++el) {        
    
        if (el->first.as<std::string>().compare("Type") == 0) {
            type = static_cast<ROSEE::Action::Type> ( el->second.as < int > () );
            break; //we only need the type
        }
    }
    
    switch (type) {
    case Action::Type::Primitive : {
        std::cout << "[ERROR YAMLPARSER:: " << __func__ << "]: file " << fileWithPath << " contains a primitive action, "
        << " please use parseYamlPrimitive to parse it " << std::endl;
        return ptrAction;
        break;
    }
    case Action::Type::Generic : {
        ptrAction = std::make_shared <ActionGeneric> ();
        break;
    }
    case Action::Type::Composed : {
        ptrAction = std::make_shared <ActionComposed> ();
        break;
    }
    case Action::Type::Timed : {
        std::cout << "[ERROR YAMLPARSER:: " << __func__ << "]: file " << fileWithPath << " contains a timed action, "
        << " please use parseYamlTimed to parse it " << std::endl;
        return ptrAction;
        break;
    }
    default : {
        std::cout << "[ERROR YAMLPARSER:: " << __func__ << "]: file " << fileWithPath << " contains an action of not know type "
        << type << std::endl;
        return ptrAction; 
    }
    }

    ptrAction->fillFromYaml(it4Action);
    
    return ptrAction;
    
    
}


std::shared_ptr < ROSEE::ActionTimed > ROSEE::YamlWorker::parseYamlTimed (std::string fileWithPath){
    
    std::shared_ptr < ROSEE::ActionTimed > ptrAction; 

    //TODO check elsewhere if file exist or not?
    std::ifstream ifile(fileWithPath);
    if (! ifile) {
        std::cout << "[ERROR YAMLPARSER:: " << __func__ << "]: file " << fileWithPath << " not found. "  << std::endl;
            return nullptr;
    }
    
    YAML::Node node = YAML::LoadFile(fileWithPath);
    YAML::const_iterator yamlIt = node.begin();
    ptrAction = std::make_shared <ActionTimed> ();

    ptrAction->fillFromYaml ( yamlIt );
    
    return ptrAction;
    
}
