/*
 * Copyright (C) 2020 IIT-HHCM
 * Author: Davide Torielli
 * email:  davide.torielli@iit.it
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

#include <ROSEndEffector/ActionPinchStrong.h>

ROSEE::ActionPinchStrong::ActionPinchStrong() : 
    ActionPinchGeneric ("pinchStrong", 2, 3, ActionType::PinchStrong) { }

ROSEE::ActionPinchStrong::ActionPinchStrong(unsigned int jointStateSetMaxSize) : 
    ActionPinchGeneric ("pinchStrong", 2, jointStateSetMaxSize, ActionType::PinchStrong) { }

ROSEE::ActionPinchStrong::ActionPinchStrong (std::pair <std::string, std::string> tipNames, 
    JointStates js, collision_detection::Contact cont) :
    ActionPinchGeneric ("pinchStrong", 2, 3, ActionType::PinchStrong )  {

    //different from insertState, here we are sure the set is empty (we are in costructor)
    this->tipsPair = tipNames;
    statesInfoSet.insert (std::make_pair (js, cont) );
}


std::vector < ROSEE::JointStates > ROSEE::ActionPinchStrong::getActionStates() const{
    
    std::vector < JointStates > retVect;
    retVect.reserve(statesInfoSet.size());
    
    for (auto it : statesInfoSet ) {
        retVect.push_back(it.first);
    }
    
    return retVect;
}


std::vector < ROSEE::ActionPinchStrong::StateWithContact > ROSEE::ActionPinchStrong::getActionStatesWithContact() const {
    
    std::vector < ROSEE::ActionPinchStrong::StateWithContact > retVect;
    retVect.reserve ( statesInfoSet.size() );
    
    for (auto it : statesInfoSet ) {
        retVect.push_back(it);
    }
    
    return retVect;
    
}


bool ROSEE::ActionPinchStrong::setActionStates (std::vector < ROSEE::JointStates > jsVect) {
    
    collision_detection::Contact cont;
    cont.depth = 0; //we need to initialize it because it is used in the set comparator
    for (auto it : jsVect) {
        if (! insertActionState (it, cont)) {
            return false;
        }
    }
    return true;
}

bool ROSEE::ActionPinchStrong::insertActionState (ROSEE::JointStates js, collision_detection::Contact cont) {

    auto pairRet = statesInfoSet.insert ( std::make_pair (js, cont) ) ;
    
    if (! pairRet.second ) {
        //no insertion, some error
        return false;
    }
    
    if (statesInfoSet.size() > jointStateSetMaxSize) { 
        //max capacity reached, we have to delete the last one
        auto it = pairRet.first;        
        
        if (++(it) == statesInfoSet.end() ){
           // the new inserted is the last one and has to be erased
            statesInfoSet.erase(pairRet.first);
            return false;
        }
        
        // the new inserted is not the last one that has to be erased
        auto lastElem = statesInfoSet.end();
        --lastElem;
        statesInfoSet.erase(lastElem);
    }
    
    return true;
}


void ROSEE::ActionPinchStrong::printAction () const {
    
    std::stringstream output;
    
    output << tipsPair.first << ", " << tipsPair.second << std::endl ;
    
    unsigned int nActState = 1;
    for (auto itemSet : statesInfoSet) {  //the element in the set
        output << "\tAction_State_" << nActState << " :" << std::endl;

        output << "\t\t" << "JointStates:" << std::endl;
        output << itemSet.first;
        output << "\t\t" << "MoveitContact:" << std::endl;
        output << "\t\t\tbody_name_1: " << itemSet.second.body_name_1 << std::endl;
        output << "\t\t\tbody_name_2: " << itemSet.second.body_name_2 << std::endl;
        output << "\t\t\tbody_type_1: " << itemSet.second.body_type_1 << std::endl;
        output << "\t\t\tbody_type_2: " << itemSet.second.body_type_2 << std::endl;
        output << "\t\t\tdepth: " << itemSet.second.depth << std::endl;
        output << "\t\t\tnormal: " << "["<< itemSet.second.normal.x() << ", " 
            << itemSet.second.normal.y() << ", " << itemSet.second.normal.z() << "]" << std::endl;
        output << "\t\t\tpos: " << "["<< itemSet.second.pos.x() << ", " 
            << itemSet.second.pos.y() << ", " << itemSet.second.pos.z() << "]" << std::endl;
            
        nActState++;
    }
    output << std::endl;
    
    std::cout << output.str();

}

bool ROSEE::ActionPinchStrong::emitYamlForContact (collision_detection::Contact moveitContact, YAML::Emitter& out) {

    out << YAML::BeginMap;
        out << YAML::Key << "MoveItContact" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "body_name_1";
            out << YAML::Value << moveitContact.body_name_1;
            out << YAML::Key << "body_name_2";
            out << YAML::Value << moveitContact.body_name_2;
            out << YAML::Key << "body_type_1";
            out << YAML::Value << moveitContact.body_type_1;
            out << YAML::Key << "body_type_2";
            out << YAML::Value << moveitContact.body_type_2;
            out << YAML::Key << "depth";
            out << YAML::Value << moveitContact.depth;
            out << YAML::Key << "normal";
            std::vector < double > normal ( moveitContact.normal.data(), moveitContact.normal.data() +  moveitContact.normal.rows());  
            out << YAML::Value << YAML::Flow << normal;
            out << YAML::Key << "pos";
            std::vector < double > pos ( moveitContact.pos.data(), moveitContact.pos.data() +  moveitContact.pos.rows());
            out << YAML::Value << YAML::Flow << pos;
        out << YAML::EndMap;
    out << YAML::EndMap;
    
    return true;
}

void ROSEE::ActionPinchStrong::emitYaml ( YAML::Emitter& out ) {
    
    // YAML << not valid for pair, we have to "convert" into vector
    std::vector <std::string> vectKeys {tipsPair.first, tipsPair.second};
    out << YAML::Key << YAML::Flow << vectKeys;
    
    unsigned int nCont = 1;
    out << YAML::Value << YAML::BeginMap;
    for (const auto & actionState : statesInfoSet) { //.second is the set of ActionState
        
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
            emitYamlForContact(actionState.second, out);
            
        out << YAML::EndMap;
        nCont++;
    }
    out << YAML::EndMap;

}


bool ROSEE::ActionPinchStrong::fillFromYaml ( YAML::const_iterator yamlIt ) {
        
    tipsPair = yamlIt->first.as<std::pair < std::string, std::string >> ();

    for ( YAML::const_iterator actionState = yamlIt->second.begin(); actionState != yamlIt->second.end(); ++actionState) {        
        // actionState->first == ActionState_x

        JointStates jointStates;
        collision_detection::Contact contact;
        for(YAML::const_iterator asEl = actionState->second.begin(); asEl != actionState->second.end(); ++asEl) {

            //asEl can be the map JointStates or the map Optional
            if (asEl->first.as<std::string>().compare ("JointStates") == 0 ) {
                jointStates = asEl->second.as < JointStates >(); 
            } else if (asEl->first.as<std::string>().compare ("Optional") == 0 ) {
                
                YAML::Node cont =  asEl->second["MoveItContact"];
                contact.body_name_1 = cont["body_name_1"].as < std::string >();
                contact.body_name_2 = cont["body_name_2"].as < std::string >();
                switch (cont["body_type_1"].as < int >())
                {
                case 0:
                    contact.body_type_1 = collision_detection::BodyType::ROBOT_LINK;
                    break;
                case 1:
                    contact.body_type_1 = collision_detection::BodyType::ROBOT_ATTACHED;
                    break;
                case 2:
                    contact.body_type_1 = collision_detection::BodyType::WORLD_OBJECT;
                    break;
                default:
                    std::cout << "some error, body_type_1" << cont["body_type_1"].as < int >()
                        << "unknown" << std::endl;
                    contact.body_type_1 = collision_detection::BodyType::WORLD_OBJECT;
                }
                switch (cont["body_type_2"].as < int >())
                {
                case 0:
                    contact.body_type_2 = collision_detection::BodyType::ROBOT_LINK;
                    break;
                case 1:
                    contact.body_type_2 = collision_detection::BodyType::ROBOT_ATTACHED;
                    break;
                case 2:
                    contact.body_type_2 = collision_detection::BodyType::WORLD_OBJECT;
                    break;
                default:
                    std::cout << "some error, body_type_2" << cont["body_type_2"].as < int >()
                        << "unknown" << std::endl;
                    contact.body_type_2 = collision_detection::BodyType::WORLD_OBJECT;
                }
                contact.depth = cont["depth"].as<double>();
                std::vector < double > normVect = cont["normal"].as < std::vector <double> >();
                std::vector < double > posVect = cont["pos"].as < std::vector <double> >();
                contact.normal = Eigen::Vector3d (normVect.data() );
                contact.pos = Eigen::Vector3d (posVect.data() );
                
            } else {
                //ERRROr, only joinstates and optional at this level
                return false;
            }
        }  
        statesInfoSet.insert ( std::make_pair (jointStates, contact));
    }
    
    return true;
}


