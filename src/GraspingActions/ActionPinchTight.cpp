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

#include <end_effector/GraspingActions/ActionPinchTight.h>

ROSEE::ActionPinchTight::ActionPinchTight() : 
    ActionPinchGeneric ("pinchTight", 2, 3, ActionPrimitive::Type::PinchTight) { }

ROSEE::ActionPinchTight::ActionPinchTight(unsigned int jointStateSetMaxSize) : 
    ActionPinchGeneric ("pinchTight", 2, jointStateSetMaxSize, ActionPrimitive::Type::PinchTight) { }

ROSEE::ActionPinchTight::ActionPinchTight (std::pair <std::string, std::string> fingerNamesPair, 
    JointPos jp, collision_detection::Contact cont) :
    ActionPinchGeneric ("pinchTight", 2, 3, ActionPrimitive::Type::PinchTight )  {

    //different from insertState, here we are sure the set is empty (we are in costructor)
    fingersInvolved.insert(fingerNamesPair.first);
    fingersInvolved.insert(fingerNamesPair.second);
    actionStates.insert (std::make_pair (jp, cont) );
}

ROSEE::ActionPinchTight::ActionPinchTight (std::string finger1, std::string finger2, 
    JointPos jp, collision_detection::Contact cont) :
    ActionPinchGeneric ("pinchTight", 2, 3, ActionPrimitive::Type::PinchTight )  {

    //different from insertState, here we are sure the set is empty (we are in costructor)
    fingersInvolved.insert(finger1);
    fingersInvolved.insert(finger2);
    actionStates.insert (std::make_pair (jp, cont) );
}

ROSEE::JointPos ROSEE::ActionPinchTight::getJointPos() const {
    return (actionStates.begin()->first);
}

ROSEE::JointPos ROSEE::ActionPinchTight::getJointPos( unsigned int index) const {
    auto it = actionStates.begin();
    unsigned int i = 1;
    while (i < index ) {
        ++ it;
    }
    return (it->first);
}

std::vector < ROSEE::JointPos > ROSEE::ActionPinchTight::getAllJointPos() const{
    
    std::vector < JointPos > retVect;
    retVect.reserve ( actionStates.size() );
    
    for (auto it : actionStates ) {
        retVect.push_back(it.first);
    }
    
    return retVect;
}


std::vector < ROSEE::ActionPinchTight::StateWithContact > ROSEE::ActionPinchTight::getActionStates () const {
    
    std::vector < ROSEE::ActionPinchTight::StateWithContact > retVect;
    retVect.reserve ( actionStates.size() );
    
    for (auto it : actionStates ) {
        retVect.push_back(it);
    }
    
    return retVect;
    
}

bool ROSEE::ActionPinchTight::insertActionState (ROSEE::JointPos jp, collision_detection::Contact cont) {

    auto pairRet = actionStates.insert ( std::make_pair (jp, cont) ) ;
    
    if (! pairRet.second ) {
        //TODO print error no insertion because depth is equal... very improbable
        return false;
    }
    
    if (actionStates.size() > maxStoredActionStates) { 
        //max capacity reached, we have to delete the last one
        auto it = pairRet.first;        
        
        if ( (++it) == actionStates.end() ){
           // the new inserted is the last one and has to be erased
            actionStates.erase(pairRet.first);
            return false;
        }
        
        // the new inserted is not the last one that has to be erased
        auto lastElem = actionStates.end();
        --lastElem;
        actionStates.erase(lastElem);
    }
    
    return true;
}


void ROSEE::ActionPinchTight::print () const {
    
    std::stringstream output;
    output << "ActionName: " << name << std::endl;
    
    output << "FingersInvolved: [";
    for (auto fingName : fingersInvolved){
        output << fingName << ", " ;
    }
    output.seekp (-2, output.cur); //to remove the last comma (and space)
    output << "]" << std::endl;
    
    output << "JointsInvolvedCount: " << std::endl;;
    output << jointsInvolvedCount << std::endl;
    
    unsigned int nActState = 1;
    for (auto itemSet : actionStates) {  //the element in the set
        output << "Action_State_" << nActState << " :" << std::endl;

        output << "\t" << "JointStates:" << std::endl;
        output << itemSet.first;
        output << "\t" << "MoveitContact:" << std::endl;
        output << "\t\tbody_name_1: " << itemSet.second.body_name_1 << std::endl;
        output << "\t\tbody_name_2: " << itemSet.second.body_name_2 << std::endl;
        output << "\t\tbody_type_1: " << itemSet.second.body_type_1 << std::endl;
        output << "\t\tbody_type_2: " << itemSet.second.body_type_2 << std::endl;
        output << "\t\tdepth: " << itemSet.second.depth << std::endl;
        output << "\t\tnormal: " << "["<< itemSet.second.normal.x() << ", " 
            << itemSet.second.normal.y() << ", " << itemSet.second.normal.z() << "]" << std::endl;
        output << "\t\tpos: " << "["<< itemSet.second.pos.x() << ", " 
            << itemSet.second.pos.y() << ", " << itemSet.second.pos.z() << "]" << std::endl;
            
        nActState++;
    }
    output << std::endl;
    
    std::cout << output.str();

}


void ROSEE::ActionPinchTight::emitYaml ( YAML::Emitter& out ) const {
    
    out << YAML::Key << YAML::Flow << fingersInvolved;
    
    unsigned int nCont = 1;
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "PrimitiveType" << YAML::Value << primitiveType;
    out << YAML::Key << "ActionName" << YAML::Value << name;
    out << YAML::Key << "JointsInvolvedCount" << YAML::Value << YAML::BeginMap;
    for (const auto &jointCount : jointsInvolvedCount ) {
        out << YAML::Key << jointCount.first;
        out << YAML::Value << jointCount.second;
    }
    out << YAML::EndMap;
    
    for (const auto & actionState : actionStates) { //.second is the set of ActionState
        
        std::string contSeq = "ActionState_" + std::to_string(nCont);
        out << YAML::Key << contSeq; 
        
        out << YAML::Value << YAML::BeginMap;
            //actionState.first, the jointstates map
            out << YAML::Key << "JointPos" << YAML::Value << YAML::BeginMap;
            for (const auto &joint : actionState.first) {
                out << YAML::Key << joint.first;
                out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
            }
            out << YAML::EndMap;
            
            //actionState.second, the optional
            out << YAML::Key << "Optional" << YAML::Value;
            emitYamlForContact ( actionState.second, out );
            
        out << YAML::EndMap;
        nCont++;
    }
    out << YAML::EndMap;

}


bool ROSEE::ActionPinchTight::fillFromYaml ( YAML::const_iterator yamlIt ) {
        
    std::vector <std::string> fingInvolvedVect = yamlIt->first.as <std::vector < std::string >> ();
    for (const auto &it : fingInvolvedVect) {
        fingersInvolved.insert(it);
    }

    for ( YAML::const_iterator keyValue = yamlIt->second.begin(); keyValue != yamlIt->second.end(); ++keyValue) {        
        
        std::string key = keyValue->first.as<std::string>();
        if (key.compare("JointsInvolvedCount") == 0) {
            jointsInvolvedCount = keyValue->second.as < JointsInvolvedCount > ();
            
        } else if (key.compare ("ActionName") == 0 ) {
            name = keyValue->second.as <std::string> ();
        
        } else if (key.compare ("PrimitiveType") == 0) {
            ROSEE::ActionPrimitive::Type parsedType = static_cast<ROSEE::ActionPrimitive::Type> ( 
                keyValue->second.as <unsigned int>() );
            if (parsedType != primitiveType ) {
                std::cerr << "[ERROR ActionPinchTight::" << __func__ << " parsed a type " << parsedType << 
                    " but this object has primitive type " << primitiveType << std::endl; 
                return false;
            }
            
        } else if (key.compare(0, 12, "ActionState_") == 0) { //compare 12 caracters from index 0 of key

            JointPos jointPos;
            collision_detection::Contact contact;
            for(YAML::const_iterator asEl = keyValue->second.begin(); asEl != keyValue->second.end(); ++asEl) {

                //asEl can be the map JointPos or the map Optional
                if (asEl->first.as<std::string>().compare ("JointPos") == 0 ) {
                    jointPos = asEl->second.as < JointPos >(); 
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
                    std::cerr << "[ERROR ActionPinchTight::" << __func__ << "not know key " 
                        << asEl->first.as<std::string>() << 
                        " found in the yaml file at this level" << std::endl; 
                    return false;
                }
            }  
            actionStates.insert ( std::make_pair (jointPos, contact));
            
        } else {
            std::cerr << "[ERROR ActionPinchTight::" << __func__ << "not know key " << key << 
                " found in the yaml file" << std::endl; 
            return false;
        }
    }
    
    return true;
}

bool ROSEE::ActionPinchTight::emitYamlForContact (collision_detection::Contact moveitContact, YAML::Emitter& out) const {

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
