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

#include <end_effector/GraspingActions/ActionComposed.h>

ROSEE::ActionComposed::ActionComposed() : ActionGeneric() {
    independent = true;
    nInnerActions = 0;
    type = Action::Type::Composed;
}

ROSEE::ActionComposed::ActionComposed(std::string name) : ActionGeneric(name) {
    independent = true;
    nInnerActions = 0;
    type = Action::Type::Composed;
}

ROSEE::ActionComposed::ActionComposed(std::string name, bool independent) : ActionGeneric(name) {
    this->independent = independent;
    nInnerActions = 0;
    type = Action::Type::Composed;
}

unsigned int ROSEE::ActionComposed::numberOfInnerActions() const {
    return nInnerActions;
}

bool ROSEE::ActionComposed::isIndependent() const {
    return independent;
}

std::vector<std::string> ROSEE::ActionComposed::getInnerActionsNames() const {
    return innerActionsNames;
}

bool ROSEE::ActionComposed::empty() {
    return (nInnerActions == 0);
}


bool ROSEE::ActionComposed::sumAction ( ROSEE::Action::Ptr action, double jointPosScaleFactor, unsigned int jointPosIndex )
{
    
    if ( ! checkIndependency(action) ) {
        return false; //cant add this primitive
    }
    
    if ( jointPosIndex > action->getAllJointPos().size()-1 ) {
        std::cerr << "[ACTIONCOMPOSED:: " << __func__ << "] The given jointPosindex " << jointPosIndex  
            << " exceed the number  " << action->getAllJointPos().size() << " of jointpos of passed action" << std::endl;
        return false;
    }
    
    if ( nInnerActions > 0 && 
        (! ROSEE::Utils::keys_equal(action->getAllJointPos().at(jointPosIndex), jointPos)) ) {
        std::cerr << "[ACTIONCOMPOSED:: " << __func__ << "] The action passed as argument has different keys in jointPosmap" 
                  << " respect to the others inserted in this composed action " << std::endl;
        return false;
    }
    
    if (jointPosScaleFactor < 0) {
        std::cerr << "[ACTIONCOMPOSED:: " << __func__ << "] You can not scale the joint position of the action to be inserted by a " 
                  <<  "value less than 0; jointPosScaleFactor passed is: " << jointPosScaleFactor << std::endl;
        return false;
    } 
    
    if (jointPosScaleFactor > 1) {
        std::wcerr << "[ACTIONCOMPOSED:: " << __func__ << "] WARNING, You are scaling with a value greater than 1 " 
                   << " this could cause to command position over the joint limits;  jointPosScaleFactor passed is: " << jointPosScaleFactor << std::endl;
    } 

    JointPos actionJP = action->getAllJointPos().at(jointPosIndex) * jointPosScaleFactor;
    JointsInvolvedCount actionJIC = action->getJointsInvolvedCount();
        
    if (nInnerActions == 0) { //first primitive inserted
        
        for (auto joint : actionJP ){
            jointsInvolvedCount.insert ( std::make_pair (joint.first, actionJIC.at(joint.first) ) );
            jointPos.insert ( std::make_pair (joint.first, joint.second) );
        }
        
    } else {
        
        if (independent) {

            //if here, action is independent, we can add the joints states
            for ( auto joint : actionJP ){ 
                
                if ( actionJIC.at (joint.first ) > 0 ) {
                    
                    jointPos.at(joint.first) = joint.second;
                    jointsInvolvedCount.at(joint.first) += actionJIC.at (joint.first );
                    //+= or = is the same for the checks done before 
                } 
            }
            
        } else {
            // for each joint, add the state's value to the mean 
            // (number of element in the previous mean is given by jointsInvolvedCount.at(x))

            for ( auto joint : actionJP ) { 
                if ( actionJIC.at( joint.first ) == 0 ) {
                    continue; //if the action that is being added has this joint not setted, not consider it
                }
                
                //update the count 
                jointsInvolvedCount.at(joint.first) += actionJIC.at (joint.first ); 
                
                //iterate all dofs of jointPos
                for (unsigned int dof = 0; dof < joint.second.size(); dof++ ) {
                    
                    double mean = jointPos.at( joint.first ).at(dof) + 
                        ( (joint.second.at(dof) - jointPos.at(joint.first).at(dof)) / 
                            jointsInvolvedCount.at(joint.first) );
                        
                    jointPos.at(joint.first).at(dof) = mean;     
                }
            }
        }
    }
    
    innerActionsNames.push_back ( action->getName() );
    for ( auto it: action->getFingersInvolved() ) {
         fingersInvolved.insert ( it );
    }

    nInnerActions ++;
    
    return true;
}

bool ROSEE::ActionComposed::checkIndependency ( ROSEE::Action::Ptr action ) {
    
    if (!independent) {
        
    } else if (nInnerActions == 0 ) {
        
        for ( auto jic : action->getJointsInvolvedCount() ){
            if ( jic.second  > 1  ) {
                // if action is dependent we check that all its joints are setted only once
                // so we can teoretically add a "dipendent" action if all its joints are setted once
                return false; //cant add this primitive
            }
        }
        
    } else {   
    
        for ( auto jic : action->getJointsInvolvedCount() ){
            if ( jic.second + jointsInvolvedCount.at(jic.first) > 1  ) {
                // we use the sum so also if action is dependent we check that all its joints are setted once
                // so we can teoretically add a "dipendent" action if all its joints are setted once
                return false; //cant add this primitive
            }
        }
    }
    
    return true;
}


void ROSEE::ActionComposed::print () const {
    
    std::stringstream output;
    
    output << "Composed Action '" << name;
    independent ? output << "' (independent):" : output << "' (not independent):" ;
    output << std::endl;
    
    output << "Composed by " << nInnerActions << " inner action: [" ;
    for (auto it : innerActionsNames) {
        output << it << ", ";
    }
    output.seekp (-2, output.cur); //to remove the last comma (and space)
    output << "]" << std::endl;
    
    output << "Fingers involved: [" ;
    for (auto it : fingersInvolved) {
        output << it << ", ";
    }
    output.seekp (-2, output.cur); //to remove the last comma (and space)
    output << "]" << std::endl;
    
    output << "Each joint influenced by x inner action:" << std::endl;
    output << jointsInvolvedCount;

    output << "JointPos:" << std::endl;
    output << jointPos << std::endl;
    
    std::cout << output.str();
}


void ROSEE::ActionComposed::emitYaml ( YAML::Emitter& out) const {
    
    out << YAML::BeginMap << YAML::Key << name << YAML::Value << YAML::BeginMap ;
        out << YAML::Key << "Type" << YAML::Value << type;
        out << YAML::Key << "Independent" << YAML::Value << independent;
        out << YAML::Key << "NInnerActions" << YAML::Value << nInnerActions;
        out << YAML::Key << "InnerActionsNames" << YAML::Value << YAML::Flow << innerActionsNames;
        out << YAML::Key << "FingersInvolved" << YAML::Value << YAML::Flow << fingersInvolved;
        out << YAML::Key << "JointsInvolvedCount" << YAML::Value << YAML::BeginMap;
        for (const auto &jointCount : jointsInvolvedCount ) {
            out << YAML::Key << jointCount.first;
            out << YAML::Value << jointCount.second;
        } 
        out << YAML::EndMap;

        out << YAML::Key << "JointPos" << YAML::Value << YAML::BeginMap;
            for (const auto &joint : jointPos) {
                out << YAML::Key << joint.first;
                out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
            }
        out << YAML::EndMap;
    out << YAML::EndMap;
    out << YAML::EndMap;
}

// if parsed, we dont have the link with the primitives... so primitiveObjects is empty
bool ROSEE::ActionComposed::fillFromYaml ( YAML::const_iterator yamlIt ) {
    
    name = yamlIt->first.as<std::string>();
            
    for (auto keyValue = yamlIt->second.begin(); keyValue != yamlIt->second.end(); ++keyValue ) {

        std::string key = keyValue->first.as<std::string>();

        if ( key.compare ("Independent") == 0 ) {
            independent = keyValue->second.as<bool>();
            
        } else if ( key.compare ("NInnerActions") == 0 ) {
            nInnerActions = keyValue->second.as <unsigned int>();
            
        } else if ( key.compare ("Type") == 0 ) {
            if (ROSEE::Action::Type::Composed != static_cast<ROSEE::Action::Type> ( keyValue->second.as <unsigned int>() )) {
                std::cout << "[COMPOSED ACTION::" << __func__ << "] Error, found type  " << keyValue->second.as <unsigned int>()
                << "instead of Composed type (" << ROSEE::Action::Type::Composed << ")" << std::endl;
                return false;
            }
            type = ROSEE::Action::Type::Composed;
            
        } else if ( key.compare ("InnerActionsNames") == 0 ) {
            innerActionsNames = keyValue->second.as <std::vector <std::string> >();
        
        } else if ( key.compare ("FingersInvolved") == 0 ) { 
            auto tempVect = keyValue->second.as <std::vector <std::string> > ();
            fingersInvolved.insert ( tempVect.begin(), tempVect.end() );
            
        } else if ( key.compare ("JointsInvolvedCount") == 0 ) {
            jointsInvolvedCount = keyValue->second.as < JointsInvolvedCount >(); 
            
        } else if ( key.compare ("JointPos") == 0 ) {
            jointPos = keyValue->second.as < JointPos >();
            
        } else {
            std::cout << "[COMPOSED ACTION PARSER] Error, not known key " << key << std::endl;
            return false;
        }
    } 
    return true;
}



