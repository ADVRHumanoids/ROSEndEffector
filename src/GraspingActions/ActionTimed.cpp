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

#include <end_effector/GraspingActions/ActionTimed.h>

ROSEE::ActionTimed::ActionTimed() {
    type = Action::Type::Timed;
}

ROSEE::ActionTimed::ActionTimed (std::string name ) : Action(name, Action::Type::Timed) {
    
}

ROSEE::JointPos ROSEE::ActionTimed::getJointPos() const {
    return jointPosFinal;
}

std::vector<ROSEE::JointPos> ROSEE::ActionTimed::getAllJointPos() const {
    
    std::vector<ROSEE::JointPos> jpVect;
    jpVect.reserve (actionsNamesOrdered.size());
    for (auto actName : actionsNamesOrdered) {
        jpVect.push_back( actionsJointPosMap.at (actName) );
    }
    return jpVect;
}

std::vector<ROSEE::JointsInvolvedCount> ROSEE::ActionTimed::getAllJointCountAction() const {
    
    std::vector<ROSEE::JointsInvolvedCount> jcVect;
    jcVect.reserve (actionsNamesOrdered.size());
    for (auto actName : actionsNamesOrdered) {
        jcVect.push_back( actionsJointCountMap.at (actName) );
    }
    return jcVect;
}

std::vector<std::pair<double, double> > ROSEE::ActionTimed::getAllActionMargins() const {
    
    std::vector <std::pair <double,double> > timeVect;
    timeVect.reserve (actionsNamesOrdered.size());
    for (auto actName : actionsNamesOrdered) {
        timeVect.push_back( actionsTimeMarginsMap.at (actName) );
    }
    return timeVect;
}

ROSEE::JointPos ROSEE::ActionTimed::getJointPosAction (std::string actionName) const {
    
    auto it = actionsJointPosMap.find(actionName);
    
    if ( it != actionsJointPosMap.end() ) {
        return (it->second);
        
    } else {
        std::cerr << "[ACTIONTIMED:: " << __func__ << "] ERROR: action " << actionName << " not present in this composed timed action" << std::endl;
        return ROSEE::JointPos();
    }
}

ROSEE::JointsInvolvedCount ROSEE::ActionTimed::getJointCountAction(std::string actionName) const {
    
    auto it = actionsJointCountMap.find(actionName);
    
    if ( it != actionsJointCountMap.end() ) {
        return (it->second);
        
    } else {
        std::cerr << "[ACTIONTIMED:: " << __func__ << "] ERROR: action " << actionName << " not present in this composed timed action" << std::endl;
        return ROSEE::JointsInvolvedCount();
    } 
}


std::pair <double, double> ROSEE::ActionTimed::getActionMargins ( std::string actionName ) const {
 
    auto it = actionsTimeMarginsMap.find(actionName);
    
    if ( it != actionsTimeMarginsMap.end() ) {
        return ( it->second );
        
    } else {
        std::cerr << "[ACTIONTIMED:: " << __func__ << "] ERROR: action " << actionName << " not present in this composed timed action" << std::endl;
        return std::make_pair(-1, -1);
    }
}

std::vector<std::string> ROSEE::ActionTimed::getInnerActionsNames() const {
    return actionsNamesOrdered;
}


void ROSEE::ActionTimed::print() const {
    
    std::stringstream output;
    
    output << "Timed Action '" << name << "'" << std::endl;
    
    output << "\tNice TimeLine:" << std::endl << "\t\t";
    for ( auto it : actionsNamesOrdered )  {
        output << actionsTimeMarginsMap.at(it).first << " -----> " << it << " -----> " << actionsTimeMarginsMap.at(it).second << " + ";
    }
    output.seekp (-3, output.cur); //to remove the last  --+--
    output << std::endl;
    
    output << "\tJointPos of each actions, in order of execution:\n";
    for ( auto actionName : actionsNamesOrdered )  {
        output << "\t" << actionName << std::endl;
        output << actionsJointPosMap.at(actionName) << std::endl;
    }
    
    output << "\tJointPos final, the sum of all joint pos of each inner action:\n";
    output << jointPosFinal << std::endl;
    
    output << "\tFingers involved: [" ;
    for (auto it : fingersInvolved) {
        output << it << ", ";
    }
    output.seekp (-2, output.cur); //to remove the last comma (and space)
    output << "]" << std::endl;
    
    output << "\tEach joint influenced by x inner action:" << std::endl;
    output << jointsInvolvedCount << std::endl;
    
    std::cout << output.str();
    
}

void ROSEE::ActionTimed::emitYaml(YAML::Emitter& out) const {
    
    out << YAML::BeginMap << YAML::Key << name << YAML::Value << YAML::BeginMap ;
        std::string timeline;
        for ( auto it : actionsNamesOrdered )  {
            timeline += (std::to_string(actionsTimeMarginsMap.at(it).first) + " -----> " + 
                it + " -----> " + std::to_string(actionsTimeMarginsMap.at(it).second) + " + ");
        }
        if (! timeline.empty() ) { 
            timeline.pop_back(); timeline.pop_back(); timeline.pop_back(); //to delete last " + " 
        }
        
        out << YAML::Comment(timeline);
        out << YAML::Key << "Type" << YAML::Value << type;
        out << YAML::Key << "FingersInvolved" << YAML::Value << YAML::Flow << fingersInvolved;
        
        out << YAML::Key << "JointsInvolvedCount" << YAML::Value << YAML::BeginMap;
        for (const auto &jointCount : jointsInvolvedCount ) {
            out << YAML::Key << jointCount.first;
            out << YAML::Value << jointCount.second;
        } 
        out << YAML::EndMap;
        
        out << YAML::Key << "ActionsJointPosFinal" << YAML::Value << YAML::BeginMap;
        for ( const auto joint : jointPosFinal ) {
            out << YAML::Key << joint.first;
            out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
        }
        out << YAML::EndMap;
        
        out << YAML::Key << "ActionsNamesOrdered" << YAML::Value << YAML::Flow << actionsNamesOrdered;

        out << YAML::Key << "ActionsTimeMargins" << YAML::Value << YAML::BeginMap;
        for (const std::string action : actionsNamesOrdered ){

            out << YAML::Key << action << YAML::Value << YAML::BeginMap;
                
                out << YAML::Key << "marginBefore" << 
                    YAML::Value << actionsTimeMarginsMap.at(action).first;
                    
                out << YAML::Key << "marginAfter" << 
                    YAML::Value << actionsTimeMarginsMap.at(action).second;
            out << YAML::EndMap;
        }
        out << YAML::EndMap;

        out << YAML::Key << "ActionsJointPos" << YAML::Value << YAML::BeginMap;
        for (const std::string action : actionsNamesOrdered ){

            out << YAML::Key << action << YAML::Value << YAML::BeginMap;
                
                for ( const auto joint : actionsJointPosMap.at(action) ) {
                    out << YAML::Key << joint.first;
                    out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
                }
            out << YAML::EndMap;
        }
        out << YAML::EndMap;
        
        
        out << YAML::Key << "ActionsJointCount" << YAML::Value << YAML::BeginMap;
        for (const std::string action : actionsNamesOrdered ){

            out << YAML::Key << action << YAML::Value << YAML::BeginMap;
                
                for ( const auto joint : actionsJointCountMap.at(action) ) {
                    out << YAML::Key << joint.first;
                    out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
                }
            out << YAML::EndMap;
        }
        out << YAML::EndMap;
        


        
        out << YAML::EndMap; // map began at the beginning of the function
    out << YAML::EndMap;// map began at the beginning of the function
}

bool ROSEE::ActionTimed::fillFromYaml(YAML::const_iterator yamlIt){
    
    name = yamlIt->first.as<std::string>();
    type = Action::Type::Timed;
    
    for (auto keyValue = yamlIt->second.begin(); keyValue != yamlIt->second.end(); ++keyValue ) {

        std::string key = keyValue->first.as<std::string>();
        
        if ( key.compare ("FingersInvolved") == 0 ) { 
            auto tempVect = keyValue->second.as <std::vector <std::string> > ();
            fingersInvolved.insert ( tempVect.begin(), tempVect.end() );
            
        } else if ( key.compare ("Type") == 0 ) {
            if (ROSEE::Action::Type::Timed != static_cast<ROSEE::Action::Type> ( keyValue->second.as <unsigned int>() )) {
                std::cout << "[Timed ACTION::" << __func__ << "] Error, found type  " << keyValue->second.as <unsigned int>()
                << "instead of Timed type (" << ROSEE::Action::Type::Timed << ")" << std::endl;
                return false;
            }
            type = ROSEE::Action::Type::Timed;
            
        } else if ( key.compare ("ActionsNamesOrdered") == 0 ) {
            actionsNamesOrdered = keyValue->second.as < std::vector <std::string> > ();
            
        } else if ( key.compare ("JointsInvolvedCount") == 0 ) {
            jointsInvolvedCount = keyValue->second.as < JointsInvolvedCount >(); 
            
        } else if ( key.compare("ActionsTimeMargins") == 0 ) {
            
            for (auto tMargins = keyValue->second.begin(); tMargins != keyValue->second.end(); ++tMargins ) {

                std::string actNAme = tMargins->first.as<std::string>();
                double before = tMargins->second["marginBefore"].as<double>();
                double after = tMargins->second["marginAfter"].as<double>();
                actionsTimeMarginsMap.insert (std::make_pair (actNAme, std::make_pair(before, after)  ) ) ;
            }
            
        } else if ( key.compare("ActionsJointPos") == 0) {
            
            for (auto jPos = keyValue->second.begin(); jPos != keyValue->second.end(); ++jPos ) {
                
                std::string actName = jPos->first.as<std::string>();
                JointPos jp = jPos->second.as<ROSEE::JointPos>();
                actionsJointPosMap.insert (std::make_pair (actName, jp) );
            }

        } else if ( key.compare("ActionsJointCount") == 0) {
            
            for (auto jCount = keyValue->second.begin(); jCount != keyValue->second.end(); ++jCount ) {
                
                std::string actName = jCount->first.as<std::string>();
                JointsInvolvedCount jc = jCount->second.as<ROSEE::JointsInvolvedCount>();
                actionsJointCountMap.insert (std::make_pair (actName, jc) );
            }

            
        } else if ( key.compare("ActionsJointPosFinal") == 0) {
            
            jointPosFinal =  keyValue->second.as < JointPos >();
        
        } else {
            std::cerr << "[TIMEDACTION::" << __func__ << "] Error, not known key " << key << std::endl;
            return false;
        }
    }
    
    return true;
}

bool ROSEE::ActionTimed::insertAction(ROSEE::Action::Ptr action, double marginBefore, double marginAfter, 
                                      unsigned int jointPosIndex, double percentJointPos, std::string newActionName) {
    
    std::string usedName = (newActionName.empty()) ? action->getName() : newActionName;

    unsigned int count = actionsJointPosMap.count( usedName );

    if (count > 0) {
        std::cerr << "[ACTIONTIMED:: " << __func__ << "] ERROR: " << usedName << " already present. Failed Insertion" << std::endl;
        return false;
    }

    if (marginAfter < 0 || marginBefore < 0) {
        std::cerr << "[ACTIONTIMED:: " << __func__ << "] ERROR: Please pass positive time margins" << std::endl;
        return false;
    }
    
    if ( jointPosIndex > action->getAllJointPos().size()-1 ) {
        std::cerr << "[ACTIONTIMED:: " << __func__ << "] ERROR: you pass index "<< jointPosIndex <<
            " but there are only " << action->getAllJointPos().size() << " JointPos in the action passed as argument" << std::endl;
        return false;
    }
    
    if (percentJointPos < 0 || percentJointPos > 1) {
        std::cerr << "[ACTIONTIMED:: " << __func__ << "] Please insert percentage for jointpos between 0 and 1. Passed: " 
            << percentJointPos << std::endl;
        return false;
    }
    
    if ( actionsNamesOrdered.size() > 0 && 
        (! ROSEE::Utils::keys_equal(action->getAllJointPos().at(jointPosIndex), actionsJointPosMap.begin()->second)) ) {
        //we need only to compare the first jointPos (.begin())
        std::cerr << "[ACTIONTIMED:: " << __func__ << "] The action passed as argument has different keys in jointPosmap" 
                  << " respect to the others inserted in this timed action " << std::endl;
        return false;
    }

    ROSEE::JointPos insertingJP = (percentJointPos)*(action->getAllJointPos().at( jointPosIndex )) ;
    actionsJointPosMap.insert (std::make_pair ( usedName, insertingJP) );
    actionsTimeMarginsMap.insert ( std::make_pair( usedName, std::make_pair(marginBefore, marginAfter)));
    actionsNamesOrdered.push_back ( usedName );
    actionsJointCountMap.insert (std::make_pair (usedName, action->getJointsInvolvedCount()));

    //father member
    for ( auto it: action->getFingersInvolved() ) {
         fingersInvolved.insert ( it );
    }

    if (actionsNamesOrdered.size() == 1 ) { //We are inserting first action, we have to init the JointsInvolvedCount map 
        
        jointsInvolvedCount = action->getJointsInvolvedCount();
        jointPosFinal = insertingJP;
        
    } else { 
        // add the action.jointInvolvedCount to the timed jointCount
        // and update jointPosFinal 
        for (auto jic : action->getJointsInvolvedCount() ) {
            jointsInvolvedCount.at(jic.first) += jic.second;
            
            if (jic.second > 0) {
                //if so, we must overwrite the pos of this joint in jointPosFina
                jointPosFinal.at(jic.first) = insertingJP.at(jic.first);
            }
        } 
    }

    return true;
}


