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

#include <ROSEndEffector/ActionTimed.h>

ROSEE::ActionTimed::ActionTimed() {
    
}

ROSEE::ActionTimed::ActionTimed (std::string name ) : Action(name) {
    
}

ROSEE::JointPos ROSEE::ActionTimed::getJointPos() const {
    return (actionsJointPosMap.at(actionsNamesOrdered.back()));
}

std::vector<ROSEE::JointPos> ROSEE::ActionTimed::getAllJointPos() const {
    
    std::vector<ROSEE::JointPos> jpVect;
    jpVect.reserve (actionsNamesOrdered.size());
    for (auto actName : actionsNamesOrdered) {
        jpVect.push_back( actionsJointPosMap.at (actName) );
    }
    return jpVect;
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

std::pair <double, double> ROSEE::ActionTimed::getActionMargins ( std::string actionName ) const {
 
    auto it = actionsTimeMarginsMap.find(actionName);
    
    if ( it != actionsTimeMarginsMap.end() ) {
        return ( it->second );
        
    } else {
        std::cerr << "[ACTIONTIMED:: " << __func__ << "] ERROR: action " << actionName << " not present in this composed timed action" << std::endl;
        return std::make_pair(-1, -1);
    }
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
        out << YAML::Key << "FingersInvolved" << YAML::Value << YAML::Flow << fingersInvolved;
        out << YAML::Key << "JointsInvolvedCount" << YAML::Value << YAML::BeginMap;
        for (const auto &jointCount : jointsInvolvedCount ) {
            out << YAML::Key << jointCount.first;
            out << YAML::Value << jointCount.second;
        } 
        out << YAML::EndMap;
        
        out << YAML::Key << "ActionsNamesOrdered" << YAML::Value << YAML::Flow << actionsNamesOrdered;

        for (int i = 0; i < actionsNamesOrdered.size(); i++){

            out << YAML::Key << ("ActionTimeMargins_" + std::to_string(i+1))
                << YAML::Comment(actionsNamesOrdered.at(i)) << YAML::Value << YAML::BeginMap;
                
                out << YAML::Key << "marginBefore" << 
                    YAML::Value << actionsTimeMarginsMap.at(actionsNamesOrdered.at(i)).first;
                    
                out << YAML::Key << "marginAfter" << 
                    YAML::Value << actionsTimeMarginsMap.at(actionsNamesOrdered.at(i)).second;
            out << YAML::EndMap;
        }
        
        for (int i = 0; i < actionsNamesOrdered.size(); i++){

            out << YAML::Key << ("JointPos_" + std::to_string(i+1))
                << YAML::Comment(actionsNamesOrdered.at(i)) << YAML::Value << YAML::BeginMap;
                for (const auto joint : actionsJointPosMap.at(actionsNamesOrdered.at(i))) {
                    out << YAML::Key << joint.first;
                    out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
                }
            out << YAML::EndMap;
        }
        
        out << YAML::EndMap;
    out << YAML::EndMap;
}

bool ROSEE::ActionTimed::fillFromYaml(YAML::const_iterator yamlIt){
    
    name = yamlIt->first.as<std::string>();
    std::vector<JointPos> tempJointPos;
    std::vector<std::pair <double, double>> tempTimeMargins;
    
    for (auto keyValue = yamlIt->second.begin(); keyValue != yamlIt->second.end(); ++keyValue ) {

        std::string key = keyValue->first.as<std::string>();
        
        if ( key.compare ("FingersInvolved") == 0 ) { 
            auto tempVect = keyValue->second.as <std::vector <std::string> > ();
            fingersInvolved.insert ( tempVect.begin(), tempVect.end() );
            
        } else if ( key.compare ("JointsInvolvedCount") == 0 ) {
            jointsInvolvedCount = keyValue->second.as < JointsInvolvedCount >(); 
        
        } else if ( key.compare ("ActionsNamesOrdered") == 0 ) {
            actionsNamesOrdered = keyValue->second.as < std::vector <std::string> > ();
            
        } else if ( key.compare(0, 18, "ActionTimeMargins_") == 0 ) { //compare 18 caracters from index 0 of key
            //we store temporanely in the vector, only after we can put in the actionsTimeMarginsMap
            //because we may not know the keys of actionsTimeMarginsMap at this point.
            double before = keyValue->second["marginBefore"].as<double>();
            double after = keyValue->second["marginAfter"].as<double>();
            tempTimeMargins.push_back (std::make_pair(before, after) ) ;
            
        } else if ( key.compare(0, 9, "JointPos_") == 0 ) { //compare 9 caracters from index 0 of key
            //we store temporanely in the vector, only after we can put in the actionsJointPosMap
            //because we may not know the keys of actionsJointPosMap at this point.
            tempJointPos.push_back ( keyValue->second.as <JointPos>() ) ;
            
        } else {
            std::cerr << "[TIMEDACTION::" << __func__ << "] Error, not known key " << key << std::endl;
            return false;
        }
    }
    
    if (tempJointPos.size() != actionsNamesOrdered.size() ) {
        std::cerr << "[TIMEDACTION::" << __func__ << "] Error size of actionsNamesOrdered is " << actionsNamesOrdered.size()
        << " while there are " << tempJointPos.size() << " joint positions" << std::endl;

        return false;
    }
    
    if (tempTimeMargins.size() != actionsNamesOrdered.size() ) {
        std::cerr << "[TIMEDACTION::" << __func__ << "] Error size of actionsNamesOrdered is " << actionsNamesOrdered.size()
        << " while there are " << tempTimeMargins.size() << " pair of time margins" << std::endl;

        return false;
    }
    
    for (int i = 0; i < actionsNamesOrdered.size() ; i++ ) {
        actionsJointPosMap.insert ( std::make_pair (actionsNamesOrdered.at(i), tempJointPos.at(i) ) );
        actionsTimeMarginsMap.insert ( std::make_pair (actionsNamesOrdered.at(i), tempTimeMargins.at(i) ) );
    }
    
    return true;
}

bool ROSEE::ActionTimed::insertAction(ROSEE::Action::Ptr action, double marginBefore, double marginAfter, 
                                      unsigned int jointPosIndex, std::string newActionName) {
    
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

    actionsJointPosMap.insert (std::make_pair ( usedName, action->getAllJointPos().at( jointPosIndex ) ));
    actionsTimeMarginsMap.insert ( std::make_pair( usedName, std::make_pair(marginBefore, marginAfter)));
    actionsNamesOrdered.push_back ( usedName );

    //father member
    for ( auto it: action->getFingersInvolved() ) {
         fingersInvolved.insert ( it );
    }

    if (actionsNamesOrdered.size() == 1 ) { //We are inserting first action, we have to init the JointsInvolvedCount map 
        
        jointsInvolvedCount = action->getJointsInvolvedCount();
        
    } else {
        for (auto jic : action->getJointsInvolvedCount() ) {
            jointsInvolvedCount.at(jic.first) += jic.second;
        } 
    }
    
    return true;
}


