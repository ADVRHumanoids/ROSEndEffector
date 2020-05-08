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

#include <ros_end_effector/Action.h>

/******************************** OPERATORS OVERLOAD FOR TYPEDEFS ***********************************/

/** operator overload for JointPos so it is easier to print */
std::ostream& ROSEE::operator << (std::ostream& output, const ROSEE::JointPos jp) {
    for (const auto &jsEl : jp) {
        output << "\t\t"<<jsEl.first << " : "; //joint name
        for(const auto &jValue : jsEl.second){
            output << jValue << ", "; //joint position (vector because can have multiple dof)
        }
        output.seekp (-2, output.cur); //to remove the last comma (and space)
        output << std::endl;       
    }
    return output;
}

ROSEE::JointPos ROSEE::operator * (const double multiplier, const ROSEE::JointPos jp) {
    
    ROSEE::JointPos jpNew;
    for (const auto &jsEl : jp) {
        std::vector<double> newPos;
       // std::cout << jsEl.first << std::endl;
        for (const double pos : jsEl.second) {
            //std::cout << "old " << pos << "   new: " << pos*multiplier << std::endl;
            newPos.push_back (pos*multiplier);
        }
        jpNew.insert ( std::make_pair (jsEl.first, newPos) );
    }
    
    return jpNew;
}

ROSEE::JointPos ROSEE::operator * (const ROSEE::JointPos jp, const double multiplier ) {
    return (multiplier * jp );
}

ROSEE::JointPos ROSEE::operator + (const ROSEE::JointPos jp1, const ROSEE::JointPos jp2) {
    
    if ( ! ROSEE::Utils::keys_equal(jp1, jp2) ) {
        throw ROSEE::Utils::DifferentKeysException<ROSEE::JointPos, ROSEE::JointPos>(&jp1, &jp2);
    }
    
    ROSEE::JointPos jpNew;
    for (const auto &jsEl : jp1) {
        if (jsEl.second.size() != jp2.at(jsEl.first).size() ) {
            throw "Dofs not same";
        }

        std::vector<double> newPos;
        for (int i = 0; i < jsEl.second.size(); i++) {
            newPos.push_back (jsEl.second.at(i) +  jp2.at(jsEl.first).at(i));
        }
        jpNew.insert ( std::make_pair (jsEl.first, newPos) );
    }
    
    return jpNew;
}

std::ostream& ROSEE::operator << (std::ostream& output, const ROSEE::JointsInvolvedCount jic) {
    for (const auto &jicEl : jic) {
        output << "\t"<< jicEl.first << " : " << jicEl.second;
        output << std::endl;       
    }
    return output;
}

/********************************************* CLASS ACTION **************************************/
ROSEE::Action::Action () {

}


ROSEE::Action::Action ( std::string name, Action::Type type ) {
    this->name = name;
    this->type = type;
}

std::string ROSEE::Action::getName() const {
    return name;
}

ROSEE::Action::Type ROSEE::Action::getType() const {
    return type;
}


std::set<std::string> ROSEE::Action::getFingersInvolved() const {
    return fingersInvolved;
}


ROSEE::JointsInvolvedCount ROSEE::Action::getJointsInvolvedCount() const {
    return jointsInvolvedCount;
}


void ROSEE::Action::print () const {
    
    std::stringstream output;
    output << "ActionName: " << name << std::endl;
    
    if (fingersInvolved.size() > 0 ){
        output << "FingersInvolved: [";
        for (auto fingName : fingersInvolved){
            output << fingName << ", " ;
        }
        output.seekp (-2, output.cur); //to remove the last comma (and space)
        output << "]" << std::endl;
        
    } else {
        output << "FingersInvolved: <not inserted>" << std::endl; // can happen, for ex for genericAction
    }

    
    output << "JointsInvolvedCount: " << std::endl;;
    output << jointsInvolvedCount << std::endl;
    
    output << "JointPos:" << std::endl;
    output << getJointPos() << std::endl;

    output << std::endl;

    std::cout << output.str();
}
