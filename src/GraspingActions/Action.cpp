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

#include <end_effector/GraspingActions/Action.h>

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

ROSEE::JointPos ROSEE::operator * ( double multiplier,  ROSEE::JointPos jp) {
    
    return jp *= multiplier;
}

ROSEE::JointPos ROSEE::operator * ( ROSEE::JointPos jp,  double multiplier ) {
    
    return jp *= multiplier;
}

ROSEE::JointPos& ROSEE::operator *= ( ROSEE::JointPos& jp, double multiplier) {
    
    for ( auto &jsEl : jp) {
        for (int i = 0; i< jsEl.second.size(); i++) {
            jsEl.second.at(i) *= multiplier;
        }
    }
    
    return jp;
}

ROSEE::JointPos ROSEE::operator + ( ROSEE::JointPos jp1, ROSEE::JointPos jp2) {
    
    return jp1 += jp2;
}

ROSEE::JointPos& ROSEE::operator += (ROSEE::JointPos& jp1, ROSEE::JointPos jp2) {
    
    if ( ! ROSEE::Utils::keys_equal(jp1, jp2) ) {
        throw ROSEE::Utils::DifferentKeysException<ROSEE::JointPos, ROSEE::JointPos>(&jp1, &jp2);
    }
    
    for (auto &jsEl : jp1) {
        if (jsEl.second.size() != jp2.at(jsEl.first).size() ) {
            throw "Dofs not same";
        }

        for (int i = 0; i < jsEl.second.size(); i++) {
            jsEl.second.at(i) += jp2.at(jsEl.first).at(i); 
        }
    }
    
    return jp1;
}

std::ostream& ROSEE::operator << (std::ostream& output, const ROSEE::JointsInvolvedCount jic) {
    for (const auto &jicEl : jic) {
        output << "\t"<< jicEl.first << " : " << jicEl.second;
        output << std::endl;       
    }
    return output;
}

std::ostream& ROSEE::operator<<(std::ostream& out, const ROSEE::Action::Type type){
    
        const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(type){
        PROCESS_VAL(ROSEE::Action::Type::Primitive);     
        PROCESS_VAL(ROSEE::Action::Type::Generic);     
        PROCESS_VAL(ROSEE::Action::Type::Composed);
        PROCESS_VAL(ROSEE::Action::Type::Timed);
        PROCESS_VAL(ROSEE::Action::Type::None);
    }
#undef PROCESS_VAL

    return out << s;
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
