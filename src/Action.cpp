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

#include <ROSEndEffector/Action.h>

ROSEE::Action::Action () {

}


ROSEE::Action::Action ( std::string name ) {
    this->name = name;
}

std::string ROSEE::Action::getName() const {
    return name;
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
