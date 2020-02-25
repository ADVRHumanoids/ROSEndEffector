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

#include <ROSEndEffector/ActionPrimitive.h>

ROSEE::ActionPrimitive::ActionPrimitive(
    std::string name, unsigned int nFingersInvolved, unsigned int maxStoredActionStates,
        Type type) :
        Action(name), nFingersInvolved(nFingersInvolved), maxStoredActionStates(maxStoredActionStates),
        type(type) {}


ROSEE::ActionPrimitive::Type ROSEE::ActionPrimitive::getType() const {
    return type;
}

unsigned int ROSEE::ActionPrimitive::getMaxStoredActionStates() const {
    return maxStoredActionStates;
}

unsigned int ROSEE::ActionPrimitive::getnFingersInvolved() const {
    return nFingersInvolved;
}

void ROSEE::ActionPrimitive::setJointsInvolvedCount(ROSEE::JointsInvolvedCount jointsInvolvedCount) {
    this->jointsInvolvedCount = jointsInvolvedCount;
}



void ROSEE::ActionPrimitive::emitYaml ( YAML::Emitter& out ) const {

    // key: set of string (eg two tip names)
    out << YAML::Key << YAML::Flow << fingersInvolved;

    unsigned int nCont = 1;
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "ActionName" << YAML::Value << name;
    out << YAML::Key << "JointsInvolvedCount" << YAML::Value << YAML::BeginMap;
    for (const auto &jointCount : jointsInvolvedCount ) {
        out << YAML::Key << jointCount.first;
        out << YAML::Value << jointCount.second;
    }
    out << YAML::EndMap;


    for (const auto & jointPos : getAllJointPos()) {

        std::string contSeq = "ActionState_" + std::to_string(nCont);
        out << YAML::Key << contSeq;

        out << YAML::Value << YAML::BeginMap;
            //actionState.first, the jointstates map
            out << YAML::Key << "JointPos" << YAML::Value << YAML::BeginMap;
            for (const auto &joint : jointPos) {
                out << YAML::Key << joint.first;
                out << YAML::Value << YAML::Flow << joint.second; //vector of double is emitted like Seq
            }
            out << YAML::EndMap;

        out << YAML::EndMap;
        nCont++;
    }
    out << YAML::EndMap;
}


