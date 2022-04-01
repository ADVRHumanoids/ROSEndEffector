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

#include <end_effector/GraspingActions/ActionPrimitive.h>

// std::ostream& operator<<(std::ostream& out, const ROSEE::ActionPrimitive::Type type){
//     const char* s = 0;
// #define PROCESS_VAL(p) case(p): s = #p; break;
//     switch(type){
//         PROCESS_VAL(ROSEE::ActionPrimitive::Type::PinchTight);     
//         PROCESS_VAL(ROSEE::ActionPrimitive::Type::PinchLoose);     
//         PROCESS_VAL(ROSEE::ActionPrimitive::Type::MultiplePinchTight);
//         PROCESS_VAL(ROSEE::ActionPrimitive::Type::Trig);
//         PROCESS_VAL(ROSEE::ActionPrimitive::Type::TipFlex);
//         PROCESS_VAL(ROSEE::ActionPrimitive::Type::FingFlex);
//         PROCESS_VAL(ROSEE::ActionPrimitive::Type::SingleJointMultipleTips);
//         PROCESS_VAL(ROSEE::ActionPrimitive::Type::None);
//     }
// #undef PROCESS_VAL
// 
//     return out << s;
// }

ROSEE::ActionPrimitive::ActionPrimitive(std::string name,  unsigned int maxStoredActionStates, ActionPrimitive::Type primitiveType) : 
    Action(name, Action::Type::Primitive), maxStoredActionStates(maxStoredActionStates), primitiveType(primitiveType) {};

ROSEE::ActionPrimitive::ActionPrimitive(
    std::string name, unsigned int nFingersInvolved, unsigned int maxStoredActionStates,
        ActionPrimitive::Type primitiveType) :
        Action(name, Action::Type::Primitive), nFingersInvolved(nFingersInvolved), maxStoredActionStates(maxStoredActionStates),
        primitiveType(primitiveType) {}


ROSEE::ActionPrimitive::Type ROSEE::ActionPrimitive::getPrimitiveType() const {
    return primitiveType;
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
    out << YAML::Key << "PrimitiveType" << YAML::Value << primitiveType;
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



