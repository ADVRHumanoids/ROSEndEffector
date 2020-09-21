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

#include <ros_end_effector/RosServiceHandler.h>

ROSEE::RosServiceHandler::RosServiceHandler( ros::NodeHandle *nh, ROSEE::MapActionHandler::Ptr mapActionHandler) {
    
    if (mapActionHandler == nullptr) {
        ROS_ERROR_STREAM ( "[RosServiceHandler " << __func__ << " ] the mapActionHandler in not initialized");
        return;
    }
    
    this->_mapActionHandler = mapActionHandler;

    this->_nh = nh;
    
    initGraspingActionsServices();

}

//TODO see if this argument can be avoided, maybe use extract_keys_merged in map action handler?
bool ROSEE::RosServiceHandler::init(unsigned int nFinger) {
        
    for (auto primitiveContainers : _mapActionHandler->getAllPrimitiveMaps() ) {
        
        rosee_msg::ActionInfo actInfo;
        actInfo.action_name = primitiveContainers.first;
        actInfo.action_type = ROSEE::Action::Type::Primitive;
        actInfo.actionPrimitive_type = primitiveContainers.second.begin()->second->getPrimitiveType();
        //until now, there is not a primitive that does not have "something" to select
        // (eg pinch has 2 fing, trig one fing, singleJointMultipleTips 1 joint...). 
        //Instead generic action has always no thing to select (next for loop)
        actInfo.max_selectable = primitiveContainers.second.begin()->first.size();
        //TODO extract the keys with another mapActionHandler function?
        actInfo.selectable_names =
            ROSEE::Utils::extract_keys_merged(primitiveContainers.second, nFinger);
        _primitiveActionsInfoVect.push_back(actInfo);
        _allActionsInfoVect.push_back(actInfo);

    }

    for (auto genericMap : _mapActionHandler->getAllGenerics() ) {

        rosee_msg::ActionInfo actInfo;
        actInfo.action_name = genericMap.first;
        actInfo.action_type = genericMap.second->getType();
        actInfo.actionPrimitive_type = ROSEE::ActionPrimitive::Type::None;
        //Generic action has always no thing to select UNTIL NOW
        actInfo.max_selectable = 0;

        _genericActionsInfoVect.push_back(actInfo);
        _allActionsInfoVect.push_back(actInfo);

    }
    
    for (auto timedMap : _mapActionHandler->getAllTimeds() ) {
        
        rosee_msg::ActionInfo actInfo;
        actInfo.action_name = timedMap.first;
        actInfo.action_type = timedMap.second->getType();
        actInfo.actionPrimitive_type = ROSEE::ActionPrimitive::Type::None;
        actInfo.max_selectable = 0;
        // we use selectable items info to store in it the action that compose this timed
        for (std::string act : timedMap.second->getInnerActionsNames()) {
            actInfo.inner_actions.push_back(act);
            auto margin = timedMap.second->getActionMargins(act);
            actInfo.before_margins.push_back(margin.first);
            actInfo.after_margins.push_back(margin.second);
        }

        _timedActionsInfoVect.push_back(actInfo);
        _allActionsInfoVect.push_back(actInfo);

    }
    
    std::string actionInfoServiceName, selectablePairInfoServiceName;
    _nh->param<std::string>("/rosee/action_info_service", actionInfoServiceName, "actions_info");
    _nh->param<std::string>("/rosee/selectable_finger_pair_info", selectablePairInfoServiceName, "selectable_finger_pair_info");

    _server_actionsInfo = _nh->advertiseService(actionInfoServiceName, 
        &RosServiceHandler::actionsInfoCallback, this);
    
    _server_selectablePairInfo = _nh->advertiseService(selectablePairInfoServiceName, 
        &RosServiceHandler::selectablePairInfoCallback, this);
    
    return true;

}

bool ROSEE::RosServiceHandler::initGraspingActionsServices() { 
    
    std::string graspingActionsSrvName;
    _nh->param<std::string>("/rosee/grasping_action_srv_name", graspingActionsSrvName, "grasping_actions_available");
    
    _serverGraspingActions = _nh->advertiseService(graspingActionsSrvName, &RosServiceHandler::graspingActionsCallback, this);
    
}

bool ROSEE::RosServiceHandler::graspingActionsCallback(
    rosee_msg::GraspingActionsAvailable::Request& request,
    rosee_msg::GraspingActionsAvailable::Response& response) {
    
    switch (request.action_type) {
        
        case 0 : { //PRIMITIVE
            
            //NOTE if both primitive type and action name are set in the request, the action name is not considered

            if (request.primitive_type == 0) { 
                
                std::cout << ActionPrimitive::Type::PinchLoose << std::endl;

                if (request.action_name.size() == 0 ) {            
                    for (auto primitiveContainers : _mapActionHandler->getAllPrimitiveMaps() ) {
                        
                        //iterate all the primitive of a type
                        for (auto primitive : primitiveContainers.second) {
                            response.grasping_actions.push_back(fillGraspingActionMsg(primitive.second));
                        }
                    }
                    
                } else {
                    if (request.elements_involved.size() == 0) {
                        
                        for (auto primitive : _mapActionHandler->getPrimitiveMap(request.action_name)) {
                            response.grasping_actions.push_back(fillGraspingActionMsg(primitive.second));
                        }
                        
                    } else {
        
                        auto primitive =_mapActionHandler->getPrimitive(request.action_name, request.elements_involved);
                        response.grasping_actions.push_back(fillGraspingActionMsg(primitive));
                    }
                }
                
            } else {
               
                if (request.elements_involved.size() == 0) {
                        
                    //NOTE -1 because in the srv 0 is for all primitives, then the enum is scaled by one
                    for (auto primitives : _mapActionHandler->getPrimitiveMap(
                            static_cast<ROSEE::ActionPrimitive::Type>(request.primitive_type-1))) {
                        
                        for (auto primitive : primitives) {
                            response.grasping_actions.push_back(fillGraspingActionMsg(primitive.second));
                        }
                    }
                              
                } else {
                     
                    for (auto primitive : _mapActionHandler->getPrimitive(
                            static_cast<ROSEE::ActionPrimitive::Type>(request.primitive_type-1), request.elements_involved)) {
                        
                            response.grasping_actions.push_back(fillGraspingActionMsg(primitive));
                    }
                }
            }
            
            break;
        }
        
        case 1 : { //GENERIC_and_COMPOSED
            
            //NOTE for these some fields are ignored 
            if (request.action_name.size() == 0) {
                for (auto action : _mapActionHandler->getAllGenerics()) {
                    response.grasping_actions.push_back(fillGraspingActionMsg(action.second));
                }
                
            } else {
                response.grasping_actions.push_back(fillGraspingActionMsg(_mapActionHandler->getGeneric(request.action_name)));
            }
            break;
        }
        
        case 2 : { //TIMED
            if (request.action_name.size() == 0) {
                for (auto action : _mapActionHandler->getAllTimeds()) {
                    response.grasping_actions.push_back(fillGraspingActionMsg(action.second));
                }
                
            } else {
                response.grasping_actions.push_back(fillGraspingActionMsg(_mapActionHandler->getTimed(request.action_name)));
            }
            break;
        }
        
        default : {
            ROS_ERROR_STREAM ( "[RosServiceHandler " << __func__ << " ] request.actionType can only be 0(ALL), 1(PRIMITIVE), "
                << "2(GENERIC_and_COMPOSED), or 3(TIMED); I have received " << request.action_type);
            return false;
            
        }
    }
    
    return true;    
}

rosee_msg::GraspingAction ROSEE::RosServiceHandler::fillGraspingActionMsg(ROSEE::ActionPrimitive::Ptr primitive) {
    
    rosee_msg::GraspingAction primitiveMsg;
    
    if (primitive == nullptr) {
        return primitiveMsg;
    }

    primitiveMsg.action_name = primitive->getName();
    primitiveMsg.action_type = primitive->getType();
    primitiveMsg.primitive_type = primitive->getPrimitiveType();
    auto elements = primitive->getKeyElements();
    primitiveMsg.elements_involved.assign(elements.begin(), elements.end());


    //iterate all the possible motor pos (eg pinch with 2 finger can have more than one way to perform)
    for ( auto motorPosMultiple : primitive->getAllJointPos()) {

        //iterate over the single motor positions
        rosee_msg::MotorPosition motorPosMsg;
        for ( auto motorPos : motorPosMultiple) { 
            motorPosMsg.name.push_back(motorPos.first);
            motorPosMsg.position.push_back(motorPos.second.at(0)); //at(0). because multiple dof is considered in general

        }
        primitiveMsg.action_motor_positions.push_back(motorPosMsg); 

    }
    
    return primitiveMsg;
     
}

rosee_msg::GraspingAction ROSEE::RosServiceHandler::fillGraspingActionMsg(ROSEE::ActionGeneric::Ptr generic) {
    
    rosee_msg::GraspingAction genericActionMsg;
    if (generic == nullptr) {
        return genericActionMsg;
    }
    
    genericActionMsg.action_type = generic->getType();
    genericActionMsg.primitive_type = genericActionMsg.PRIMITIVE_NONE;
    genericActionMsg.action_name = generic->getName();
    rosee_msg::MotorPosition motorPosMsg;
    for ( auto motorPos : generic->getJointPos()) { 
        motorPosMsg.name.push_back(motorPos.first);
        motorPosMsg.position.push_back(motorPos.second.at(0)); //at(0). because multiple dof is considered in general

    }
    genericActionMsg.action_motor_positions.push_back(motorPosMsg);
    
    ActionComposed::Ptr composedCasted = std::dynamic_pointer_cast<ActionComposed>(generic);
    if ( composedCasted != nullptr) {
        genericActionMsg.inner_actions = composedCasted->getInnerActionsNames();
    }
    
    return genericActionMsg;

}

rosee_msg::GraspingAction ROSEE::RosServiceHandler::fillGraspingActionMsg(ROSEE::ActionTimed::Ptr timed) {
    
    rosee_msg::GraspingAction timedActionMsg;
    if (timed == nullptr) {
        return timedActionMsg;
    }
    
    timedActionMsg.action_type = timed->getType();
    timedActionMsg.primitive_type = timedActionMsg.PRIMITIVE_NONE;
    timedActionMsg.action_name = timed->getName();
    for ( auto motorPosInners : timed->getAllJointPos()) { 
        rosee_msg::MotorPosition motorPosMsg;

        for ( auto motorPos : motorPosInners) { 
            motorPosMsg.name.push_back(motorPos.first);
            motorPosMsg.position.push_back(motorPos.second.at(0)); //at(0). because multiple dof is considered in general
        }
        timedActionMsg.action_motor_positions.push_back(motorPosMsg);

    }
    
    for (auto innerMargin : timed->getAllActionMargins()){
        timedActionMsg.before_time_margins.push_back(innerMargin.first);
        timedActionMsg.after_time_margins.push_back(innerMargin.second);
    }
    
    timedActionMsg.inner_actions = timed->getInnerActionsNames();
    
    
    
    return timedActionMsg;

}

bool ROSEE::RosServiceHandler::actionsInfoCallback(
    rosee_msg::ActionsInfo::Request& request,
    rosee_msg::ActionsInfo::Response& response) {
    
    //here we only send the actionsInfo vector, it is better to build it not in this clbk    
    switch (request.actionType) {
        case 0 : { //ALL
            response.actionsInfo = _allActionsInfoVect;
            break;
        }
        
        case 1 : { //PRIMITIVe
            response.actionsInfo = _primitiveActionsInfoVect;
            break;
        }
        
        case 2 : { //GENERIC_and_COMPOSED
            response.actionsInfo = _genericActionsInfoVect;
            break;
        }
        
        case 3 : { //TIMED
            response.actionsInfo = _timedActionsInfoVect;
            break;
        }
        
        default : {
            ROS_ERROR_STREAM ( "[RosServiceHandler " << __func__ << " ] request.actionType can only be 0(ALL), 1(PRIMITIVE), "
                << "2(GENERIC_and_COMPOSED), or 3(TIMED); I have received " << request.actionType);
            return false;
            
        }
        
    }
    
    return true;    
}

bool ROSEE::RosServiceHandler::selectablePairInfoCallback(
    rosee_msg::SelectablePairInfo::Request& request,
    rosee_msg::SelectablePairInfo::Response& response) {
    
    std::set<std::string> companionFingers;
    if (request.action_name.compare ("pinchTight") == 0) {
        companionFingers =
            _mapActionHandler->getFingertipsForPinch(request.element_name,
                ROSEE::ActionPrimitive::Type::PinchTight) ;
                
    } else if (request.action_name.compare ("pinchLoose") == 0) {
        companionFingers =
            _mapActionHandler->getFingertipsForPinch(request.element_name,
                ROSEE::ActionPrimitive::Type::PinchLoose) ;
                
    } else {
        ROS_ERROR_STREAM ( "Received" << request.action_name << " that is not" <<
            "a recognizible action name to look for finger companions" );
        return false;
    }
    
    if (companionFingers.size() == 0) {
        return false;
    }
    
    //push the elements of set into the vector
    for (auto fing : companionFingers ) {
        response.pair_elements.push_back (fing);
    }
    
    return true;
        
}
