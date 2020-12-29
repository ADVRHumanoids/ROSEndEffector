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

ROSEE::RosServiceHandler::RosServiceHandler( ros::NodeHandle *nh, ROSEE::MapActionHandler::Ptr mapActionHandler, std::string path2saveYamlGeneric) {
    
    if (mapActionHandler == nullptr) {
        ROS_ERROR_STREAM ( "[RosServiceHandler " << __func__ << " ] the mapActionHandler in not initialized");
        return;
    }
    
    this->_mapActionHandler = mapActionHandler;
    this->_path2saveYamlGeneric = path2saveYamlGeneric;

    this->_nh = nh;
    
}

//TODO see if this argument can be avoided, maybe use extract_keys_merged in map action handler?
bool ROSEE::RosServiceHandler::init(unsigned int nFinger) {
    
    this->nFinger = nFinger;
    
    std::string graspingActionsSrvName, actionInfoServiceName, 
      selectablePairInfoServiceName, handInfoServiceName, newGraspingActionServiceName;

    _nh->param<std::string>("/rosee/grasping_action_srv_name", graspingActionsSrvName, "grasping_actions_available");
    _nh->param<std::string>("/rosee/primitive_aggregated_srv_name", actionInfoServiceName, "primitives_aggregated_available");
    _nh->param<std::string>("/rosee/selectable_finger_pair_info", selectablePairInfoServiceName, "selectable_finger_pair_info");
    _nh->param<std::string>("/rosee/hand_info", handInfoServiceName, "hand_info");
    _nh->param<std::string>("/rosee/new_grasping_action_srv_name", newGraspingActionServiceName, "new_generic_grasping_action");
    
    _serverGraspingActions = _nh->advertiseService(graspingActionsSrvName, 
        &RosServiceHandler::graspingActionsCallback, this);

    _serverPrimitiveAggregated = _nh->advertiseService(actionInfoServiceName, 
        &RosServiceHandler::primitiveAggregatedCallback, this);
    
    _server_selectablePairInfo = _nh->advertiseService(selectablePairInfoServiceName, 
        &RosServiceHandler::selectablePairInfoCallback, this);
    
    _serverHandInfo = _nh->advertiseService(handInfoServiceName, 
        &RosServiceHandler::handInfoCallback, this);

    _serverNewGraspingAction = _nh->advertiseService(newGraspingActionServiceName, 
        &RosServiceHandler::newGraspingActionCallback, this);
    
    return true;
}

bool ROSEE::RosServiceHandler::graspingActionsCallback(
    rosee_msg::GraspingActionsAvailable::Request& request,
    rosee_msg::GraspingActionsAvailable::Response& response) {
    
    switch (request.action_type) {
        
        case 0 : { //PRIMITIVE
            
            //NOTE if both primitive type and action name are set in the request, the action name is not considered

            if (request.primitive_type == 0) { 
                
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

    fillCommonInfoGraspingActionMsg(primitive, &primitiveMsg);

    primitiveMsg.primitive_type = primitive->getPrimitiveType();
    
    auto elements = primitive->getKeyElements();
    primitiveMsg.elements_involved.assign(elements.begin(), elements.end());
    
    return primitiveMsg;
}

rosee_msg::GraspingAction ROSEE::RosServiceHandler::fillGraspingActionMsg(ROSEE::ActionGeneric::Ptr generic) {
    
    rosee_msg::GraspingAction genericActionMsg;
    if (generic == nullptr) {
        return genericActionMsg;
    }
    
    fillCommonInfoGraspingActionMsg(generic, &genericActionMsg);

    genericActionMsg.primitive_type = genericActionMsg.PRIMITIVE_NONE;
    
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
    
    fillCommonInfoGraspingActionMsg(timed, &timedActionMsg);

    timedActionMsg.primitive_type = timedActionMsg.PRIMITIVE_NONE;
    
    timedActionMsg.inner_actions = timed->getInnerActionsNames();

    for (auto innerMargin : timed->getAllActionMargins()){
        timedActionMsg.before_time_margins.push_back(innerMargin.first);
        timedActionMsg.after_time_margins.push_back(innerMargin.second);
    }
    
    return timedActionMsg;
}

void ROSEE::RosServiceHandler::fillCommonInfoGraspingActionMsg(ROSEE::Action::Ptr action, 
                                                               rosee_msg::GraspingAction* graspingMsg) {
    
    graspingMsg->action_type = action->getType();
    graspingMsg->action_name = action->getName();
    
    rosee_msg::JointsInvolvedCount motorCountMsg;
    for ( auto motorCount : action->getJointsInvolvedCount()) { 
        motorCountMsg.name.push_back(motorCount.first);
        motorCountMsg.count.push_back(motorCount.second);
    }
    graspingMsg->action_motor_count = motorCountMsg;
    
    for ( auto motorPosMultiple : action->getAllJointPos()) {

        //iterate over the single motor positions
        rosee_msg::MotorPosition motorPosMsg;
        for ( auto motorPos : motorPosMultiple) { 
            motorPosMsg.name.push_back(motorPos.first);
            motorPosMsg.position.push_back(motorPos.second.at(0)); //at(0). because multiple dof is considered in general

        }
        graspingMsg->action_motor_positions.push_back(motorPosMsg); 
    }
    
    for (auto elementInvolved : action->getFingersInvolved()) {
        graspingMsg->elements_involved.push_back(elementInvolved);
    }
}

bool ROSEE::RosServiceHandler::primitiveAggregatedCallback(
    rosee_msg::GraspingPrimitiveAggregatedAvailable::Request& request,
    rosee_msg::GraspingPrimitiveAggregatedAvailable::Response& response) {
    
     if (request.primitive_type == 0) { 
                
        if (request.action_name.size() == 0 ) {
            // return all primitives
                        
            for (auto primitiveMaps : _mapActionHandler->getAllPrimitiveMaps() ) {
                
                response.primitives_aggregated.push_back(fillPrimitiveAggregatedMsg(primitiveMaps.second));
            }
            
        } else {
            if (request.elements_involved.size() == 0) {
                
                auto primitiveMap = _mapActionHandler->getPrimitiveMap(request.action_name);                     
                response.primitives_aggregated.push_back(fillPrimitiveAggregatedMsg(primitiveMap));

                
            } else {

                auto primitive =_mapActionHandler->getPrimitive(request.action_name, request.elements_involved);
                response.primitives_aggregated.push_back(fillPrimitiveAggregatedMsg(primitive));
            }
        }
        
    } else {
        
        if (request.elements_involved.size() == 0) {
                
            //NOTE -1 because in the srv 0 is for all primitives, then the enum is scaled by one
            for (auto primitiveMap : _mapActionHandler->getPrimitiveMap(
                    static_cast<ROSEE::ActionPrimitive::Type>(request.primitive_type-1))) {
            
                response.primitives_aggregated.push_back(fillPrimitiveAggregatedMsg(primitiveMap));

            }
                        
        } else {
                
            for (auto primitive : _mapActionHandler->getPrimitive(
                    static_cast<ROSEE::ActionPrimitive::Type>(request.primitive_type-1), request.elements_involved)) {
                                    
                response.primitives_aggregated.push_back(fillPrimitiveAggregatedMsg(primitive));
            }
        }
    }
    return true;    
}


rosee_msg::GraspingPrimitiveAggregated ROSEE::RosServiceHandler::fillPrimitiveAggregatedMsg(
    ROSEE::MapActionHandler::ActionPrimitiveMap primitiveMap) {
    
    rosee_msg::GraspingPrimitiveAggregated primitiveMsg;

    primitiveMsg.action_name = primitiveMap.begin()->second->getName();
    primitiveMsg.primitive_type = primitiveMap.begin()->second->getPrimitiveType();
    //until now, there is not a primitive that does not have "something" to select
    // (eg pinch has 2 fing, trig one fing, singleJointMultipleTips 1 joint...). 
    //Instead generic action has always no thing to select (next for loop)
    primitiveMsg.max_selectable = primitiveMap.begin()->first.size();
    //TODO extract the keys with another mapActionHandler function?
    primitiveMsg.selectable_names =
        ROSEE::Utils::extract_keys_merged(primitiveMap, nFinger);

    return primitiveMsg;
}

rosee_msg::GraspingPrimitiveAggregated ROSEE::RosServiceHandler::fillPrimitiveAggregatedMsg(
    ROSEE::ActionPrimitive::Ptr primitive) {

    rosee_msg::GraspingPrimitiveAggregated primitiveMsg;
    primitiveMsg.action_name = primitive->getName();
    primitiveMsg.primitive_type = primitive->getPrimitiveType();
    
    auto elements = primitive->getKeyElements();
    primitiveMsg.max_selectable = elements.size();
    primitiveMsg.selectable_names.assign(elements.begin(), elements.end());

    return primitiveMsg;
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

bool ROSEE::RosServiceHandler::handInfoCallback(
        rosee_msg::HandInfo::Request& request,
        rosee_msg::HandInfo::Response& response) {

    if (! ros::service::exists("/EEHalExecutor/hand_info", false) ) {
        return false;
    }
    
    ros::ServiceClient handInfoClient = 
        _nh->serviceClient<rosee_msg::HandInfo>("/EEHalExecutor/hand_info");
    rosee_msg::HandInfo handInfoMsg;
    
    //request may be empty or not according to which hal we are using
    handInfoMsg.request = request;
    if (handInfoClient.call(handInfoMsg)) {
        
        response = handInfoMsg.response;
        
    } else {
        return false;
    }
    
    return true;
}
        
//TODO error msg useless becaus if return false the response is not send
//at today (2020) it seems there not exist a method to return false plus an error message.
bool ROSEE::RosServiceHandler::newGraspingActionCallback(
        rosee_msg::NewGenericGraspingActionSrv::Request& request,
        rosee_msg::NewGenericGraspingActionSrv::Response& response){
    
    response.accepted = false;
    response.emitted = false;
    
    if (request.newAction.action_name.empty()) {
        
        response.error_msg = "action_name can not be empty";
        ROS_ERROR_STREAM ( "[RosServiceHandler " << __func__ << " ] " << response.error_msg);
        return true; //so the client receive the response
    }
    
    if (request.newAction.action_motor_position.name.size() == 0 ||
        request.newAction.action_motor_position.position.size() == 0 ||
        request.newAction.action_motor_position.position.size() != request.newAction.action_motor_position.name.size()) {
        
        response.error_msg = "action_motor_position is empty or badly formed";
        ROS_ERROR_STREAM ( "[RosServiceHandler " << __func__ << " ] " << response.error_msg);

        return true; //so the client receive the response
    }
    
    if (request.newAction.action_motor_count.name.size() != request.newAction.action_motor_count.count.size()) {
        
        response.error_msg = "action_motor_count is badly formed, name and count have different sizes";
        ROS_ERROR_STREAM ( "[RosServiceHandler " << __func__ << " ] " << response.error_msg);

        return true; //so the client receive the response
    }
    
    // TODO request.newAction.action_motor_count : if empty, ActionGeneric costructor will consider all joint
    // with 0 position as not used. This may change in future when we will support not 0 default joint positions
    
    if (_mapActionHandler->getGeneric(request.newAction.action_name, false) != nullptr) {
        
        response.error_msg = "A generic action with name '" + request.newAction.action_name + "' already exists";
        ROS_ERROR_STREAM ( "[RosServiceHandler " << __func__ << " ] " << response.error_msg);

        return true; //so the client receive the response
        
    }

    ROSEE::ActionGeneric::Ptr newAction;
    ROSEE::JointPos jp;
    ROSEE::JointsInvolvedCount jic;
    std::set<std::string> elementInvolved;
    
    for (int i = 0; i < request.newAction.action_motor_position.name.size(); i++){
        
        std::vector<double> one_dof{request.newAction.action_motor_position.position.at(i)};        
        jp.insert(std::make_pair(request.newAction.action_motor_position.name.at(i),
                                 one_dof));
    }
    
    for (int i = 0; i < request.newAction.action_motor_count.name.size(); i++){
        
        jic.insert(std::make_pair(request.newAction.action_motor_count.name.at(i),
                                  request.newAction.action_motor_count.count.at(i)));
    }
    
    for (int i = 0; i< request.newAction.elements_involved.size(); i++) {
        
        elementInvolved.insert(request.newAction.elements_involved.at(i));  
    }
    
    //costructor will handle jpc and elementInvolved also if empty
    try { newAction = std::make_shared<ROSEE::ActionGeneric>(request.newAction.action_name,
                                                             jp,
                                                             jic,
                                                             elementInvolved);
    
    } catch (const ROSEE::Utils::DifferentKeysException<ROSEE::JointPos, ROSEE::JointsInvolvedCount>) {
        
        response.error_msg = "action_motor_position and action_motor_count have different names element. They must be the same because they refer to actuator of the end-effector";
        ROS_ERROR_STREAM ( "[RosServiceHandler " << __func__ << " ] " << response.error_msg);

        return true; //so the client receive the response
    } 
    
    //u rosee main node use always mapActionHandler to check if an action exists. Thus, we need to add this new 
    // action into the mapActionHandler "database" (ie private member map of the generic actions)
    if (! _mapActionHandler->insertSingleGeneric(newAction)){
        
        response.error_msg = "error by mapActionHandler when inserting the new generic action";
        ROS_ERROR_STREAM ( "[RosServiceHandler " << __func__ << " ] " << response.error_msg);

        return true; //so the client receive the response
    }
    
    ROS_INFO_STREAM ( "[RosServiceHandler " << __func__ << " ] The new action '"<< newAction->getName() << "' is inserted in the system");

    
    if (request.emitYaml) {
        
        ROSEE::YamlWorker yamlWorker;
        auto path = yamlWorker.createYamlFile(newAction, _path2saveYamlGeneric);
        ROS_INFO_STREAM ( "[RosServiceHandler " << __func__ << " ] The received new action '"<< newAction->getName() << "' has been stored in " << path);
        response.emitted = true;
    }
    
    response.accepted = true;
    return true;
}
