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

}

//TODO see if this argument can be avoided, maybe use extract_keys_merged in map action handler?
bool ROSEE::RosServiceHandler::init(unsigned int nFinger) {
        
    for (auto primitiveContainers : _mapActionHandler->getAllPrimitiveMaps() ) {
        
        rosee_msg::ActionInfo actInfo;
        actInfo.action_name = primitiveContainers.first;
        actInfo.action_type = ROSEE::Action::Type::Primitive;
        actInfo.actionPrimitive_type = primitiveContainers.second.begin()->second->getPrimitiveType();
        actInfo.seq = 0; //TODO check if necessary the seq in this msg
        //until now, there is not a primitive that does not have "something" to select
        // (eg pinch has 2 fing, trig one fing, singleJointMultipleTips 1 joint...). 
        //Instead generic action has always no thing to select (next for loop)
        actInfo.max_selectable = primitiveContainers.second.begin()->first.size();
        //TODO extract the keys with another mapActionHandler function?
        actInfo.selectable_names =
            ROSEE::Utils::extract_keys_merged(primitiveContainers.second, nFinger);
        _actionsInfoVect.push_back(actInfo);

    }

    for (auto genericMap : _mapActionHandler->getAllGenerics() ) {

        rosee_msg::ActionInfo actInfo;
        actInfo.action_name = genericMap.first;
        actInfo.action_type = genericMap.second->getType();
        actInfo.actionPrimitive_type = ROSEE::ActionPrimitive::Type::None;
        actInfo.seq = 0; //TODO check if necessary the seq in this msg
        //Generic action has always no thing to select UNTIL NOW
        actInfo.max_selectable = 0;

        _actionsInfoVect.push_back(actInfo);

    }
    
    for (auto timedMap : _mapActionHandler->getAllTimeds() ) {
        
        rosee_msg::ActionInfo actInfo;
        actInfo.action_name = timedMap.first;
        actInfo.action_type = timedMap.second->getType();
        actInfo.actionPrimitive_type = ROSEE::ActionPrimitive::Type::None;
        actInfo.seq = 0; //TODO check if necessary the seq in this msg
        actInfo.max_selectable = 0;
        // we use selectable items info to store in it the action that compose this timed
        for (std::string act : timedMap.second->getInnerActionsNames()) {
            actInfo.inner_actions.push_back(act);
            auto margin = timedMap.second->getActionMargins(act);
            actInfo.before_margins.push_back(margin.first);
            actInfo.after_margins.push_back(margin.second);
        }

        _actionsInfoVect.push_back(actInfo);
        
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

bool ROSEE::RosServiceHandler::actionsInfoCallback(
    rosee_msg::ActionsInfo::Request& request,
    rosee_msg::ActionsInfo::Response& response) {
    
    //here we only send the actionsInfo vector, it is better to build it not in this clbk
    
    for (auto &act : _actionsInfoVect) {
        act.stamp = ros::Time::now();
    }
    response.actionsInfo = _actionsInfoVect;
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
