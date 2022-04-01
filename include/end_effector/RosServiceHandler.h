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

#ifndef ROSSERVICEHANDLER_H
#define ROSSERVICEHANDLER_H

#include <ros/ros.h>
#include <end_effector/MapActionHandler.h>
#include <rosee_msg/GraspingPrimitiveAggregated.h>
#include <rosee_msg/GraspingPrimitiveAggregatedAvailable.h>
#include <rosee_msg/SelectablePairInfo.h>
#include <rosee_msg/GraspingActionsAvailable.h>
#include <rosee_msg/GraspingAction.h>
#include <rosee_msg/MotorPosition.h>
#include <rosee_msg/HandInfo.h>
#include <rosee_msg/NewGenericGraspingAction.h>
#include <rosee_msg/NewGenericGraspingActionSrv.h>


namespace ROSEE {
/**
 * @todo write docs
 */
class RosServiceHandler
{
public:
    /**
     * Default constructor
     */
    RosServiceHandler(ros::NodeHandle *nh, ROSEE::MapActionHandler::Ptr, std::string path2saveYamlGeneric);
    bool init(unsigned int nFinger);

    //this response is filled by UROSEE in the initialization
    rosee_msg::HandInfo::Response handInfoResponse;
private:
    
    MapActionHandler::Ptr _mapActionHandler;
    std::string _path2saveYamlGeneric;
    ros::NodeHandle* _nh;
    ros::ServiceServer _serverPrimitiveAggregated;
    ros::ServiceServer _server_selectablePairInfo;
    ros::ServiceServer _serverGraspingActions;
    ros::ServiceServer _serverNewGraspingAction;
    
    ros::ServiceServer _serverHandInfo;
    
    
    unsigned int nFinger;
    
    bool selectablePairInfoCallback ( rosee_msg::SelectablePairInfo::Request& request, rosee_msg::SelectablePairInfo::Response& response);
    
    bool graspingActionsCallback(rosee_msg::GraspingActionsAvailable::Request& request,   
                                 rosee_msg::GraspingActionsAvailable::Response& response);
  
    rosee_msg::GraspingAction fillGraspingActionMsg(ROSEE::ActionPrimitive::Ptr primitive);
    rosee_msg::GraspingAction fillGraspingActionMsg(ROSEE::ActionGeneric::Ptr generic);
    rosee_msg::GraspingAction fillGraspingActionMsg(ROSEE::ActionTimed::Ptr timed);

    /**
     * @brief Internal function called by each of the fillGraspingActionMsg, it fills the GraspingAction message
     * with the info that are always present in any kind of action (like name and type). 
     * Each specific \ref fillGraspingActionMsg will insert fill the field specific for the action type 
     * (like time margins for timed actions)
     *
     * @param action the action that must be sent as response
     * @param msg the ROS message that must be filled with action info
     */
    void fillCommonInfoGraspingActionMsg(ROSEE::Action::Ptr action, rosee_msg::GraspingAction* msg);

    
    bool primitiveAggregatedCallback(
        rosee_msg::GraspingPrimitiveAggregatedAvailable::Request& request,
        rosee_msg::GraspingPrimitiveAggregatedAvailable::Response& response);

    rosee_msg::GraspingPrimitiveAggregated fillPrimitiveAggregatedMsg(
                        ROSEE::MapActionHandler::ActionPrimitiveMap primitiveMap);
    rosee_msg::GraspingPrimitiveAggregated fillPrimitiveAggregatedMsg(
                        ROSEE::ActionPrimitive::Ptr primitive);
    
    bool handInfoCallback(
        rosee_msg::HandInfo::Request& request,
        rosee_msg::HandInfo::Response& response);

    bool newGraspingActionCallback(
        rosee_msg::NewGenericGraspingActionSrv::Request& request,
        rosee_msg::NewGenericGraspingActionSrv::Response& response);
};

} //end namespace

#endif // ROSSERVICEHANDLER_H
