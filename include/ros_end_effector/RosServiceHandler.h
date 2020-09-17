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
#include <ros_end_effector/MapActionHandler.h>
#include <rosee_msg/ActionsInfo.h>
#include <rosee_msg/SelectablePairInfo.h>


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
    RosServiceHandler(ros::NodeHandle *nh, ROSEE::MapActionHandler::Ptr);
    bool init(unsigned int nFinger);

    
private:
    
    MapActionHandler::Ptr _mapActionHandler;
    ros::NodeHandle* _nh;

    std::vector<rosee_msg::ActionInfo> _allActionsInfoVect; //redundant, but faster in service callback
    std::vector<rosee_msg::ActionInfo> _primitiveActionsInfoVect;
    std::vector<rosee_msg::ActionInfo> _genericActionsInfoVect;
    std::vector<rosee_msg::ActionInfo> _timedActionsInfoVect;

    ros::ServiceServer _server_actionsInfo;
    ros::ServiceServer _server_selectablePairInfo;
    
    bool actionsInfoCallback ( rosee_msg::ActionsInfo::Request& request, rosee_msg::ActionsInfo::Response& response);
    bool selectablePairInfoCallback ( rosee_msg::SelectablePairInfo::Request& request, rosee_msg::SelectablePairInfo::Response& response);


};

} //end namespace

#endif // ROSSERVICEHANDLER_H
