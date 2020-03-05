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

#include <ROSEndEffector/RosActionServer.h>

ROSEE::RosActionServer::RosActionServer (std::string rosActionName, ros::NodeHandle* nh) :
    _actionServer(*nh, rosActionName, false) {
    
    this->nh = nh;
    this->rosActionName = rosActionName;
    goalInExecution = false;
    newGoal = false;
    
    _actionServer.registerGoalCallback (boost::bind(&RosActionServer::goalReceivedCallback, this));
    _actionServer.registerPreemptCallback (boost::bind (&RosActionServer::preemptReceivedCallback, this));
    
    _actionServer.start();
    
}

bool ROSEE::RosActionServer::hasGoal () {
    return goalInExecution;
}

bool ROSEE::RosActionServer::hasNewGoal () {
    return newGoal;
}

rosee_msg::ROSEEActionControl ROSEE::RosActionServer::getGoal() { 
    newGoal = false;
    return actionControlMsg;
}


void ROSEE::RosActionServer::goalReceivedCallback() {
    
    ROS_INFO_STREAM ("ROSACTION SERVER received goal");
    goalInExecution = true;
    newGoal = true;
    this->actionControlMsg = _actionServer.acceptNewGoal()->goal_action;

}

void ROSEE::RosActionServer::preemptReceivedCallback() {
    
    ROS_INFO_STREAM("ROSACTION SERVER Preempted old goal");
    goalInExecution = false;
    newGoal = false;
    // set the action state to preempted
    _actionServer.setPreempted();
    
}

void ROSEE::RosActionServer::sendFeedback(double completation_percentage) {
    
    ROS_INFO_STREAM("ROSACTION SERVER Sending feedback " << completation_percentage );

    rosee_msg::ROSEECommandFeedback feedback;
    feedback.completation_percentage = completation_percentage;
    
    _actionServer.publishFeedback(feedback);
    
}

void ROSEE::RosActionServer::sendComplete (rosee_msg::ROSEEActionControl msg)  {
    
    ROS_INFO_STREAM("ROSACTION SERVER Sending final result completed ");
    goalInExecution = false;
    rosee_msg::ROSEECommandResult result;
    result.completed_action = msg;
    
    _actionServer.setSucceeded ( result ); 

}

