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

#ifndef ROSACTIONSERVER_H
#define ROSACTIONSERVER_H

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <rosee_msg/ROSEECommandAction.h>

namespace ROSEE {
/**
 * @todo write docs
 */
class RosActionServer {
    
public:
    RosActionServer(std::string rosActionName, ros::NodeHandle *nh);
    ~RosActionServer() {};
    
    rosee_msg::ROSEEActionControl getGoal() ;
    bool hasGoal();
    bool hasNewGoal();

    /** 
     * @brief send Feedback to the client who has requested the goal.
     * 
     * @param completation_percentage the percentae that tells how much we have completed 
     * the action requested
     * @param currentAction current action that is running. If it is empty, it is taken the name
     *    of the action from member \ref actionControlMsg . You should passed not empty string when
     *    dealing with timed action (passing the inner action of the timed one)
     */
    void sendFeedback(double completation_percentage, std::string currentAction) ;
    void sendComplete () ;
    void abortGoal(std::string errorMsg = "");

    
protected:
    ros::NodeHandle* nh;
    std::string rosActionName;
    actionlib::SimpleActionServer <rosee_msg::ROSEECommandAction> _actionServer;
    rosee_msg::ROSEEActionControl actionControlMsg;
    bool goalInExecution;
    bool newGoal;
    
    void goalReceivedCallback ();
    void preemptReceivedCallback ();


};

}

#endif // ROSACTIONSERVER_H