/*
 * Copyright (C) 2019 IIT-HHCM
 * Author: Luca Muratore
 * email:  luca.muratore@iit.it
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef __ROSEE_UNIVERSAL_ROS_END_EFFECTOR_EXECUTOR_
#define __ROSEE_UNIVERSAL_ROS_END_EFFECTOR_EXECUTOR_

#include <memory>
#include <string>

#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>

#include <sensor_msgs/JointState.h>

#include <ros_end_effector/Parser.h>
#include <ros_end_effector/EEInterface.h>
#include <ros_end_effector/Utils.h>
#include <ros_end_effector/MapActionHandler.h>
#include <ros_end_effector/YamlWorker.h>
#include <ros_end_effector/RosActionServer.h>
#include <ros_end_effector/RosServiceHandler.h>

#include <ros_end_effector/GraspingActions/ActionPrimitive.h>
#include <ros_end_effector/GraspingActions/ActionComposed.h>
#include <rosee_msg/HandInfo.h>

namespace ROSEE
{

/**
 * @brief Class representing the ROS executor implementing the unviversal ROS EE concept
 *
 */
class UniversalRosEndEffectorExecutor
{

public:

    typedef std::shared_ptr<UniversalRosEndEffectorExecutor> Ptr;
    typedef std::shared_ptr<const UniversalRosEndEffectorExecutor> ConstPtr;

    UniversalRosEndEffectorExecutor ( std::string ns = "" );
    virtual ~UniversalRosEndEffectorExecutor();

    void spin();

    void timer_callback ( const ros::TimerEvent& timer_ev );

private:

    bool init_motor_reference_pub();
    bool init_qref_filter();    
    void init_joint_state_sub();
    bool init_grasping_primitive();    

    bool publish_motor_reference();
    
    bool updateGoal(); //the "new" pinch/grasp callback (now used for all actions)
    bool readOptionalCommands(std::vector<std::string> motors_names, std::vector<double> motors_commands);
    bool updateRefGoal(double percentage = 1.0);
    double sendFeedbackGoal(std::string currentAction = "");
    bool update_send_timed();
    
    ros::NodeHandle _nh;
    ros::Timer _loop_timer;

    double _time, _period, _rate;

    ROSEE::EEInterface::Ptr _ee;

    ros::Publisher _motor_reference_pub;
    sensor_msgs::JointState _mr_msg;
    int _motors_num = 0;
    int _seq_id = 0;
    
    ROSEE::JointPos _joint_actual_pos;
    ros::Subscriber _joint_state_sub;
    void joint_state_clbk(const sensor_msgs::JointStateConstPtr& msg);

    std::vector<std::string> _motors_names;
    
    std::string folderForActions;

    Eigen::VectorXd _qref, _qref_filtered, _qref_optional;
    
    ROSEE::Utils::SecondOrderFilter<Eigen::VectorXd> _filt_q;
    
    // grasping primitives maps
    // TODO still needed? now we have the handler that store them...
    std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> _pinchParsedMap;
    std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> _pinchLooseParsedMap;
    std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> _trigParsedMap;
    std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> _tipFlexParsedMap;
    std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> _fingFlexParsedMap;
    
    std::shared_ptr<ROSEE::ActionGeneric> _graspParsed;
    
    MapActionHandler::Ptr mapActionHandlerPtr;
    
    std::shared_ptr <RosServiceHandler> _ros_service_handler;
    std::shared_ptr <RosActionServer> _ros_action_server;
    
    ROSEE::JointsInvolvedCount _motor_involved_mask;
    ROSEE::JointPos _motor_position_goal;
    double normGoalFromInitialPos;

    bool timed_requested;
    std::shared_ptr<ROSEE::ActionTimed> timedAction;
    unsigned int timed_index;
    ROSEE::Utils::Timer<> timer; //for time margins of timed action
    double msToWait;

};

} //namespace

#endif //__ROSEE_UNIVERSAL_ROS_END_EFFECTOR_EXECUTOR_
