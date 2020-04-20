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

#include <ROSEndEffector/Parser.h>
#include <ROSEndEffector/EEInterface.h>
#include <ROSEndEffector/EEHal.h>
#include <ROSEndEffector/DummyHal.h>
#include <ROSEndEffector/Utils.h>
#include <ROSEndEffector/MapActionHandler.h>
#include <ROSEndEffector/YamlWorker.h>
#include <ROSEndEffector/RosActionServer.h>

#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionComposed.h>

/// new messages from rosee_msg package
//for services to gui
#include <rosee_msg/ActionsInfo.h> //service
#include <rosee_msg/ActionInfo.h>  //message
#include <rosee_msg/SelectablePairInfo.h>  //message
#include <rosee_msg/ROSEEActionControl.h> //msg
#include <rosee_msg/ROSEECommandAction.h> //action

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

    void fill_publish_joint_states();
    
    void set_references();

    bool init_grapsing_primitive();
    
    bool init_action_server();
    
    bool init_actionsInfo_services() ;
    
    bool actionsInfoCallback (rosee_msg::ActionsInfo::Request& request,
        rosee_msg::ActionsInfo::Response& response);
    
    bool selectablePairInfoCallback( rosee_msg::SelectablePairInfo::Request& request,
                                     rosee_msg::SelectablePairInfo::Response& response);

    
    ros::NodeHandle _nh;
    ros::Timer _loop_timer;

    double _time, _period, _rate;

    ROSEE::EEInterface::Ptr _ee;

    ros::Publisher _joint_state_pub;
    sensor_msgs::JointState _js_msg;

    int _joint_num = 0;
    int _seq_id = 0;

    ROSEE::EEHal::Ptr _hal;

    std::vector<std::string> _all_joints;
    std::vector<std::string> _joints;

    ros::Subscriber _sub_grasp, _sub_pinch, _sub_trigger, sub_finger_flextion, sub_tip_flextion;

    Eigen::VectorXd _qref, _qref_filtered;
    
    ROSEE::Utils::SecondOrderFilter<Eigen::VectorXd> _filt_q;
    
    // grasping primitives maps
    // TODO still needed? now we have the handler that store them...
    std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> _pinchParsedMap;
    std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> _pinchLooseParsedMap;
    std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> _trigParsedMap;
    std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> _tipFlexParsedMap;
    std::map<std::set<std::string>, ROSEE::ActionPrimitive::Ptr> _fingFlexParsedMap;
    
    std::shared_ptr<ROSEE::ActionGeneric> _graspParsed;
    
    MapActionHandler mapActionHandler;
    
    //for service info to gui 
    std::vector<rosee_msg::ActionInfo> _actionsInfoVect;
    ros::ServiceServer _ros_server_actionsInfo;
    ros::ServiceServer _ros_server_selectablePairInfo;
    
    std::shared_ptr <RosActionServer> _ros_action_server;
    
    // we need this as global member because in send_feedback we need it...
    ROSEE::JointsInvolvedCount joint_involved_mask;
    ROSEE::JointPos joint_position_goal;
    double normGoalFromInitialPos;
    
    bool updateGoal(); //the "new" pinch/grasp callback (now used for all actions)
    bool updateRefGoal(double percentage = 1.0);
    double sendFeedbackGoal(std::string currentAction = "");
    bool update_send_timed();
    bool timed_requested;
    std::shared_ptr<ROSEE::ActionTimed> timedAction;
    unsigned int timed_index;
    ROSEE::Utils::Timer<> timer; //for time margins of timed action
    double msToWait;

    
    //TODO this should be done by hal?
    // ALSO todo get state from gazebo if it is used...
    void init_robotState_sub();
    ROSEE::JointPos jointPos;
    ros::Subscriber jointPosSub;
    void jointStateClbk(const sensor_msgs::JointStateConstPtr& msg);
    
    
    

    

};

}


#endif //__ROSEE_UNIVERSAL_ROS_END_EFFECTOR_EXECUTOR_
