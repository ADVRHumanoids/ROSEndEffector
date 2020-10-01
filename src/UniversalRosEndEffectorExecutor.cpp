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

#include <ros_end_effector/UniversalRosEndEffectorExecutor.h>


ROSEE::UniversalRosEndEffectorExecutor::UniversalRosEndEffectorExecutor ( std::string ns ) : _nh ( ns ) {

    if ( ! _nh.getParam ( "/rate", _rate ) ) {
        ROS_INFO_STREAM ( "Ros parameter for rate not found, I'm setting the default rate of 100 Hz" );
        _rate = 100.0;
    }
    _period = 1.0 / _rate;
    _loop_timer = _nh.createTimer ( ros::Duration ( _period ),
                                    &UniversalRosEndEffectorExecutor::timer_callback,
                                    this, false, false );
    _time = 0.0;

    ROSEE::Parser p ( _nh );
    p.init (); //TBD check return
    p.printEndEffectorFingerJointsMap();

    // retrieve the ee interface
    _ee = std::make_shared<ROSEE::EEInterface> ( p );
    ROS_INFO_STREAM ( "Fingers in EEInterface: " );
    for ( auto& f : _ee->getFingers() ) {
        ROS_INFO_STREAM ( f );
    }
    
    folderForActions = p.getActionPath();
    if ( folderForActions.size() == 0 ){ //if no action path is set in the yaml file...
        folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/" + _ee->getName();
    }
    
    _all_joints =_ee->getActuatedJoints();

    // prepare joint state publisher
    std::string jstate_topic_name  = "joint_commands";
    const int jstate_queue = 10;

    _joint_state_pub = _nh.advertise<sensor_msgs::JointState> ( jstate_topic_name, jstate_queue );

    _joint_num = _ee->getActuatedJointsNum();
    _js_msg.name.resize ( _joint_num );
    _js_msg.position.resize ( _joint_num );
    _js_msg.velocity.resize ( _joint_num );
    _js_msg.effort.resize ( _joint_num );

    // allocate HAL TBD get from parser the lib to load
    _hal = std::make_shared<ROSEE::DummyHal> ( _ee );

    // filter TBD select filter profile
    const double DAMPING_FACT = 1.0;
    const double BW_MEDIUM = 2.0;
    double omega = 2.0 * M_PI * BW_MEDIUM;

    _filt_q.setDamping ( DAMPING_FACT );
    _filt_q.setTimeStep ( _period );
    _filt_q.setOmega ( omega );

    // initialize references
    _qref.resize ( _joint_num );
    // TBD init from current position reference
    _qref.setZero();
    // reset filter
    _filt_q.reset ( _qref );

    // primitives
    init_grapsing_primitive();
    
    //services, be sure to have mapActionHandler filled (it is done in init_grapsing_primitive)
    _ros_service_handler = std::make_shared<RosServiceHandler>(&_nh, mapActionHandlerPtr);
    _ros_service_handler->init(_ee->getFingers().size());
    
    // actions
    std::string actionGraspingCommandName;
    _nh.param<std::string>("/rosee/rosAction_grasping_command", actionGraspingCommandName, "action_command");
    _ros_action_server = std::make_shared<RosActionServer> (actionGraspingCommandName , &_nh);
    timed_requested = false;
    timed_index = -1;

    // this should be done by hal?
    init_robotState_sub();
}

bool ROSEE::UniversalRosEndEffectorExecutor::init_grapsing_primitive() {

    // get all action in the handler
    mapActionHandlerPtr = std::make_shared<ROSEE::MapActionHandler>();
    mapActionHandlerPtr->parseAllActions(folderForActions);

    // pinch tight
    _pinchParsedMap = mapActionHandlerPtr->getPrimitiveMap("pinchTight");
    
    // pinch loose
    if (mapActionHandlerPtr->getPrimitiveMap(ROSEE::ActionPrimitive::Type::PinchLoose).size()>0) {
        //another method to get the map
        _pinchLooseParsedMap = mapActionHandlerPtr->getPrimitiveMap(ROSEE::ActionPrimitive::Type::PinchLoose).at(0);
    } 
    
    // trig, tip flex and fing flex
    _trigParsedMap = mapActionHandlerPtr->getPrimitiveMap("trig");
    _tipFlexParsedMap = mapActionHandlerPtr->getPrimitiveMap("tipFlex");
    _fingFlexParsedMap = mapActionHandlerPtr->getPrimitiveMap("fingFlex");

    // NOTE maps useful just to recap
    ROS_INFO_STREAM ( "PINCHES-TIGHT:" );
    for ( auto &i : _pinchParsedMap ) {
        i.second->print();
    }
    ROS_INFO_STREAM ( "PINCHES-LOOSE:" );
    for ( auto &i : _pinchLooseParsedMap ) {
        i.second->print();
    }
    ROS_INFO_STREAM ( "TRIGGERS:" );
    for ( auto &i : _trigParsedMap ) {
        i.second->print();
    }
    ROS_INFO_STREAM ( "TIP FLEX:" );
    for ( auto &i : _tipFlexParsedMap ) {
        i.second->print();
    }
    ROS_INFO_STREAM ( "FINGER FLEX:" );
    for ( auto &i : _fingFlexParsedMap ) {
        i.second->print();
    }
    
    // composed actions
    _graspParsed = mapActionHandlerPtr->getGeneric("grasp");
    
    // recap
    ROS_INFO_STREAM ( "GRASP:" );
    if (_graspParsed != nullptr) {
        _graspParsed->print();
    }
    
    return true;
}

//**************************** TODO this should be in the hal? **********************************************//
void ROSEE::UniversalRosEndEffectorExecutor::init_robotState_sub () {

    //to get joint state from gazebo, if used
    std::string topic_name_js;
    _nh.param<std::string>("/rosee/joint_states_topic", topic_name_js, "/ros_end_effector/joint_states");
    
    ROS_INFO_STREAM ( "Getting joint pos from '" << topic_name_js << "'" );
    
    jointPosSub = _nh.subscribe (topic_name_js, 1, 
                                 &ROSEE::UniversalRosEndEffectorExecutor::jointStateClbk, this);
}

void ROSEE::UniversalRosEndEffectorExecutor::jointStateClbk(const sensor_msgs::JointStateConstPtr& msg) {
    
    //store only joint pos now, this should not be here anyaway...
    //HACK if gazebo is used, here we store all joint info received, also the fixed and not actuated 
    // but anyway they will not be used, but check if size is used.
    for (int i=0; i< msg->name.size(); i++) {
        std::vector <double> one_dof { msg->position.at(i) };
        jointPos[msg->name.at(i)] = one_dof;
        
    }
}
//**************************** above this should be in the hal? **********************************************//


// set q ref, similarly to pinch and grasp clbk
bool ROSEE::UniversalRosEndEffectorExecutor::updateGoal() {
    
    rosee_msg::ROSEEActionControl goal = _ros_action_server->getGoal();
    
    timed_requested = false;
    
    if (goal.action_type != ROSEE::Action::Type::Timed){
        if (goal.percentage < 0 || goal.percentage > 1) { 
            ROS_ERROR_STREAM ( "Received an action-goal with percentage " << goal.percentage << 
                " Please insert a value between 0 (for 0%) and 1 (for 100%) ");
            _ros_action_server->abortGoal("Percentage not valid");
            return false;
            
        }
    }
    
    // we need this as global member because in send_feedback we need it...
    //JointsInvolvedCount joint_involved_mask;
    
    //Be sure that action_type of goal is defined well in msg, it is not a real enum
    switch (goal.action_type) {
        
    case ROSEE::Action::Type::Primitive :
    {
        ROSEE::ActionPrimitive::Ptr primitive = mapActionHandlerPtr->getPrimitive (goal.action_name, goal.selectable_items);
        
        if (primitive == nullptr) {
            //error message already printed in getPrimitive
            _ros_action_server->abortGoal("Primitive Action not found");
            return false;
        }
        
        joint_position_goal = primitive->getJointPos();
        joint_involved_mask = primitive->getJointsInvolvedCount();

        break;
    }
    case ROSEE::Action::Type::Generic : // same thing as composed
    case ROSEE::Action::Type::Composed :
    {
        ROSEE::ActionGeneric::Ptr generic = mapActionHandlerPtr->getGeneric(goal.action_name);
        
        if (generic == nullptr) {
            //error message already printed in getGeneric
            _ros_action_server->abortGoal("Generic Action not found");
            return false;
        }
        
        joint_position_goal = generic->getJointPos();
        joint_involved_mask = generic->getJointsInvolvedCount();
        
        break;
    }
    case ROSEE::Action::Type::Timed : {
        // here we take the first of the timed action and we set refs for it
        //TODO first time margin (before) is not considered?

        timedAction = mapActionHandlerPtr->getTimed(goal.action_name);
        
        if (timedAction == nullptr) {
            //error message already printed in getTimed
            _ros_action_server->abortGoal("Timed Action not found");
            return false;
        }
        
        timed_requested = true;
        timed_index = 0;
        joint_position_goal = timedAction->getAllJointPos().at(0);
        joint_involved_mask = timedAction->getJointCountAction(
            timedAction->getInnerActionsNames().at(0));

        goal.percentage = 1; //so we are sure it is set, for timed is always 100%
        
        break;
    }
    case ROSEE::Action::Type::None :
    {
        ROS_ERROR_STREAM ( "Received an action-goal of type None (" << goal.action_type << 
            ") which is a no-type. Please use valid Action::Type ");
        _ros_action_server->abortGoal("Invalid type NONE");
        return false;
        break;
    }
    default : {
        ROS_ERROR_STREAM ( "Received an action-goal of type " << goal.action_type << 
            " which I do not recognize. Please use valid Action::Type ");
        _ros_action_server->abortGoal("Invalid type " + std::to_string(goal.action_type) );
        return false;
    }
    } 
    
    return updateRefGoal(goal.percentage);
    
}

bool ROSEE::UniversalRosEndEffectorExecutor::updateRefGoal(double percentage) {
    
    normGoalFromInitialPos = 0;
    for ( auto it : joint_involved_mask ) {

        if ( it.second  != 0 ) {
            int id = -1;
            _ee->getInternalIdForJoint ( it.first, id );
            
            if( id >= 0 ) {
                // NOTE assume single joint
                _qref[id] = joint_position_goal.at ( it.first ).at ( 0 ) * percentage;
                // to give the % as feedback, we store the initial distance from the goal
                //TODO take care that initially jointPos can be empty...
                normGoalFromInitialPos +=  pow (_qref[id] - jointPos.at(it.first).at(0), 2 )  ;
            }
            else {
                ROS_WARN_STREAM ( "Trying to move Joint: " << it.first << " with ID: " << id );
                _ros_action_server->abortGoal("Invalid Joint id" );
                return false;
            }
        }
    }
    normGoalFromInitialPos = sqrt(normGoalFromInitialPos); 
    
    if (timed_requested) {
        
        //we are in this function BEFORE the begin of execution of a inner... so
        //we set the toWait to (afterMargin of previous inner + beforeMargin of actual (todo) inner)
        //except when timed_index == 0 where we have only the before
        //the after margin of last inner is not considered.
        if (timed_index == 0) {
            msToWait = timedAction->getAllActionMargins().at(timed_index).first;
            
        } else if (timed_index < timedAction->getInnerActionsNames().size()) {
            msToWait = timedAction->getAllActionMargins().at(timed_index-1).second + 
                     timedAction->getAllActionMargins().at(timed_index).first;
        }
        msToWait *= 1000;
        timer.reset();
    }
    
    return true;
}

double ROSEE::UniversalRosEndEffectorExecutor::sendFeedbackGoal(std::string currentAction) {
    //norm between goal and initial (when the goal arrived) position, but only considering the jointInvolved
    
    double actualNorm = 0;
    
    for ( auto it : joint_involved_mask ) {

        if ( it.second  != 0 ) {
            int id = -1;
            _ee->getInternalIdForJoint ( it.first, id );
            
            if( id >= 0 ) {
                // NOTE assume single joint
                //TODO qref or qref_filtered?
                actualNorm += pow ( _qref[id] - jointPos.at(it.first).at(0) , 2 );
            }
            
            else {
                ROS_ERROR_STREAM ( "YOU SHOULD NOT BE HERE, previous error should stop execution");
            }
        }
    }
    actualNorm = sqrt(actualNorm);
    
    double actualCompletationPercentage;

    if (actualNorm < _ros_action_server->getWantedNormError()) { 
        actualCompletationPercentage = 100;
    } else {
        
    //NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
        actualCompletationPercentage =
        (((actualNorm - normGoalFromInitialPos) * (100-0)) / (0 - normGoalFromInitialPos)) + 0;
    }

    _ros_action_server->sendFeedback(actualCompletationPercentage, currentAction);
    return actualCompletationPercentage;
}


void ROSEE::UniversalRosEndEffectorExecutor::fill_publish_joint_states() {

    _js_msg.header.stamp = ros::Time::now();
    _js_msg.header.seq = _seq_id++;

    int c = 0;
    for ( auto& f : _ee->getFingers() ) {

        _ee->getActuatedJointsInFinger ( f, _joints );

        double value = 0;
        for ( auto& j : _joints ) {

            _js_msg.name[c] = j;

            _hal->getMotorPosition ( j, value );
            _js_msg.position[c] = value;

            _hal->getMotorVelocity ( j, value );
            _js_msg.velocity[c] = value;

            _hal->getMotorEffort ( j, value );
            _js_msg.effort[c] = value;

            c++;
        }
        _joints.clear();
    }

    _joint_state_pub.publish ( _js_msg );
}

void ROSEE::UniversalRosEndEffectorExecutor::set_references() {

    //TODO
    //_qref_filtered = _filt_q.process ( _qref );
    _qref_filtered = _qref ;
    int id = -1;

    for ( const auto& j : _all_joints ) {

        _ee->getInternalIdForJoint ( j, id );
        _hal->setPositionReference ( j, _qref_filtered[id] );
    }

}


void ROSEE::UniversalRosEndEffectorExecutor::timer_callback ( const ros::TimerEvent& timer_ev ) {

    //TODO check the order of these functions...
    
    _hal->sense();

    fill_publish_joint_states();
    
    //this is true only when a new goal has arrived... so new goal ovewrite the old one
    if (_ros_action_server->hasNewGoal()) {
        
        updateGoal(); //store jp of goal and update qref
        
    } else if (_ros_action_server->hasGoal()) {

        if (! timed_requested ) {

            if (sendFeedbackGoal() >= 100) {
                
                _ros_action_server->sendComplete();
            }
            
        } else { //a timed is running
            
            if ( sendFeedbackGoal(timedAction->getInnerActionsNames().at(timed_index)) >= 100 ) {
                
                if (timed_index == timedAction->getInnerActionsNames().size()-1)  {
                // completed all the timed action (the last inner has finished)

                    _ros_action_server->sendComplete();
                    timed_requested = false;
                    
                } else if ( 
                    timer.elapsed_time<double, std::chrono::milliseconds>() >
                        (timedAction->getAllActionMargins().at(timed_index).second*1000) ) {
                    
                // if the time passed is greater than the .after margin of the last executed,
                // then pass to the other action. This is done so we continue to send feedback
                // of a inner (with value 100) until the margin after is passed. If we are here,
                // a inner action (but not the last one) is completed (included after margin),
                // so we have to execute the next one now
                    
                    timed_index++;
                    joint_position_goal = timedAction->getAllJointPos().at(timed_index);
                    joint_involved_mask = timedAction->getAllJointCountAction().at(timed_index);
                    updateRefGoal();
                    
                } 
            }
        }  
    }


    if (timed_requested) {
        //TODO is better a mini state machine to deal with timed action?
        if (timer.elapsed_time<double, std::chrono::milliseconds>()  >  msToWait )  {
            //TODO fow now it is this function that make robot moves, and not _hal->move()
            set_references(); 
                
        } else {
            ROS_INFO_STREAM ("Waiting time to execute action...");
        }
            
        
    } else {
        //TODO fow now it is this function that make robot moves, and not _hal->move()
        set_references(); 
    }

    _hal->move();

    // update time
    _time += _period;
}


void ROSEE::UniversalRosEndEffectorExecutor::spin() {

    _loop_timer.start();

    ROS_INFO_STREAM ( "Started looping @ " << 1./_period << "Hz" );

    ros::spin();
}



ROSEE::UniversalRosEndEffectorExecutor::~UniversalRosEndEffectorExecutor() {

}
