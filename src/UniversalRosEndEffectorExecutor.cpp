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
    
    _motors_names =_ee->getActuatedJoints();

    // prepare publisher which publish motor references
    init_motor_reference_pub();
    
    //init q_ref and the filter
    init_qref_filter();

    // primitives
    init_grasping_primitive();
    
    //services, check on mapActionHandlerPtr is done inside the costructor
    _ros_service_handler = std::make_shared<RosServiceHandler>(&_nh, mapActionHandlerPtr, folderForActions+"/generics/");
    _ros_service_handler->init(_ee->getFingers().size());
    
    // actions
    std::string actionGraspingCommandName;
    _nh.param<std::string>("/rosee/rosAction_grasping_command", actionGraspingCommandName, "action_command");
    _ros_action_server = std::make_shared<RosActionServer> (actionGraspingCommandName , &_nh);
    timed_requested = false;
    timed_index = -1;
    
    init_joint_state_sub();
    
}

bool ROSEE::UniversalRosEndEffectorExecutor::init_motor_reference_pub() {
    
    //Fixed topic, Hal will read always from here
    std::string motor_reference_topic  = "motor_reference_pos";
    const int motor_reference_queue = 10;

    //We could use MotorPosition message... but with JointState the hal node has not to include
    //our rosee_msg package, so maybe it is better use official ros JointState
    _motor_reference_pub = _nh.advertise<sensor_msgs::JointState> ( motor_reference_topic, motor_reference_queue );
    _motors_num = _motors_names.size();
    //mr = motor reference
    _mr_msg.name = _motors_names;
    _mr_msg.position.resize ( _motors_num );
    _mr_msg.velocity.resize ( _motors_num );
    _mr_msg.effort.resize ( _motors_num ); 
    
    return true;
}

bool ROSEE::UniversalRosEndEffectorExecutor::init_qref_filter() {
    
    // filter TBD select filter profile
    const double DAMPING_FACT = 1.0;
    const double BW_MEDIUM = 2.0;
    double omega = 2.0 * M_PI * BW_MEDIUM;

    _filt_q.setDamping ( DAMPING_FACT );
    _filt_q.setTimeStep ( _period );
    _filt_q.setOmega ( omega );

    // initialize references
    _qref.resize ( _motors_num );
    _qref_optional.resize ( _motors_num );
    _qref_optional.setZero();
    // TBD init from current position reference
    _qref.setZero();
    // reset filter
    _filt_q.reset ( _qref );
    
    return true;
}


bool ROSEE::UniversalRosEndEffectorExecutor::init_grasping_primitive() {

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

void ROSEE::UniversalRosEndEffectorExecutor::init_joint_state_sub () {

    //Fixed topic, Hal will publish always here
    std::string topic_name_js = "/ros_end_effector/joint_states";
    
    ROS_INFO_STREAM ( "Getting joint pos from '" << topic_name_js << "'" );
    
    _joint_state_sub = _nh.subscribe (topic_name_js, 1, 
                                 &ROSEE::UniversalRosEndEffectorExecutor::joint_state_clbk, this);
}

void ROSEE::UniversalRosEndEffectorExecutor::joint_state_clbk(const sensor_msgs::JointStateConstPtr& msg) {
    
    //We store all what we receive from hal node, to not loose time in the clbk to not consider the not motors
    for (int i=0; i< msg->name.size(); i++) {
        std::vector <double> one_dof { msg->position.at(i) };
        _joint_actual_pos[msg->name.at(i)] = one_dof;
    }
}


// set q ref
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
        
        _motor_position_goal = primitive->getJointPos();
        _motor_involved_mask = primitive->getJointsInvolvedCount();

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
        
        _motor_position_goal = generic->getJointPos();
        _motor_involved_mask = generic->getJointsInvolvedCount();
        
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
        _motor_position_goal = timedAction->getAllJointPos().at(0);
        _motor_involved_mask = timedAction->getJointCountAction(
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

    //WE reset this so if it is not used it stay to zero
    _qref_optional.setZero();
    if (goal.optional_motors_names.size() != 0) {
        readOptionalCommands(goal.optional_motors_names, goal.optional_motors_commands);
    }
    
    return updateRefGoal(goal.percentage);
    
}

bool ROSEE::UniversalRosEndEffectorExecutor::readOptionalCommands(
        std::vector<std::string> motors_names, std::vector<double> motors_commands) {
    
    bool flag = true;
    
    if (motors_names.size() != motors_commands.size() &&
        motors_names.size() != _motors_num) {
        ROS_ERROR_STREAM ( "In receiving the goal command, the optional field is formed badly: " 
            << "optional_motors_names and optional_motors_commands and number of motors have different size (" 
            <<  motors_names.size() << " and " << motors_commands.size() << " and " << _motors_num
            << " respectively).  I will ignore the optional command");
        return false;
    } 
    

    for (int i=0; i<motors_names.size(); i++) {
        
        int id =-1;
        _ee->getInternalIdForJoint ( motors_names.at(i), id );
        if( id >= 0 ) {
            // NOTE assume single dof
            _qref_optional[id] = motors_commands.at(i);
        }
        
        else {
            ROS_WARN_STREAM ( "Trying to send an optional command to motor: " << motors_names.at(i) 
            << " which is not defined" );
            flag = false;
        }
    }
    
    //TODO we have to filter also qref_optional ??
    
    return flag;
}

bool ROSEE::UniversalRosEndEffectorExecutor::updateRefGoal(double percentage) {
    
    normGoalFromInitialPos = 0;
    for ( auto it : _motor_involved_mask ) {

        if ( it.second  != 0 ) {
            int id = -1;
            _ee->getInternalIdForJoint ( it.first, id );
            
            if( id >= 0 ) {
                // NOTE assume single joint
                _qref[id] = _motor_position_goal.at ( it.first ).at ( 0 ) * percentage;
                // to give the % as feedback, we store the initial distance from the goal
                //TODO take care that initially jointPos can be empty...
                normGoalFromInitialPos +=  pow (_qref[id] - _joint_actual_pos.at(it.first).at(0), 2 )  ;
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
    
    for ( auto it : _motor_involved_mask ) {

        if ( it.second  != 0 ) {
            int id = -1;
            _ee->getInternalIdForJoint ( it.first, id );
            
            if( id >= 0 ) {
                // NOTE assume single joint
                actualNorm += pow ( _qref[id] - _joint_actual_pos.at(it.first).at(0) , 2 );
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


bool ROSEE::UniversalRosEndEffectorExecutor::publish_motor_reference() {

    _mr_msg.header.stamp = ros::Time::now();
    _mr_msg.header.seq = _seq_id++;

    _qref_filtered = _filt_q.process ( _qref );
    
    int id = -1;
    for ( const auto& motor_name : _motors_names ) {

        //id to put take the qref from the right index in qref
        _ee->getInternalIdForJoint ( motor_name, id );
        _mr_msg.name[id] = motor_name;
        _mr_msg.position[id] =_qref_filtered[id] ;
        _mr_msg.effort[id] = _qref_optional[id];
    }

    _motor_reference_pub.publish ( _mr_msg );
    
    return true;
}

void ROSEE::UniversalRosEndEffectorExecutor::timer_callback ( const ros::TimerEvent& timer_ev ) {
    
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
                    _motor_position_goal = timedAction->getAllJointPos().at(timed_index);
                    _motor_involved_mask = timedAction->getAllJointCountAction().at(timed_index);
                    updateRefGoal();
                    
                } 
            }
        }  
    }


    //if it is a timed, we have to check if we need to still wait to execute the subsequent innet action
    if (timed_requested) {
        if (timer.elapsed_time<double, std::chrono::milliseconds>()  >  msToWait )  {
            publish_motor_reference(); 
                
        } else {
            ROS_INFO_STREAM ("Waiting time to execute timed action...");
        }
            
    } else {
        publish_motor_reference(); 
    }

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
