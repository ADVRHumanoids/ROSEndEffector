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

#include <ROSEndEffector/UniversalRosEndEffectorExecutor.h>


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
    _ee->getActuatedJoints ( _all_joints );


    // prepare joint state publisher
    std::string jstate_topic_name  = "joint_states";
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
    init_grapsing_primitive_subscribers();
    
    // actions
    init_action_server();
    timed_requested = false;
    timed_index = -1;

    
    // services (only for gui now)
    init_actionsInfo_services();

    // this should be done by hal?
    init_robotState_sub();
}

void ROSEE::UniversalRosEndEffectorExecutor::graspCallback ( const ros_end_effector::EEGraspControlConstPtr& msg ) {

    ROSEE::JointPos grasp_js = _graspParsed->getJointPos();
    // get the joints involved bool vector
    JointsInvolvedCount grasp_joint_involved_mask = _graspParsed->getJointsInvolvedCount();
     
    for( auto it : grasp_joint_involved_mask ) {
        
        if ( it.second  != 0 ) {
            int id = -1;
            _ee->getInternalIdForJoint ( it.first, id );
            
            if( id >= 0 ) {
                // NOTE assume single joint
                _qref[id] = grasp_js.at ( it.first ).at ( 0 ) * msg->percentage;
            }
            else {
                    ROS_WARN_STREAM ( "Trying to move Joint: " << it.first << " with ID: " << id );
            }

        }
        
    }

}


void ROSEE::UniversalRosEndEffectorExecutor::pinchCallback ( const ros_end_effector::EEPinchControlConstPtr& msg ) {

    std::vector<int> ids;

    std::set<std::string> pinch_set;
    pinch_set.insert ( msg->finger_pinch_1 );
    pinch_set.insert ( msg->finger_pinch_2 );

    if ( _pinchParsedMap.count ( pinch_set ) ) {

        ROSEE::ActionPrimitive::Ptr p = _pinchParsedMap.at ( pinch_set );
        // NOTE take the best pinch for now
        ROSEE::JointPos pinch_js = p->getAllJointPos().at ( 0 );

        // get the joints involved bool vector
        JointsInvolvedCount pinch_joint_involved_mask = p->getJointsInvolvedCount();

        for ( auto it : pinch_joint_involved_mask ) {

            if ( it.second  != 0 ) {
                int id = -1;
                _ee->getInternalIdForJoint ( it.first, id );
                
                if( id >= 0 ) {
                    // NOTE assume single joint
                    _qref[id] = pinch_js.at ( it.first ).at ( 0 ) * msg->percentage;
                }
                else {
                    ROS_WARN_STREAM ( "Trying to move Joint: " << it.first << " with ID: " << id );
                }
            }
        }

    } else {
        
        ROS_ERROR_STREAM ( "finger_pinch_1 :" << msg->finger_pinch_1 << 
                           " and finger_pinch_2 :" << msg->finger_pinch_2 << 
                           " is not a feasible couple for the Pinch Grasping Action" );
    }

}


bool ROSEE::UniversalRosEndEffectorExecutor::init_grapsing_primitive_subscribers() {

    // parse YAML for End-Effector cconfiguration
    ROSEE::YamlWorker yamlWorker ;
    
    std::string folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/" + _ee->getName();
    std::string folderForActionsComposed = ROSEE::Utils::getPackagePath() + "/configs/actions/" + _ee->getName() + "/generics/";

    mapActionHandler.parseAllActions(folderForActions);

    
    //TODO check if store in this class all the maps is necessary...
    _pinchParsedMap = mapActionHandler.getPrimitiveMap("pinchStrong");
    
    if (mapActionHandler.getPrimitiveMap(ROSEE::ActionPrimitive::Type::PinchWeak).size()>0) {
        //another method to get the map
        _pinchWeakParsedMap = mapActionHandler.getPrimitiveMap(ROSEE::ActionPrimitive::Type::PinchWeak).at(0);
    } 
    
    _trigParsedMap = mapActionHandler.getPrimitiveMap("trig");
    _tipFlexParsedMap = mapActionHandler.getPrimitiveMap("tipFlex");
    _fingFlexParsedMap = mapActionHandler.getPrimitiveMap("fingFlex");

    // old way to get maps
    //_pinchParsedMap = yamlWorker.parseYamlPrimitive ( folderForActions + "pinchStrong.yaml",ROSEE::ActionPrimitive::Type::PinchStrong );

    ROS_INFO_STREAM ( "PINCHES-STRONG:" );
    for ( auto &i : _pinchParsedMap ) {
        i.second->print();
    }
    ROS_INFO_STREAM ( "PINCHES-WEAK:" );
    for ( auto &i : _pinchWeakParsedMap ) {
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
    //OLD WAY
    //_graspParsedMap = yamlWorker.parseYamlComposed (folderForActionsComposed + "grasp.yaml");
    _graspParsed = mapActionHandler.getGeneric("grasp");
    ROS_INFO_STREAM ( "GRASP:" );
    if (_graspParsed != nullptr) {
        _graspParsed->print();
    }
    // generate the subscribers and services

    if ( _graspParsed != nullptr ) {

        _sub_grasp = _nh.subscribe<ros_end_effector::EEGraspControl> ( "grasp",
                     1,
                     &ROSEE::UniversalRosEndEffectorExecutor::graspCallback,
                     this
                                                                     );
    }

    if ( !_pinchParsedMap.empty() ) {

        _sub_pinch = _nh.subscribe<ros_end_effector::EEPinchControl> ( "pinchStrong",
                     1,
                     &ROSEE::UniversalRosEndEffectorExecutor::pinchCallback,
                     this
                                                                     );
    }

    return true;
}

bool ROSEE::UniversalRosEndEffectorExecutor::init_action_server () {
    
    _ros_action_server = std::make_shared<RosActionServer> ("action_command" , &_nh);
}

//**************************** TODO this should be in the hal? **********************************************//
void ROSEE::UniversalRosEndEffectorExecutor::init_robotState_sub () {

    //to get joint state from gazebo, if used
    std::string topic_name_js;
    _nh.param<std::string>("/joint_states_topic", topic_name_js, "joint_states");
    
    if (_nh.hasParam("/gazebo_jointStates")) {
        ROS_INFO_STREAM ( "Using gazebo simulation to get joint states..." );
    } else {
        ROS_INFO_STREAM ( "Using rviz simulation to get joint states..." );
    }
    
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
        ROSEE::ActionPrimitive::Ptr primitive = mapActionHandler.getPrimitive (goal.action_name, goal.selectable_items);
        
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
        ROSEE::ActionGeneric::Ptr generic = mapActionHandler.getGeneric(goal.action_name);
        
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

        timedAction = mapActionHandler.getTimed(goal.action_name);
        
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

    if (actualNorm < 0.01) { 
        actualCompletationPercentage = 100;
    } else {
        
    //NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
        actualCompletationPercentage =
        (((actualNorm - normGoalFromInitialPos) * (100-0)) / (0 - normGoalFromInitialPos)) + 0;
    }

    _ros_action_server->sendFeedback(actualCompletationPercentage, currentAction);
    return actualCompletationPercentage;
}

bool ROSEE::UniversalRosEndEffectorExecutor::init_actionsInfo_services() {
        
    for (auto primitiveContainers : mapActionHandler.getAllPrimitiveMaps() ) {
        
        rosee_msg::ActionInfo actInfo;
        actInfo.action_name = primitiveContainers.first;
        actInfo.action_type = ROSEE::Action::Type::Primitive;
        actInfo.actionPrimitive_type = primitiveContainers.second.begin()->second->getPrimitiveType();
        actInfo.ros_action_name = _nh.getNamespace() + "/" + "action_command";
        actInfo.seq = 0; //TODO check if necessary the seq in this msg
        //until now, there is not a primitive that does not have "something" to select
        // (eg pinch has 2 fing, trig one fing, moretips 1 joint...). 
        //Instead generic action has always no thing to select (next for loop)
        actInfo.max_selectable = primitiveContainers.second.begin()->first.size();
        //TODO extract the keys with another mapActionHandler function?
        actInfo.selectable_names =
            ROSEE::Utils::extract_keys_unique(primitiveContainers.second,
                                              _ee->getFingers().size());
        _actionsInfoVect.push_back(actInfo);

    }

    for (auto genericMap : mapActionHandler.getAllGenerics() ) {

        rosee_msg::ActionInfo actInfo;
        actInfo.action_name = genericMap.first;
        actInfo.action_type = genericMap.second->getType();
        actInfo.actionPrimitive_type = ROSEE::ActionPrimitive::Type::None;
        actInfo.ros_action_name = _nh.getNamespace() + "/" + "action_command";
        actInfo.seq = 0; //TODO check if necessary the seq in this msg
        //Generic action has always no thing to select UNTIL NOW
        actInfo.max_selectable = 0;

        _actionsInfoVect.push_back(actInfo);

    }
    
    for (auto timedMap : mapActionHandler.getAllTimeds() ) {
        
        rosee_msg::ActionInfo actInfo;
        actInfo.action_name = timedMap.first;
        actInfo.action_type = timedMap.second->getType();
        actInfo.actionPrimitive_type = ROSEE::ActionPrimitive::Type::None;
        actInfo.ros_action_name = _nh.getNamespace() + "/" + "action_command";
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
    
    _ros_server_actionsInfo = _nh.advertiseService("ActionsInfo", 
        &ROSEE::UniversalRosEndEffectorExecutor::actionsInfoCallback, this);
    
    _ros_server_selectablePairInfo = _nh.advertiseService("SelectablePairInfo", 
        &ROSEE::UniversalRosEndEffectorExecutor::selectablePairInfoCallback, this);
    
    return true;

}


bool ROSEE::UniversalRosEndEffectorExecutor::actionsInfoCallback(
    rosee_msg::ActionsInfo::Request& request,
    rosee_msg::ActionsInfo::Response& response) {
    
    //here we only send the actionsInfo vector, it is better to build it not in this clbk
    
    //TODO timestamp necessary?
    for (auto &act : _actionsInfoVect) {
        act.stamp = ros::Time::now();
    }
    response.actionsInfo = _actionsInfoVect;
    return true;
    
}

bool ROSEE::UniversalRosEndEffectorExecutor::selectablePairInfoCallback(
    rosee_msg::SelectablePairInfo::Request& request,
    rosee_msg::SelectablePairInfo::Response& response) {
    
    std::set<std::string> companionFingers;
    if (request.action_name.compare ("pinchStrong") == 0) {
        companionFingers =
            mapActionHandler.getFingertipsForPinch(request.element_name,
                ROSEE::ActionPrimitive::Type::PinchStrong) ;
                
    } else if (request.action_name.compare ("pinchWeak") == 0) {
        companionFingers =
            mapActionHandler.getFingertipsForPinch(request.element_name,
                ROSEE::ActionPrimitive::Type::PinchWeak) ;
                
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

    _qref_filtered = _filt_q.process ( _qref );
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
            
            if ( sendFeedbackGoal(timedAction->getInnerActionsNames().at(timed_index)) >= 100) {
                
                if (timed_index == timedAction->getInnerActionsNames().size()-1)  {
                    _ros_action_server->sendComplete();
                    timed_requested = false;
                    
                } else {
                    
                    timed_index++;
                    joint_position_goal = timedAction->getAllJointPos().at(timed_index);
                    joint_involved_mask = timedAction->getAllJointCountAction().at(timed_index);
                    updateRefGoal();
                    
                }                      
            }
        }  
    }

    // filter references
    set_references();

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


/***OLDDD
bool ROSEE::UniversalRosEndEffectorExecutor::init_actionsInfo_services() {
        
    //TODO add all action with for looping all the files present in the folder
    rosee_msg::ActionInfo actInfo;
    actInfo.action_name = _pinchParsedMap.begin()->second->getName();
    actInfo.topic_name = _nh.getNamespace() + "/pinch"; //TODO define name topics
    actInfo.seq = 0; //TODO check if necessary the seq in this msg
    actInfo.max_selectable = _pinchParsedMap.begin()->first.size(); //the size of the key set
    actInfo.selectable_names = ROSEE::Utils::extract_keys_unique(_pinchParsedMap,  
                                                                 _ee->getFingers().size());
    _actionsInfoVect.push_back(actInfo);
    findPossiblePinchPairs();
    
    actInfo = rosee_msg::ActionInfo(); //clear container
    actInfo.action_name = _graspParsedMap.getName(); //TODO why this is called map?
    actInfo.topic_name = _nh.getNamespace() + "/grasp"; //TODO define name topics
    actInfo.max_selectable = 0; //zero means no things to select with checkboxes

    actInfo.seq = 0; //TODO check if necessary the seq in this msg

    _actionsInfoVect.push_back(actInfo);
    
    
    //TODO add others when msg and subscribers will be avaialbel (init_grapsing_primitive_subscribers func)

    
    _ros_server_actionsInfo = _nh.advertiseService("ActionsInfo", 
        &ROSEE::UniversalRosEndEffectorExecutor::actionsInfoCallback, this);
    
    _ros_server_selectablePairInfo = _nh.advertiseService("SelectablePairInfo", 
        &ROSEE::UniversalRosEndEffectorExecutor::selectablePairInfoCallback, this);
    
    return true;

}
**/
