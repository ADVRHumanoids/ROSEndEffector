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
    
    _ros_action_server = std::make_shared<RosActionServer> ("actionServer" , &_nh);
}

//**************************** TODO this should be in the hal? **********************************************//
void ROSEE::UniversalRosEndEffectorExecutor::init_robotState_sub () {

    jointPosSub = _nh.subscribe ("joint_states", 1, &ROSEE::UniversalRosEndEffectorExecutor::jointStateClbk, this);
}

void ROSEE::UniversalRosEndEffectorExecutor::jointStateClbk(const sensor_msgs::JointStateConstPtr& msg) {
    
    //store only joint pos now, this should not be here anyaway...
    for (int i=0; i< msg->name.size(); i++) {
        std::vector <double> one_dof {msg->position.at(i)};
        jointPos[msg->name.at(i)] = one_dof;
        
    }
}
//**************************** above this should be in the hal? **********************************************//

// set q ref, similarly to pinch and grasp clbk
void ROSEE::UniversalRosEndEffectorExecutor::setQRef() {
    
    rosee_msg::ROSEEActionControl goal = _ros_action_server->getGoal();
    //TODO for the moment we take all joint pos... take only the one used like pinchCallback and graspCallback
    //HACK only getprimitive now
    //TODO do in branch tori a getPrimitive with second arg a vector and not a set
    std::set <std::string> setConverted ;
    setConverted.insert (goal.selectable_items.begin(), goal.selectable_items.end() );
    ROSEE::ActionPrimitive::Ptr primitive = mapActionHandler.getPrimitive (goal.action_name, setConverted);
    ROSEE::JointPos jp = primitive->getJointPos();
    
    JointsInvolvedCount pinch_joint_involved_mask = primitive->getJointsInvolvedCount();

    for ( auto it : pinch_joint_involved_mask ) {

        if ( it.second  != 0 ) {
            int id = -1;
            _ee->getInternalIdForJoint ( it.first, id );
            
            if( id >= 0 ) {
                // NOTE assume single joint
                _qref[id] = jp.at ( it.first ).at ( 0 ) * goal.percentage;
            }
            else {
                ROS_WARN_STREAM ( "Trying to move Joint: " << it.first << " with ID: " << id );
            }
        }
    }

    
}


bool ROSEE::UniversalRosEndEffectorExecutor::init_actionsInfo_services() {
        
    for (auto primitiveContainers : mapActionHandler.getAllPrimitiveMaps() ) {
        
        rosee_msg::ActionInfo actInfo;
        actInfo.action_name = primitiveContainers.first;
        //TODO define name topics elsewhere?
        actInfo.topic_name = _nh.getNamespace() + "/" + actInfo.action_name;
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
        //TODO define name topics elsewhere?
        actInfo.topic_name = _nh.getNamespace() + "/" + actInfo.action_name;
        actInfo.seq = 0; //TODO check if necessary the seq in this msg
        //Generic action has always no thing to select UNTIL NOW
        actInfo.max_selectable = 0;

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
    
    if (_ros_action_server->hasGoal()) {
        setQRef();
    }

    // filter references
    set_references();

    _hal->move();
    
    if (_ros_action_server->hasGoal()) {
        //norm between goal and actual position

        double norm = 0;

        for ( const auto& j : _all_joints ) {
            int id = -1;
            _ee->getInternalIdForJoint ( j, id );
            //OR _qref_filtered ??
            norm += (_qref[id] * _qref[id]) - (jointPos.at(j).at(0) * jointPos.at(j).at(0));
        }
        //TODO... send not norm but percentage complete...
        _ros_action_server->sendFeedback(norm);
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
