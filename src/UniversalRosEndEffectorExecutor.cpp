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
#include <ROSEndEffector/YamlWorker.h>


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
    
    // services (only for gui now)
    init_actionsInfo_services();

}

void ROSEE::UniversalRosEndEffectorExecutor::graspCallback ( const ros_end_effector::EEGraspControlConstPtr& msg ) {

    ROSEE::JointPos grasp_js = _graspParsedMap.getJointPos();
    // get the joints involved bool vector
    JointsInvolvedCount grasp_joint_involved_mask = _graspParsedMap.getJointsInvolvedCount();
     
    for( auto it : grasp_joint_involved_mask ) {
        
        if ( it.second  != 0 ) {
            int id = -1;
            _ee->getInternalIdForJoint ( it.first, id );
            // NOTE assume single joint
            _qref[id] = grasp_js.at ( it.first ).at ( 0 ) * msg->percentage;
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
                
                if( id > 0 ) {
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
    
    std::string folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/" + _ee->getName() + "/primitives/";
    std::string folderForActionsComposed = ROSEE::Utils::getPackagePath() + "/configs/actions/" + _ee->getName() + "/generics/";

    MapActionHandler handler;
    handler.parseAllPrimitives( folderForActions );
    //TODO check if store in this class all the maps is necessary...
    _pinchParsedMap = handler.getPrimitive("pinchStrong");
    if (handler.getPrimitive(ROSEE::ActionPrimitive::Type::PinchWeak).size()>0) {//another method to get the map
        _pinchWeakParsedMap = handler.getPrimitive(ROSEE::ActionPrimitive::Type::PinchWeak).at(0);
    } 
    
    _trigParsedMap = handler.getPrimitive("trig");

    // old way to get maps
    //_pinchParsedMap = yamlWorker.parseYamlPrimitive ( folderForActions + "pinchStrong.yaml",ROSEE::ActionPrimitive::Type::PinchStrong );

    //pinch Weak
    _pinchWeakParsedMap = yamlWorker.parseYamlPrimitive ( folderForActions + "pinchWeak.yaml", ROSEE::ActionPrimitive::Type::PinchWeak );

    //trig
    _trigParsedMap = yamlWorker.parseYamlPrimitive ( folderForActions + "trig.yaml", ROSEE::ActionPrimitive::Type::Trig );

    //tipFlex
    _tipFlexParsedMap = yamlWorker.parseYamlPrimitive ( folderForActions + "tipFlex.yaml", ROSEE::ActionPrimitive::Type::TipFlex );

    //fingFlex
    _fingFlexParsedMap = yamlWorker.parseYamlPrimitive ( folderForActions + "fingFlex.yaml", ROSEE::ActionPrimitive::Type::FingFlex );

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
    _graspParsedMap = yamlWorker.parseYamlComposed (folderForActionsComposed + "grasp.yaml");
    ROS_INFO_STREAM ( "GRASP" );
    _graspParsedMap.print();

    // generate the subscribers and services

    if ( !_graspParsedMap.empty() ) {

        _sub_grasp = _nh.subscribe<ros_end_effector::EEGraspControl> ( "grasp",
                     1,
                     &ROSEE::UniversalRosEndEffectorExecutor::graspCallback,
                     this
                                                                     );
    }

    if ( !_pinchParsedMap.empty() ) {

        _sub_pinch = _nh.subscribe<ros_end_effector::EEPinchControl> ( "pinch",
                     1,
                     &ROSEE::UniversalRosEndEffectorExecutor::pinchCallback,
                     this
                                                                     );
    }

    return true;
}

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
    
    //request.action_name not used now
    auto it = pairedMap.find(request.element_name) ;
    
    if (it == pairedMap.end()) {
        return false;
    }
    
    for (auto fing : it->second ) {
        response.pair_elements.push_back (fing);
    }
    
    return true;
        
}

void ROSEE::UniversalRosEndEffectorExecutor::findPossiblePinchPairs () {
                                                 
    for (auto it : _pinchParsedMap) {
    
        for (auto fing : it.first) { //.first is a set
            
            //TODO we will insert all the set as value, this means that also will include the key itself,
            // with the meaning : finger_1 can pinch with finger_1: 
            if (pairedMap.count(fing) == 0 ) {
               pairedMap.insert(std::make_pair(fing, it.first)); 
            } else {
                pairedMap.at(fing).insert (it.first.begin(), it.first.end());
            }
        }
    }
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

    _hal->sense();

    fill_publish_joint_states();

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



