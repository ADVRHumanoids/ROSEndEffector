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


ROSEE::UniversalRosEndEffectorExecutor::UniversalRosEndEffectorExecutor ( std::string ns ) : _nh(ns) {

    // TBD rate from config
    _period = 1.0 / 100.0;
    _loop_timer = _nh.createTimer(ros::Duration(_period), 
                                  &UniversalRosEndEffectorExecutor::timer_callback, 
                                  this, false, false);
    _time = 0.0;
    
    
    // parse config TBD no hardcode
    ROSEE::Parser p(_nh);
    p.init("/home/lucamuratore/src/ros_end_effector__ws/src/ROSEndEffector/configs/test_ee.yaml");
    p.printEndEffectorFingerJointsMap();
    
    
    // retrieve the ee interface
    _ee = std::make_shared<ROSEE::EEInterface>( p );
    
    _joint_num = _ee->getActuatedJointsNum();

    ROS_INFO_STREAM("Fingers in EEInterface: ");
    for( auto& f : _ee->getFingers() ) {
         ROS_INFO_STREAM(f);
    }
    
    
    // prepare joint state publisher
    std::string jstate_topic_name  = "joint_states";
    const int jstate_queue = 10;
    
    _joint_state_pub = _nh.advertise<sensor_msgs::JointState>(jstate_topic_name, jstate_queue);

    _js_msg.name.resize(_joint_num);
    _js_msg.position.resize(_joint_num);
    _js_msg.velocity.resize(_joint_num);
    _js_msg.effort.resize(_joint_num);
    
    // allocate HAL TBD get from parser the lib to load
    _hal = std::make_shared<ROSEE::DummyHal>(_ee);
    
    // primitives
    init_primitive_subscribers();

}

void ROSEE::UniversalRosEndEffectorExecutor::graspCallback ( const ros_end_effector::EEGraspControlConstPtr& msg ) {

    Eigen::VectorXd pos_ref_upper, pos_ref_lower;
    pos_ref_upper.resize(_ee->getActuatedJointsNum());
    pos_ref_lower.resize(_ee->getActuatedJointsNum());
    
    pos_ref_upper = _ee->getUpperPoisitionLimits() * msg->percentage;
    pos_ref_lower = _ee->getLowerPoisitionLimits() * msg->percentage;
    
    int id = -1;
    for( auto& f : _ee->getFingers() ) {
        
        std::vector<std::string> joints;
        if ( _ee->getActuatedJointsInFinger(f, joints) ) {
            
            for( auto& j : joints ) {
                
                if ( _ee->getInternalIdForJoint(j, id) ) {
                    
                    if ( (pos_ref_upper[id] >= 0) && (pos_ref_lower[id] >= 0) ){
                    
                        _hal->moveMotor(j, pos_ref_upper[id] );
                    }
                    else {
                        
                        _hal->moveMotor(j, pos_ref_lower[id] );
                    }
                }
            }
        }
    }

}


void ROSEE::UniversalRosEndEffectorExecutor::pinchCallback ( const ros_end_effector::EEPinchControlConstPtr& msg ) {

    std::vector<int> ids;

    if ( _ee->isFinger ( msg->finger_pinch_1 ) && _ee->isFinger ( msg->finger_pinch_2 ) ) {

        Eigen::VectorXd pos_ref_upper, pos_ref_lower;
        pos_ref_upper.resize ( _ee->getActuatedJointsNum() );
        pos_ref_lower.resize ( _ee->getActuatedJointsNum() );

        pos_ref_upper = _ee->getUpperPositionLimits() * msg->percentage;
        pos_ref_lower = _ee->getLowerPositionLimits() * msg->percentage;

        // set references for finger_pinch_1
        _ee->getInternalIdsForFinger ( msg->finger_pinch_1, ids );

        for ( const auto& id : ids ) {

            move_joint_in_finger(pos_ref_upper[id], pos_ref_lower[id], id);
        }
        
        // set references for finger_pinch_2
        _ee->getInternalIdsForFinger ( msg->finger_pinch_2, ids );

        for ( const auto& id : ids ) {

            move_joint_in_finger(pos_ref_upper[id], pos_ref_lower[id], id);
        }

    }
    else {
        ROS_ERROR_STREAM ( "finger_pinch_1 :" << msg->finger_pinch_1 << " or finger_pinch_2 :" << msg->finger_pinch_2 << " not defined in current End-Effector" );
    }
}



bool ROSEE::UniversalRosEndEffectorExecutor::init_primitive_subscribers() {

    _sub_grasp = _nh.subscribe<ros_end_effector::EEGraspControl> ( "grasp",
                 1,
                 &ROSEE::UniversalRosEndEffectorExecutor::graspCallback,
                 this
                                                                 );

    if( _ee->getFingersNumber() > 1 ) {
        
        _sub_pinch = _nh.subscribe<ros_end_effector::EEPinchControl> ( "pinch",
                    1,
                    &ROSEE::UniversalRosEndEffectorExecutor::pinchCallback,
                    this
                                                                    );
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


void ROSEE::UniversalRosEndEffectorExecutor::timer_callback ( const ros::TimerEvent& timer_ev ) {

    _hal->sense();
    
    fill_publish_joint_states();

    _hal->move();

    // update time
    _time += _period;
}


void ROSEE::UniversalRosEndEffectorExecutor::spin() {
    
    _loop_timer.start();
    
    ROS_INFO_STREAM( "Started looping @ " << 1./_period << "Hz");
    
    ros::spin();
}



ROSEE::UniversalRosEndEffectorExecutor::~UniversalRosEndEffectorExecutor() {

}



