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


#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/JointState.h>

#include <ROSEndEffector/Parser.h>
#include <ROSEndEffector/EEInterface.h>
#include <ROSEndEffector/EEHal.h>
#include <ROSEndEffector/DummyHal.h>


int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "UniversalRosEndEffector" );
    ros::NodeHandle nh;

    ros::Rate loop_rate ( 100 );
    int count = 0;
    
    ROSEE::Parser p(nh);
    p.init("/home/lucamuratore/src/ros_end_effector__ws/src/ROSEndEffector/configs/test_ee.yaml");
    p.printEndEffectorFingerJointsMap();


    ROSEE::EEInterface::Ptr ee = p.getEndEffectorInterface();
    
    std::vector<std::string> fingers;
    fingers = ee->getFingers();

    ROS_INFO_STREAM("Fingers in EEInterface: ");
    for( auto& f : fingers ) {
         ROS_INFO_STREAM(f);
    }
    
    int joint_num = ee->getActuatedJointsNum();
    
    // TEST
    if ( ee->isFinger("finger_4") ) {
        ROS_INFO_STREAM("finger_4 exists!");
    }
    else {
        ROS_WARN_STREAM("finger_4 does not exist!");
    }

    
    // prepare joint state publisher
    std::string jstate_topic_name  = "joint_states";
    const int jstate_queue = 10;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>(jstate_topic_name, jstate_queue);
    sensor_msgs::JointState js_msg;

    js_msg.name.resize(joint_num);
    js_msg.position.resize(joint_num);
    js_msg.velocity.resize(joint_num);
    js_msg.effort.resize(joint_num);
    
    int seq = 0;
    
    // allocate HAL TBD get from parser the lib to load
    ROSEE::EEHal::Ptr hal = std::make_shared<ROSEE::DummyHal>(ee);
    
    std::vector<std::string> joints;
    
//     double test_ref = 0.1;
    while ( ros::ok() ) {
        
        
        hal->sense();
        
        js_msg.header.stamp = ros::Time::now();
        js_msg.header.seq = seq++;
        
        int c = 0;
        for( auto& f : fingers ) {
            
            ee->getActuatedJointsInFinger(f, joints);
            
            double value = 0;
            for( auto& j : joints ) {
                        
                js_msg.name[c] = j;  
                
                hal->getMotorPosition(j, value);
                js_msg.position[c] = value;
                
                hal->getMotorVelocity(j, value);
                js_msg.velocity[c] = value;
                
                hal->getMotorEffort(j, value);
                js_msg.effort[c] = value;

                c++;
            }
            joints.clear();
        } 
        
        joint_state_pub.publish(js_msg);
        
//         test_ref += 0.001;
//         hal->moveMotor("base_to_right_finger2", test_ref);
        
        hal->move();

        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }


    return 0;
}
