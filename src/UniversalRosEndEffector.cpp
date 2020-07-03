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

#include <ros_end_effector/UniversalRosEndEffectorExecutor.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "UniversalRosEndEffector" );
    
    ROSEE::UniversalRosEndEffectorExecutor executor("ros_end_effector");
    executor.spin();
    
    return 0;
    
}
