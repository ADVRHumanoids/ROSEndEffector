/*
 * Copyright (C) 2020 IIT-HHCM
 * Author: Davide Torielli
 * email:  davide.torielli@iit.it
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

#include <ROSEndEffector/UniversalRosEndEffectorExecutor.h>
#include <ROSEndEffector/FindActions.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "FindActions" );
    
    ROSEE::FindActions actionsFinder ("robot_description");

    actionsFinder.findPinch();
    actionsFinder.findTrig();
    
    //TODO getHandName should be in the parser
    ROSEE::YamlWorker yamlWorker(actionsFinder.getHandName());

    //pinch     
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > pinchParsedMap = 
        yamlWorker.parseYaml("pinch.yaml", ROSEE::ActionType::Pinch);
    
    std::cout << "PARSED MAP OF PINCHES FROM YAML FILE:" << std::endl;
    for (auto &i : pinchParsedMap) {
        i.second->printAction();
    }
    
    //trig
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > trigParsedMap = 
        yamlWorker.parseYaml("trig.yaml", ROSEE::ActionType::Trig);
    
    std::cout << "PARSED MAP OF TRIGS FROM YAML FILE:" << std::endl;
    for (auto &i : trigParsedMap) {
        i.second->printAction();
    }
    
    return 0;
    
}

