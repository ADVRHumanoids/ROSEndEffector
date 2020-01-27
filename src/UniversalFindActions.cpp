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
    
    std::map <std::string, ROSEE::ActionTrig> trigMap =  actionsFinder.findTrig (ROSEE::ActionType::Trig) ;
    std::map <std::string, ROSEE::ActionTrig> tipFlexMap = actionsFinder.findTrig (ROSEE::ActionType::TipFlex);
    std::map <std::string, ROSEE::ActionTrig> fingFlexMap = actionsFinder.findTrig (ROSEE::ActionType::FingFlex);
   
    

    /// PARSING TEST and print... these things should not be here

    //TODO getHandName should be in the parser
    ROSEE::YamlWorker yamlWorker(actionsFinder.getHandName());
    
    //pinch     
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > pinchParsedMap = 
        yamlWorker.parseYaml("pinchStrong.yaml", ROSEE::ActionType::PinchStrong);
    
    std::cout << "PARSED MAP OF PINCHESSTRONG FROM YAML FILE:" << std::endl;
    for (auto &i : pinchParsedMap) {
        i.second->printAction();
    }
    
    
    //pinch Weak  
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > pinchWeakParsedMap = 
        yamlWorker.parseYaml("pinchWeak.yaml", ROSEE::ActionType::PinchWeak);
    
    std::cout << "PARSED MAP OF PINCHESWEAK FROM YAML FILE:" << std::endl;
    for (auto &i : pinchWeakParsedMap) {
        i.second->printAction();
    }
    
    /*
    //trig
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > trigParsedMap = 
        yamlWorker.parseYaml("trig.yaml", ROSEE::ActionType::Trig);
    
    std::cout << "PARSED MAP OF TRIGS FROM YAML FILE:" << std::endl;
    for (auto &i : trigParsedMap) {
        i.second->printAction();
    }
    
    //tipFlex
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > tipFlexParsedMap = 
        yamlWorker.parseYaml("tipFlex.yaml", ROSEE::ActionType::TipFlex);
    
    std::cout << "PARSED MAP OF TIPFLEX FROM YAML FILE:" << std::endl;
    for (auto &i : tipFlexParsedMap) {
        i.second->printAction();
    }
    
    //fingFlex
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > fingFlexParsedMap = 
        yamlWorker.parseYaml("fingFlex.yaml", ROSEE::ActionType::FingFlex);
    
    std::cout << "PARSED MAP OF FINGFLEX FROM YAML FILE:" << std::endl;
    for (auto &i : fingFlexParsedMap) {
        i.second->printAction();
    }
    
    */
    
    return 0;
    
}

