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
#include <ROSEndEffector/ActionComposed.h>
#include <ROSEndEffector/ParserMoveIt.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "FindActions" );
    
    std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();
    parserMoveIt->init ("robot_description") ;
    
    ROSEE::FindActions actionsFinder (parserMoveIt);

    auto maps = actionsFinder.findPinch();

    std::map <std::string, ROSEE::ActionTrig> trigMap =  actionsFinder.findTrig (ROSEE::ActionType::Trig) ;
    std::map <std::string, ROSEE::ActionTrig> tipFlexMap = actionsFinder.findTrig (ROSEE::ActionType::TipFlex);
    std::map <std::string, ROSEE::ActionTrig> fingFlexMap = actionsFinder.findTrig (ROSEE::ActionType::FingFlex);
    

    /** ********************* PARSING TEST and print... these things should not be here ****************/

    //TODO getHandName should be in the parser
    ROSEE::YamlWorker yamlWorker ( parserMoveIt->getHandName() ); 

    //pinch     
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > pinchParsedMap = 
        yamlWorker.parseYaml("pinchStrong.yaml", ROSEE::ActionType::PinchStrong);
            
    //pinch Weak  
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > pinchWeakParsedMap = 
        yamlWorker.parseYaml("pinchWeak.yaml", ROSEE::ActionType::PinchWeak);
        
    //trig
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > trigParsedMap = 
        yamlWorker.parseYaml("trig.yaml", ROSEE::ActionType::Trig);
        
    //tipFlex
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > tipFlexParsedMap = 
        yamlWorker.parseYaml("tipFlex.yaml", ROSEE::ActionType::TipFlex);
        
    //fingFlex
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > fingFlexParsedMap = 
        yamlWorker.parseYaml("fingFlex.yaml", ROSEE::ActionType::FingFlex);
        
    /******************************* PRINTS OF PARSED PRIMITIVES *********************************************/
    std::cout << "PARSED MAP OF PINCHESSTRONG FROM YAML FILE:" << std::endl;
    for (auto &i : pinchParsedMap) {
        i.second->printAction();
    }    
    std::cout << "PARSED MAP OF PINCHESWEAK FROM YAML FILE:" << std::endl;
    for (auto &i : pinchWeakParsedMap) {
        i.second->printAction();
    }
    std::cout << "PARSED MAP OF TRIGS FROM YAML FILE:" << std::endl;
    for (auto &i : trigParsedMap) {
        i.second->printAction();
    }
    std::cout << "PARSED MAP OF TIPFLEX FROM YAML FILE:" << std::endl;
    for (auto &i : tipFlexParsedMap) {
        i.second->printAction();
    }
    std::cout << "PARSED MAP OF FINGFLEX FROM YAML FILE:" << std::endl;
    for (auto &i : fingFlexParsedMap) {
        i.second->printAction();
    }
    
    /** **************************** COMPOSITE ACTION THINGS *************************************************/
    
    std::cout << "A composed action with Independent primitives: " << std::endl;


    ROSEE::ActionComposed grasp ("grasp", true);
    for (auto trig : trigParsedMap) {
        grasp.sumPrimitive ( (trig.second) ); 
    }
    
    grasp.printAction();
    yamlWorker.createYamlFile (&grasp);
    
    //Parsing
    //auto actionParsed = yamlWorker.parseYamlComposed ("grasp.yaml");
    //std::cout << "parsed Composed" << std::endl;
    //actionParsed.printAction();
    
    
    std::cout << "A composed action with dependent primitives: " << std::endl;
    
    ROSEE::ActionComposed grasp2 ("grasp2", false);

    for (auto trig : trigParsedMap) {
        grasp2.sumPrimitive ( (trig.second) );  
    }
    
    grasp2.sumPrimitive ( (pinchParsedMap.begin()->second) ) ;
    grasp2.sumPrimitive ( ((++pinchParsedMap.begin())->second) ) ;

    grasp2.printAction();
    yamlWorker.createYamlFile (&grasp);
    
    //Parsing
    auto actionParsed = yamlWorker.parseYamlComposed ("grasp.yaml");
    std::cout << "The composed action with dependent primitives (parsed from generated yaml file):" << std::endl;
    actionParsed.printAction();
    
    return 0;
    
}

