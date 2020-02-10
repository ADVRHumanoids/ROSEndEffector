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
#include <ROSEndEffector/Action.h>
#include <ROSEndEffector/ActionComposed.h>
#include <ROSEndEffector/ParserMoveIt.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "FindActions" );
    
    std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();
    parserMoveIt->init ("robot_description") ;
    

    ROSEE::FindActions actionsFinder (parserMoveIt);

    auto maps = actionsFinder.findPinch();

    std::map <std::string, ROSEE::ActionTrig> trigMap =  actionsFinder.findTrig (ROSEE::ActionPrimitive::Type::Trig) ;

    std::map <std::string, ROSEE::ActionTrig> tipFlexMap = actionsFinder.findTrig (ROSEE::ActionPrimitive::Type::TipFlex);

    std::map <std::string, ROSEE::ActionTrig> fingFlexMap = actionsFinder.findTrig (ROSEE::ActionPrimitive::Type::FingFlex);


    /** ********************* PARSING TEST and print... these things should not be here ****************/

    //TODO getHandName should be in the parser
    ROSEE::YamlWorker yamlWorker ( parserMoveIt->getHandName() ); 

    //pinch     
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > pinchParsedMap = 
        yamlWorker.parseYamlPrimitive("pinchStrong.yaml", ROSEE::ActionPrimitive::Type::PinchStrong);
            
    //pinch Weak  
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > pinchWeakParsedMap = 
        yamlWorker.parseYamlPrimitive("pinchWeak.yaml", ROSEE::ActionPrimitive::Type::PinchWeak);
        
    //trig
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > trigParsedMap = 
        yamlWorker.parseYamlPrimitive("trig.yaml", ROSEE::ActionPrimitive::Type::Trig);

    //tipFlex
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > tipFlexParsedMap = 
        yamlWorker.parseYamlPrimitive("tipFlex.yaml", ROSEE::ActionPrimitive::Type::TipFlex);
        
    //fingFlex
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > fingFlexParsedMap = 
        yamlWorker.parseYamlPrimitive("fingFlex.yaml", ROSEE::ActionPrimitive::Type::FingFlex);
        
    /******************************* PRINTS OF PARSED PRIMITIVES *********************************************/
    std::cout << "PARSED MAP OF PINCHESSTRONG FROM YAML FILE:" << std::endl;
    for (auto &i : pinchParsedMap) {
        i.second->print();
    }    
    std::cout << "PARSED MAP OF PINCHESWEAK FROM YAML FILE:" << std::endl;
    for (auto &i : pinchWeakParsedMap) {
        i.second->print();
    }
    std::cout << "PARSED MAP OF TRIGS FROM YAML FILE:" << std::endl;
    for (auto &i : trigParsedMap) {
        i.second->print();
    }
    std::cout << "PARSED MAP OF TIPFLEX FROM YAML FILE:" << std::endl;
    for (auto &i : tipFlexParsedMap) {
        i.second->print();
    }
    std::cout << "PARSED MAP OF FINGFLEX FROM YAML FILE:" << std::endl;
    for (auto &i : fingFlexParsedMap) {
        i.second->print();
    }
    
    /** **************************** COMPOSITE ACTION THINGS *************************************************/
    
    std::cout << "A composed action with Independent inner action: " << std::endl;


    ROSEE::ActionComposed grasp ("grasp", true);

    for (auto trig : trigParsedMap) {
        grasp.sumAction  (trig.second) ; 
    }
    
    grasp.print();
    yamlWorker.createYamlFile (&grasp);
    
    //Parsing
    auto actionParsedSimple = yamlWorker.parseYamlComposed ("grasp.yaml");
    std::cout << "The composed action with independent inner action (parsed from generated yaml file):" << std::endl;
    actionParsedSimple.print();
    
    
    if (pinchParsedMap.size() > 0 ) { //if not, we cant add a pinch in the grasp2, so this test isnt done
        
        std::cout << "A composed action with dependent inner action: " << std::endl;
        
        ROSEE::ActionComposed grasp2 ("grasp2", false);

        for (auto trig : trigParsedMap) {
            grasp2.sumAction ( (trig.second) );  
        }

        grasp2.sumAction ( (pinchParsedMap.begin()->second) ) ; 
        if (pinchParsedMap.size() > 1) {
            grasp2.sumAction ( ((++pinchParsedMap.begin())->second) ) ;
        }
        grasp2.print();
        yamlWorker.createYamlFile (&grasp2);
        
        //Parsing
        auto actionParsed = yamlWorker.parseYamlComposed ("grasp2.yaml");
        std::cout << "The composed action with dependent inner action (parsed from generated yaml file):" << std::endl;
        actionParsed.print();
    }
    
    return 0;
    
}

