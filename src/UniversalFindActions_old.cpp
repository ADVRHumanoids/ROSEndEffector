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
#include <ROSEndEffector/ActionTimed.h>
#include <ROSEndEffector/ActionGeneric.h>
#include <ROSEndEffector/ParserMoveIt.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "FindActions" );
    
    std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();
    parserMoveIt->init ("robot_description") ;
    
    std::string folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/" + parserMoveIt->getHandName();
    
    ROSEE::FindActions actionsFinder (parserMoveIt);

    auto maps = actionsFinder.findPinch(folderForActions + "/primitives/");

    std::map <std::string, ROSEE::ActionTrig> trigMap =  actionsFinder.findTrig (ROSEE::ActionPrimitive::Type::Trig, 
                                                                                 folderForActions + "/primitives/") ;

    std::map <std::string, ROSEE::ActionTrig> tipFlexMap = actionsFinder.findTrig (ROSEE::ActionPrimitive::Type::TipFlex, 
                                                                                   folderForActions + "/primitives/");

    std::map <std::string, ROSEE::ActionTrig> fingFlexMap = actionsFinder.findTrig (ROSEE::ActionPrimitive::Type::FingFlex, 
                                                                                    folderForActions + "/primitives/");


    /** ********************* PARSING TEST and print... these things should not be here ****************/

    //TODO getHandName should be in the parser
    ROSEE::YamlWorker yamlWorker ; 

    //pinch     
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > pinchParsedMap = 
        yamlWorker.parseYamlPrimitive("pinchStrong.yaml", ROSEE::ActionPrimitive::Type::PinchStrong, folderForActions+"/primitives/");
                
    //pinch Weak  
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > pinchWeakParsedMap = 
        yamlWorker.parseYamlPrimitive("pinchWeak.yaml", ROSEE::ActionPrimitive::Type::PinchWeak, folderForActions+"/primitives/");
        
    //trig
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > trigParsedMap = 
        yamlWorker.parseYamlPrimitive("trig.yaml", ROSEE::ActionPrimitive::Type::Trig, folderForActions+"/primitives/");

    //tipFlex
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > tipFlexParsedMap = 
        yamlWorker.parseYamlPrimitive("tipFlex.yaml", ROSEE::ActionPrimitive::Type::TipFlex, folderForActions+"/primitives/");
        
    //fingFlex
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > fingFlexParsedMap = 
        yamlWorker.parseYamlPrimitive("fingFlex.yaml", ROSEE::ActionPrimitive::Type::FingFlex, folderForActions+"/primitives/");
        
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
    yamlWorker.createYamlFile (&grasp, folderForActions + "/generics/");
    
    //Parsing
    ROSEE::Action::Ptr actionParsedSimple = std::make_shared<ROSEE::ActionComposed> () ;
    yamlWorker.parseYamlAction ("grasp.yaml", actionParsedSimple, folderForActions+"/generics/");
    std::cout << "The composed action with independent inner action (parsed from generated yaml file):" << std::endl;
    actionParsedSimple->print();
    
    
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
        yamlWorker.createYamlFile (&grasp2, folderForActions + "/generics/");
        
        //Parsing
        ROSEE::Action::Ptr actionParsed = std::make_shared<ROSEE::ActionComposed> () ;
        yamlWorker.parseYamlAction ("/generics/grasp2.yaml", actionParsed, folderForActions + "/generics/");
        std::cout << "The composed action with dependent inner action (parsed from generated yaml file):" << std::endl;
        actionParsed->print();
    }

    
    /** **************************** ACTION MORE TIPS TO MOVE MORE TIPS WITH SINGLE JOINT ****************************/
    unsigned int nFinger = 5;
    std::map < std::string, ROSEE::ActionSingleJointMultipleTips> singleJointMultipleTipsMap = actionsFinder.findSingleJointMultipleTips (nFinger, folderForActions + "/primitives/") ;
    
    std::cout << "A primitive that move " << nFinger << " fingers with a single joint " << std::endl;
    if (singleJointMultipleTipsMap.size() == 0) {
        std::cout << "Nothing :C " << std::endl;
        
    } else {
        for (auto map : singleJointMultipleTipsMap) {
            map.second.print();
        }
    }
    
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > singleJointMultipleTipsParsedMap = 
        yamlWorker.parseYamlPrimitive("singleJointMultipleTips-" + std::to_string(nFinger) + ".yaml", ROSEE::ActionPrimitive::Type::SingleJointMultipleTips, 
                                      folderForActions+"/primitives/");
    std::cout << "PARSED MAP OF SINGLEJOINTMULTIPLETIPS FROM YAML FILE:" << std::endl;
    
    for (auto &i : singleJointMultipleTipsParsedMap) {
        i.second->print();
    }

    /** **************************** TIMED ACTION THINGS ***********************************************
    ROSEE::ActionTimed actionTimed("wide_grasp");
    std::set<std::string> one;
    one.insert ("finger_1_joint_1");
    actionTimed.insertAction( singleJointMultipleTipsParsedMap.at(one), 0, 0.2, 0, 0.5, "GRASP");
    one.clear();
    one.insert("finger_1_joint_1");
    actionTimed.insertAction( singleJointMultipleTipsParsedMap.at(one), 0, 0.2, 0, 1, "GRASP2");
    
    one.clear();
    one.insert("finger_2_link_3");
    one.insert("finger_middle_link_3");    
    actionTimed.insertAction( pinchParsedMap.at(one), 0, 0.2, 0, 1, "PINCH");
    actionTimed.insertAction( pinchParsedMap.at(one), 0, 0.2, 0, 0.5, "PINCH2");

    actionTimed.print();
    
    yamlWorker.createYamlFile ( &actionTimed );
     //Parsing
    ROSEE::Action::Ptr actionTimedParsed = std::make_shared <ROSEE::ActionTimed> ();
    yamlWorker.parseYamlAction ("wide_grasp.yaml", actionTimedParsed);
    std::cout << "The timed action parsed: " << std::endl;
    actionTimedParsed->print();
    */
    /** **************************** SIMPLE ACTION MANUALLY CREATED ***********************************************    

    ROSEE::JointPos jp;

    //for now copy jp of another action
    jp = ROSEE::operator*(maps.first.begin()->second.getJointPos(), 2);
    auto jpc = maps.first.begin()->second.getJointsInvolvedCount();

    ROSEE::ActionGeneric simpleAction("casual", jp, jpc);
    simpleAction.print();
    ROSEE::ActionGeneric simple2("casual2", ROSEE::operator+(jp, jp), jpc);
    simple2.print();
    
    yamlWorker.createYamlFile( &simpleAction );

    ROSEE::Action::Ptr newCasual = std::make_shared<ROSEE::ActionGeneric>();
    yamlWorker.parseYamlAction ("casual.yaml", newCasual);
    std::cout << "The parsed casual: " << std::endl;
    newCasual->print();
    */
    
    
    /******* NEW PRIMITIVE MULTIPINCH ***********************************/
    std::cout << std::endl << std::endl << "NEW MULTIPINCH" << std::endl;
    auto mulPinch = actionsFinder.findMultiplePinch(3, folderForActions + "/primitives/" );
    for (auto &it : mulPinch) {
        for (auto itt : it.first) {
            std::cout << itt << std::endl;
        }
        it.second.print();
    }
    
    //parsing  
    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > multPinchParsedMap = 
        yamlWorker.parseYamlPrimitive("multiplePinchStrong-3.yaml",
                                    ROSEE::ActionPrimitive::Type::MultiplePinchStrong, folderForActions+"/primitives/");
        
    std::cout << "DEBUG MULTIPINCH PARSED: " << std::endl;
    for (auto &it : multPinchParsedMap) {
        it.second->print();
    }


    return 0;
    
}
