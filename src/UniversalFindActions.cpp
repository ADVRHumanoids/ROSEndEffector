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
//#include <end_effector/UniversalRosEndEffectorExecutor.h>
#include <end_effector/FindActions.h>
#include <end_effector/GraspingActions/Action.h>
#include <end_effector/GraspingActions/ActionComposed.h>
#include <end_effector/GraspingActions/ActionTimed.h>
#include <end_effector/GraspingActions/ActionGeneric.h>
#include <end_effector/ParserMoveIt.h>
#include <end_effector/Parser.h> //to take urdf from conf file

#include <end_effector/MapActionHandler.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "FindActions" );
    
    ros::NodeHandle nh;
    ROSEE::Parser parser(nh);
    parser.init();
    //Load the ROS Server with urdf and srdf 
    nh.setParam("/robot_description", parser.getUrdfString());
    nh.setParam("/robot_description_semantic", parser.getSrdfString());
    ROS_INFO_STREAM("FINDACTIONS: Set urdf and srdf file in the param server");
    
    std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();
    if (! parserMoveIt->init ("robot_description") ) {
        ROS_ERROR_STREAM ("FAILED parserMoveit Init, stopping execution");
        return -1;
    }
    //xml is necessary... but parsermoveit has no access to it, so we must pass it here
    parserMoveIt->parseNonLinearMimicRelations(parser.getUrdfString());

    
    std::string folderForActions = parser.getActionPath();
    
    ROSEE::FindActions actionsFinder (parserMoveIt);

    auto maps = actionsFinder.findPinch(folderForActions + "/primitives/");

    std::map <std::string, ROSEE::ActionTrig> trigMap =  actionsFinder.findTrig (ROSEE::ActionPrimitive::Type::Trig, 
                                                                                 folderForActions + "/primitives/") ;

    std::map <std::string, ROSEE::ActionTrig> tipFlexMap = actionsFinder.findTrig (ROSEE::ActionPrimitive::Type::TipFlex, 
                                                                                   folderForActions + "/primitives/");

    std::map <std::string, ROSEE::ActionTrig> fingFlexMap = actionsFinder.findTrig (ROSEE::ActionPrimitive::Type::FingFlex, 
                                                                                    folderForActions + "/primitives/");
    unsigned int nFinger = 3;
    std::map < std::string, ROSEE::ActionSingleJointMultipleTips> singleJointMultipleTipsMap = 
        actionsFinder.findSingleJointMultipleTips (nFinger, folderForActions + "/primitives/") ;
    
    nFinger = 2;
    std::map < std::string, ROSEE::ActionSingleJointMultipleTips> singleJointMultipleTipsMap2 = 
        actionsFinder.findSingleJointMultipleTips (nFinger, folderForActions + "/primitives/") ;
        
    nFinger = 5;
    std::map < std::string, ROSEE::ActionSingleJointMultipleTips> singleJointMultipleTipsMap5 = 
        actionsFinder.findSingleJointMultipleTips (nFinger, folderForActions + "/primitives/") ;
    
    auto mulPinch = actionsFinder.findMultiplePinch(3, folderForActions + "/primitives/" );


    /** ********************* PARSING TEST and print... these things should not be here ****************/
    
    ROSEE::MapActionHandler mapsHandler;
    mapsHandler.parseAllPrimitives(folderForActions + "/primitives/");

        
    /******************************* PRINTS OF PARSED PRIMITIVES *********************************************/
    std::cout << "PARSED MAP OF PINCHESTIGHT FROM YAML FILE:" << std::endl;
    for (auto &i : mapsHandler.getPrimitiveMap("pinchTight")) {
        i.second->print();
    }    
    std::cout << "PARSED MAP OF PINCHESLOOSE FROM YAML FILE:" << std::endl;
    for (auto &i : mapsHandler.getPrimitiveMap("pinchLoose")) {
        i.second->print();
    }
    std::cout << "PARSED MAP OF TRIGS FROM YAML FILE:" << std::endl;
    for (auto &i : mapsHandler.getPrimitiveMap("trig")) {
        i.second->print();
    }
    std::cout << "PARSED MAP OF TIPFLEX FROM YAML FILE:" << std::endl;
    for (auto &i : mapsHandler.getPrimitiveMap("tipFlex")) {
        i.second->print();
    }
    std::cout << "PARSED MAP OF FINGFLEX FROM YAML FILE:" << std::endl;
    for (auto &i : mapsHandler.getPrimitiveMap("fingFlex")) {
        i.second->print();
    }
    std::cout << "PARSED MAP OF SINGLEJOINTMULTIPLETIPS_3 FROM YAML FILE:" << std::endl;
    for (auto &i : mapsHandler.getPrimitiveMap("singleJointMultipleTips_3")) {
        i.second->print();
    }
    std::cout << "DEBUG MULTIPINCH PARSED: " << std::endl;
    for (auto &it : mapsHandler.getPrimitiveMap("multiplePinchTight_3")) {
        it.second->print();
    }
    
    
    ROSEE::YamlWorker yamlWorker;
    /** **************************** COMPOSITE ACTION THINGS ************************************************ */
/**
    if (mapsHandler.getPrimitiveMap(ROSEE::ActionPrimitive::Type::Trig).size() > 0  &&  
        mapsHandler.getPrimitiveMap(ROSEE::ActionPrimitive::Type::Trig).at(0).size() == parserMoveIt->getNFingers() ) {
        
        std::cout << "A composed action with Independent inner action: " << std::endl;
        ROSEE::ActionComposed grasp ("grasp", true);
        
        for (auto trig : mapsHandler.getPrimitiveMap("trig")) {
            grasp.sumAction  (trig.second) ; 
        }
        grasp.print();
       
        yamlWorker.createYamlFile (&grasp, folderForActions + "/generics/");
        
        mapsHandler.parseAllGenerics (folderForActions + "/generics/");
        
        std::cout << "PARSED COMPOSEd" << std::endl;
        mapsHandler.getGeneric("grasp")->print();

    } else  { //look if we have a single singleJointMultipleTips_MAXFINGER: it is 99% a grasp
        
        std::cout << "A singleJointMultipleTips that move all fingers: " << std::endl;

        std::map < std::string, ROSEE::ActionSingleJointMultipleTips> singleJointMultipleTipsMap = actionsFinder.findSingleJointMultipleTips (parserMoveIt->getNFingers(), folderForActions + "/primitives/") ;
        
        if (singleJointMultipleTipsMap.size() == 1) { //if more, we do not know which is the one for grasping
            std::cout << "No Composed Grasp with trig but I found a SingleJointMultipleTips_" <<
            parserMoveIt->getNFingers() <<" that probably is a grasp (ie a joint that move all fingers)" << std::endl;
        }
        std::cout << "PARSED MAP OF singleJointMultipleTips_" << parserMoveIt->getNFingers() << "  FROM YAML FILE:" << std::endl;
        for (auto &i : mapsHandler.getPrimitiveMap("singleJointMultipleTips_" + std::to_string(parserMoveIt->getNFingers()) )) {
            i.second->print();
        }
    }
 **/

    
    /** **************************** SIMPLE ACTION MANUALLY CREATED ***********************************************    */  

    /** example only doable if maps is not empty */
/**
    if (maps.first.size() != 0 ) {
        ROSEE::JointPos jp;

        //for now copy jp of another action
        jp = ROSEE::operator*(maps.first.begin()->second.getJointPos(), 2);
        auto jpc = maps.first.begin()->second.getJointsInvolvedCount();

        ROSEE::ActionGeneric simpleAction("casual", jp, jpc);
        std::cout << std::endl << "Casual action manually created: " << std::endl;

        simpleAction.print();
        
        yamlWorker.createYamlFile( &simpleAction,  folderForActions + "/generics/" );

        mapsHandler.parseAllGenerics (folderForActions + "/generics/"); //NOTE already called before

        std::cout << "The parsed casual: " << std::endl;
        mapsHandler.getGeneric("casual")->print();
    }
**/

    
    /** **************************** TIMED ACTION THINGS*****************************
     *************************EXAMPLE ONLY VALID FOR SPECIFIC HANDS *********************************************/

    if (parser.getEndEffectorName().compare("schunk") == 0) {
        

        //Schuk actions created for paper

        // lets create the ActionComposed object
        ROSEE::ActionComposed schunkGrasp ("grasp");

        //We fill the action with trigs done with all the long fingers
        schunkGrasp.sumAction (mapsHandler.getPrimitive("trig", "index"));
        schunkGrasp.sumAction (mapsHandler.getPrimitive("trig", "middle"));
        schunkGrasp.sumAction (mapsHandler.getPrimitive("trig", "ring"));
        schunkGrasp.sumAction (mapsHandler.getPrimitive("trig", "pinky"));

        // for the thumb, we do not want a complete trig, so we pass a scale factor < 1
        schunkGrasp.sumAction (mapsHandler.getPrimitive("trig", "thumb"), 0.3);

        //remember to emit the yaml file, so we can use the action each time we want it later
        yamlWorker.createYamlFile (&schunkGrasp, folderForActions + "/generics/");

        ROSEE::ActionTimed actionTimed ("timed_wide_grasp");

        actionTimed.insertAction( mapsHandler.getPrimitive("singleJointMultipleTips_3", "left_hand_Finger_Spread"),
                                0, 0.7, 0, 0.4, "FingerSpread");

        actionTimed.insertAction( mapsHandler.getPrimitive("singleJointMultipleTips_3", "left_hand_Thumb_Opposition"),
                                0, 0.7, 0, 0.47, "Opposition");

        ROSEE::Action::Ptr schunkTipFlexs = std::make_shared<ROSEE::ActionComposed>("TipFlexes", true);

        ROSEE::ActionComposed::Ptr schunkTipFlexsCasted = std::dynamic_pointer_cast<ROSEE::ActionComposed>(schunkTipFlexs);

        schunkTipFlexsCasted->sumAction (mapsHandler.getPrimitive("tipFlex", "index"), 0.84);
        schunkTipFlexsCasted->sumAction (mapsHandler.getPrimitive("tipFlex", "middle"), 0.76);
        schunkTipFlexsCasted->sumAction (mapsHandler.getPrimitive("trig", "thumb"), 0.8);

        actionTimed.insertAction( schunkTipFlexs, 0.6, 0.2, 0, 1, "TipFlexes");

        actionTimed.print();

        yamlWorker.createYamlFile ( &actionTimed, folderForActions + "/timeds/" );

       // mapsHandler.parseAllTimeds(folderForActions + "/timeds/");
        //std::cout << "The timed action parsed: " << std::endl;
        //mapsHandler.getTimed("timed_wide_grasp")->print();


    } else if (parser.getEndEffectorName().compare("robotiq_3f") == 0) {

        ROSEE::ActionTimed actionTimed ("timed_wide_grasp");

        actionTimed.insertAction( mapsHandler.getPrimitive("singleJointMultipleTips_2", "palm_finger_1_joint"),
                0, 1, 0, 1, "OpenWide");

        actionTimed.insertAction( mapsHandler.getPrimitive("singleJointMultipleTips_3", "finger_1_joint_1"),
                0, 0, 0, 0.6, "Grasp");

        yamlWorker.createYamlFile ( &actionTimed, folderForActions + "/timeds/" );

        
     } else  if (parserMoveIt->getHandName().compare("heri_II") == 0 ) {
        
        //first, we create the "grasp without trigger finger (the third one)
        
        //TODO correct way to do this?
        ROSEE::Action::Ptr grasp3f = std::make_shared<ROSEE::ActionComposed>("grasp3f", true);
        std::shared_ptr<ROSEE::ActionComposed> grasp3f_casted = std::dynamic_pointer_cast<ROSEE::ActionComposed> (grasp3f);
        
        grasp3f_casted->sumAction(mapsHandler.getPrimitive("trig", "finger_1"));
        grasp3f_casted->sumAction(mapsHandler.getPrimitive("trig", "finger_2"));
        grasp3f_casted->sumAction(mapsHandler.getPrimitive("trig", "thumb"));
   
       // yamlWorker.createYamlFile (&grasp3f_casted, folderForActions + "/generics/");
        
        ROSEE::ActionTimed actionTimed ("drill");

        actionTimed.insertAction( grasp3f, 0, 0, 0, 1, "Grasp3f");
        actionTimed.insertAction( mapsHandler.getPrimitive("trig", "finger_3"), 3, 0, 0, 1, "TrigOn");
        actionTimed.insertAction( mapsHandler.getPrimitive("trig", "finger_3"), 4, 0, 0, 0, "TrigOff");
        
        yamlWorker.createYamlFile ( &actionTimed, folderForActions + "/timeds/" );
        mapsHandler.parseAllTimeds(folderForActions + "/timeds/");

        std::cout << "The timed action parsed: " << std::endl;
        mapsHandler.getTimed("drill")->print();
    }

     return 0;
}

