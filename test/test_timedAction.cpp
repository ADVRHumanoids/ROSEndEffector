#include <gtest/gtest.h>
#include "testUtils.h"

#include <ros/ros.h>

#include <end_effector/FindActions.h>
#include <end_effector/ParserMoveIt.h>
#include <end_effector/GraspingActions/ActionPrimitive.h>
#include <end_effector/GraspingActions/ActionTrig.h>
#include <end_effector/GraspingActions/ActionComposed.h>
#include <end_effector/GraspingActions/ActionPinchGeneric.h>

namespace {

class testTimedAction: public ::testing::Test {


protected:

    testTimedAction() : timedAction("TestTimedAction") {
    }

    virtual ~testTimedAction() {
    }

    virtual void SetUp() override {
        
        std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();

        //if return false, models are not found and it is useless to continue the test
        ASSERT_TRUE(parserMoveIt->init ("robot_description", false)) ;

        ROSEE::FindActions actionsFinder (parserMoveIt);
                
        std::string folderForActions = "ROSEE/actions/" + parserMoveIt->getHandName();
        
        trigMap = actionsFinder.findTrig(ROSEE::ActionPrimitive::Type::Trig, folderForActions + "/primitives/") ;  
        auto pinches = actionsFinder.findPinch(folderForActions + "/primitives/") ; 
        
        pinchTightMap = pinches.first;
        pinchLooseMap = pinches.second;
                
        std::shared_ptr<ROSEE::ActionComposed> actionComposed = std::make_shared<ROSEE::ActionComposed> ("degrasp");

        if (trigMap.size() > 0){
            for (auto trig : trigMap) {
                std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
                    std::make_shared <ROSEE::ActionTrig> ( trig.second );
                    
                actionComposed->sumAction(pointer, 0);
                
                //different name for each trig inserted
                timedAction.insertAction(pointer, 0.54, 0.2, 0, 0.8, "trig_" + *(pointer->getKeyElements().begin()) );  
            }
        }
        
        //lets add a pinch
        if (pinchTightMap.size()>0) {
            
            std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
                    std::make_shared <ROSEE::ActionPinchTight> ( pinchTightMap.begin()->second );
                    
            timedAction.insertAction(pointer, 0, 0.45);
            
        } else if (pinchLooseMap.size()>0) {
            
            std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
                    std::make_shared <ROSEE::ActionPinchLoose> ( pinchLooseMap.begin()->second );
                    
            timedAction.insertAction(pointer, 0, 0.45);
        }
        
        //now lets add a reset grasp pos (trig all zero)
        if (actionComposed->numberOfInnerActions( ) > 0) {
            std::shared_ptr <ROSEE::Action> pointer =  actionComposed ;
            timedAction.insertAction(pointer);
        }
        
        ROSEE::YamlWorker yamlWorker;
        yamlWorker.createYamlFile (&timedAction, folderForActions + "/timeds/");
            
        //Parsing
        timedActionParsed = yamlWorker.parseYamlTimed (folderForActions + "/timeds/TestTimedAction.yaml");
        
    }

    virtual void TearDown() override{
    }
    
    ROSEE::ActionTrig::Map trigMap;
    ROSEE::ActionPinchTight::Map pinchTightMap;
    ROSEE::ActionPinchLoose::Map pinchLooseMap;
    
    ROSEE::ActionTimed timedAction;
    std::shared_ptr<ROSEE::ActionTimed> timedActionParsed;

   
};

TEST_F ( testTimedAction, checkMembersSizeConsistency ) {
    
    EXPECT_EQ ( timedAction.getInnerActionsNames().size(), timedAction.getAllActionMargins().size() );
    EXPECT_EQ ( timedAction.getAllActionMargins().size(), timedAction.getAllJointCountAction().size() );
    EXPECT_EQ ( timedAction.getAllJointCountAction().size(), timedAction.getAllJointPos().size() );
    
    EXPECT_EQ ( timedActionParsed->getInnerActionsNames().size(), timedActionParsed->getAllActionMargins().size() );
    EXPECT_EQ ( timedActionParsed->getAllActionMargins().size(), timedActionParsed->getAllJointCountAction().size() );
    EXPECT_EQ ( timedActionParsed->getAllJointCountAction().size(), timedActionParsed->getAllJointPos().size() );   
    
}

TEST_F ( testTimedAction, checkEmitParse ) {
    
    if (timedAction.getInnerActionsNames().size() > 0) {
        
        EXPECT_EQ (timedAction.getName(), timedActionParsed->getName() );
        EXPECT_EQ (timedAction.getType(), timedActionParsed->getType() );
        EXPECT_EQ (timedAction.getFingersInvolved(), timedActionParsed->getFingersInvolved() );
        
        for (auto jointCount : timedAction.getJointsInvolvedCount()) {
            EXPECT_EQ ( jointCount.second, timedActionParsed->getJointsInvolvedCount().at(jointCount.first) ); 
        }
        
        for (auto joint: timedAction.getJointPos() ) {
            
            //compare size of joint (number of dofs)
            ASSERT_EQ (joint.second.size(), timedActionParsed->getJointPos().at(joint.first).size() );
            //loop the eventually multiple joint pos (when dofs > 1)
            for (int j = 0; j < joint.second.size(); ++j ){
                EXPECT_DOUBLE_EQ ( joint.second.at(j), timedActionParsed->getJointPos().at(joint.first).at(j) ); 
            }     
        }
        
        unsigned int k = 0;
        for (auto innerActName : timedAction.getInnerActionsNames()) {
            EXPECT_EQ ( innerActName, timedActionParsed->getInnerActionsNames().at(k) );
            k++;
        }
        
        k = 0;
        for (auto timeMargins : timedAction.getAllActionMargins() ) {
            
            EXPECT_DOUBLE_EQ (timeMargins.first, timedActionParsed->getAllActionMargins().at(k).first);
            EXPECT_DOUBLE_EQ (timeMargins.second, timedActionParsed->getAllActionMargins().at(k).second);
            k++;
        }

        auto jpvector = timedAction.getAllJointPos();
        for (int i=0; i < jpvector.size(); i++) {
            
            for (auto joint: jpvector.at(i) ) {
                
                auto otherjointPos = timedActionParsed->getAllJointPos().at(i).at(joint.first);
                //compare size of joint (number of dofs)
                ASSERT_EQ (joint.second.size(), otherjointPos.size() );
                
                //loop the eventually multiple joint pos (when dofs > 1)
                for (int j = 0; j < joint.second.size(); ++j ){
                    EXPECT_DOUBLE_EQ ( joint.second.at(j), otherjointPos.at(j) ); 
                } 
            }
        }
        
        auto jpcvector = timedAction.getAllJointCountAction();
        for (int i=0; i < jpcvector.size(); i++) {
            
            for (auto joint: jpcvector.at(i) ) {
                
                EXPECT_EQ ( joint.second, timedActionParsed->getAllJointCountAction().at(i).at(joint.first) ); 
                
            }
        }

    }
}

} //namespace

int main ( int argc, char **argv ) {
    
    if (argc < 2 ){
        
        std::cout << "[TEST ERROR] Insert hand name as argument" << std::endl;
        return -1;
    }
    
    /* Run tests on an isolated roscore */
    if(setenv("ROS_MASTER_URI", "http://localhost:11322", 1) == -1)
    {
        perror("setenv");
        return 1;
    }
    
    //run roscore
    std::unique_ptr<ROSEE::TestUtils::Process> roscore;
    roscore.reset(new ROSEE::TestUtils::Process({"roscore", "-p", "11322"}));
    
    if ( ROSEE::TestUtils::prepareROSForTests ( argc, argv, "testTimedAction" ) != 0 ) {
        
        std::cout << "[TEST ERROR] Prepare Function failed" << std::endl;
        return -1;
    }
    
    
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
