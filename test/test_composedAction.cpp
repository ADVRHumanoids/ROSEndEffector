#include <gtest/gtest.h>
#include "testUtils.h"

#include <ros/ros.h>

#include <end_effector/FindActions.h>
#include <end_effector/ParserMoveIt.h>
#include <end_effector/GraspingActions/ActionComposed.h>
#include <end_effector/GraspingActions/ActionPrimitive.h>
#include <end_effector/GraspingActions/ActionTrig.h>

namespace {

class testComposedAction: public ::testing::Test {


protected:

    testComposedAction() : grasp("grasp", true) {
    }

    virtual ~testComposedAction() {
    }

    virtual void SetUp() override {
        
        std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();

        //if return false, models are not found and it is useless to continue the test
        ASSERT_TRUE(parserMoveIt->init ("robot_description", false)) ;

        ROSEE::FindActions actionsFinder (parserMoveIt);
        
        std::string folderForActions = "ROSEE/actions/" + parserMoveIt->getHandName();
        
        trigMap = actionsFinder.findTrig(ROSEE::ActionPrimitive::Type::Trig, folderForActions + "/primitives/") ;  

        if (trigMap.size() > 0){
            for (auto trig : trigMap) {
                std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
                    std::make_shared <ROSEE::ActionTrig> ( trig.second );
                grasp.sumAction ( pointer );  
            }

            ROSEE::YamlWorker yamlWorker;
            yamlWorker.createYamlFile (&grasp, folderForActions + "/generics/");
            
            //Parsing
            graspParsed = yamlWorker.parseYamlComposed (folderForActions + "/generics/grasp.yaml");
            
        } 
    }

    virtual void TearDown() override{
    }
    
    std::map < std::string , ROSEE::ActionTrig > trigMap;
    ROSEE::ActionComposed grasp;
    ROSEE::ActionComposed graspParsed;

   
};

TEST_F ( testComposedAction, checkNumberPrimitives ) {
    
    EXPECT_EQ ( grasp.numberOfInnerActions(), grasp.getInnerActionsNames().size() );
    
    EXPECT_EQ ( graspParsed.numberOfInnerActions(), graspParsed.getInnerActionsNames().size() );    
    
}

TEST_F ( testComposedAction, checkEmitParse ) {
    
    if (trigMap.size() > 0) { //if empty, no grasp is defined in the setup so test without meaning
        
        EXPECT_EQ (grasp.getName(), graspParsed.getName() );
        EXPECT_EQ (grasp.getType(), graspParsed.getType() );
        EXPECT_EQ (grasp.isIndependent(), graspParsed.isIndependent() );
        EXPECT_EQ (grasp.numberOfInnerActions(), graspParsed.numberOfInnerActions() );
        EXPECT_EQ (grasp.getFingersInvolved(), graspParsed.getFingersInvolved() );
        
        for (auto joint: grasp.getJointPos() ) {
            
            //compare size of joint (number of dofs)
            ASSERT_EQ (joint.second.size(), graspParsed.getJointPos().at(joint.first).size() );
            //loop the eventually multiple joint pos (when dofs > 1)
            for (int j = 0; j < joint.second.size(); ++j ){
                EXPECT_DOUBLE_EQ ( joint.second.at(j), graspParsed.getJointPos().at(joint.first).at(j) ); 
            }     
        }
        
        for (auto jointCount : grasp.getJointsInvolvedCount()) {

            EXPECT_EQ ( jointCount.second, graspParsed.getJointsInvolvedCount().at(jointCount.first) ); 
               
        }
    }
}

// if independent, at maximum only one primitive can influence each joint
TEST_F ( testComposedAction, checkIndependence ) { 
    if (grasp.isIndependent()) {
        for (auto it : grasp.getJointsInvolvedCount() ) {
            EXPECT_LE ( it.second, 1 );
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
    
    if ( ROSEE::TestUtils::prepareROSForTests ( argc, argv, "testComposedAction" ) != 0 ) {
        
        std::cout << "[TEST ERROR] Prepare Funcion failed" << std::endl;
        return -1;
    }
    
    
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
