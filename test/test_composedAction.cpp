#include <gtest/gtest.h>
#include "testUtils.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <ROSEndEffector/FindActions.h>
#include <ROSEndEffector/ParserMoveIt.h>
#include <ROSEndEffector/ActionComposed.h>
#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionTrig.h>

#define HAND_NAME_TEST "two_finger";

namespace {

class testComposedAction: public ::testing::Test {


protected:

    testComposedAction() : grasp("grasp", true) {
    }

    virtual ~testComposedAction() {
    }

    virtual void SetUp() override {
        
        //run roscore
        _roscore.reset(new ROSEE::TestUtils::Process({"roscore", "-p", "11322"}));
        
        //fill ros param with file models, needed by moveit parserMoveIt
        std::string modelPath = ROSEE::Utils::getPackagePath() + "configs/urdf/" + HAND_NAME_TEST;
        
        //Is there a better way to parse?
        std::ifstream urdf(modelPath + ".urdf");
        std::ifstream srdf(modelPath + ".srdf");
        std::stringstream sUrdf, sSrdf;
        sUrdf << urdf.rdbuf();
        sSrdf << srdf.rdbuf();
        
        ROS_WARN_STREAM ("SETTING PARAMS!!!!!");
        
        ros::param::set("robot_description" , sUrdf.str());
        ros::param::set("robot_description_semantic" , sUrdf.str());
        
        std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();

        //if return false, models are not found and it is useless to continue the test
        ASSERT_TRUE(parserMoveIt->init ("robot_description")) ;
        
        ROSEE::FindActions actionsFinder (parserMoveIt);
        
        std::string folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/tests/" + parserMoveIt->getHandName();
        
        trigMap = actionsFinder.findTrig(ROSEE::ActionPrimitive::Type::Trig, folderForActions + "/primitives/") ;  

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

    virtual void TearDown() {
    }
    
    std::unique_ptr<ROSEE::TestUtils::Process> _roscore;

    std::map < std::string , ROSEE::ActionTrig > trigMap;
    ROSEE::ActionComposed grasp;
    ROSEE::ActionComposed graspParsed;

   
};

TEST_F ( testComposedAction, checkNumberPrimitives ) {
    
    EXPECT_EQ ( grasp.numberOfInnerActions(), grasp.getInnerActionsNames().size() );
    
    EXPECT_EQ ( graspParsed.numberOfInnerActions(), graspParsed.getInnerActionsNames().size() );    
    
}

TEST_F ( testComposedAction, checkEmitParse ) {
    
    EXPECT_EQ (grasp.getName(), graspParsed.getName() );
    EXPECT_EQ (grasp.numberOfInnerActions(), graspParsed.numberOfInnerActions() );
    EXPECT_EQ (grasp.isIndependent(), graspParsed.isIndependent() );
    EXPECT_EQ (grasp.getFingersInvolved(), graspParsed.getFingersInvolved() );
    
    for (auto joint: grasp.getJointPos() ) {
        
        //compare size of joint (number of dofs)
        ASSERT_EQ (joint.second.size(), graspParsed.getJointPos().at(joint.first).size() );
        //loop the eventually multiple joint pos (when dofs > 1)
        for (int j = 0; j < joint.second.size(); ++j ){
            EXPECT_DOUBLE_EQ ( joint.second.at(j), graspParsed.getJointPos().at(joint.first).at(j) ); 
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
    
    /* Run tests on an isolated roscore */
    if(setenv("ROS_MASTER_URI", "http://localhost:11322", 1) == -1)
    {
        perror("setenv");
        return 1;
    }
    
    ros::init ( argc, argv, "testComposedAction" );
    
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
