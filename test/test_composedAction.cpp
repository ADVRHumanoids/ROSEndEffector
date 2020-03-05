#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <ROSEndEffector/FindActions.h>
#include <ROSEndEffector/ParserMoveIt.h>
#include <ROSEndEffector/MapActionHandler.h>
#include <ROSEndEffector/ActionComposed.h>
#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionTrig.h>

namespace {

class testComposedAction: public ::testing::Test {


protected:

    testComposedAction() : grasp("grasp", true) {
    }

    virtual ~testComposedAction() {
    }

    virtual void SetUp() override {
        
        const char *argv[] = {"testComposedAction", "arg"};
        int argc = sizeof(argv) / sizeof(char*) - 1;
        
        //is this cast correct?
        ros::init ( argc, (char**)argv, "testComposedAction" );
    
        std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();
        parserMoveIt->init ("robot_description") ;
        ROSEE::FindActions actionsFinder (parserMoveIt);
        
        std::string folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/" + parserMoveIt->getHandName();
        
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
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
