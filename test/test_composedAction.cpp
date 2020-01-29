#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <ROSEndEffector/FindActions.h>
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
    
        ROSEE::FindActions actionsFinder ("robot_description");

        trigMap = actionsFinder.findTrig(ROSEE::ActionType::Trig, "/configs/actions/tests/") ;  

        for (auto trig : trigMap) {
            std::shared_ptr <ROSEE::ActionPrimitive> pointer = 
                std::make_shared <ROSEE::ActionTrig> ( trig.second );
            grasp.sumPrimitive ( pointer );  
        }

        
        ROSEE::YamlWorker yamlWorker(actionsFinder.getHandName(), "/configs/actions/tests/");
        yamlWorker.createYamlFile (&grasp);
        
        //Parsing
        graspParsed = yamlWorker.parseYamlComposed ("grasp.yaml");

       
    }

    virtual void TearDown() {
    }

    std::map < std::string , ROSEE::ActionTrig > trigMap;
    ROSEE::ActionComposed grasp;
    ROSEE::ActionComposed graspParsed;

   
};

TEST_F ( testComposedAction, checkNumberPrimitives ) {
    
    EXPECT_EQ ( grasp.getnPrimitives(), grasp.getPrimitiveNames().size() );
    EXPECT_EQ ( grasp.getnPrimitives(), grasp.getPrimitiveObjects().size() );
    
    EXPECT_EQ ( graspParsed.getnPrimitives(), graspParsed.getPrimitiveNames().size() );    
    
}

TEST_F ( testComposedAction, checkEmitParse ) {
    
    EXPECT_EQ (grasp.getName(), graspParsed.getName() );
    EXPECT_EQ (grasp.getnPrimitives(), graspParsed.getnPrimitives() );
    EXPECT_EQ (grasp.getIndependent(), graspParsed.getIndependent() );
    EXPECT_EQ (grasp.getLinksInvolved(), graspParsed.getLinksInvolved() );
    
    for (auto joint: grasp.getJointStates() ) {
        
        //compare size of joint (number of dofs)
        ASSERT_EQ (joint.second.size(), graspParsed.getJointStates().at(joint.first).size() );
        //loop the eventually multiple joint pos (when dofs > 1)
        for (int j = 0; j < joint.second.size(); ++j ){
            EXPECT_DOUBLE_EQ ( joint.second.at(j), graspParsed.getJointStates().at(joint.first).at(j) ); 
        }     
    }
}

// if independent, at maximum only one primitive can influence each joint
TEST_F ( testComposedAction, checkIndependence ) { 
    if (grasp.getIndependent()) {
        for (auto it : grasp.getInvolvedJointsCount() ) {
            EXPECT_LE ( it, 1);
        }
    }
}

} //namespace

int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
