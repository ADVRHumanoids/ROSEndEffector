#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/console.h>


#include <ROSEndEffector/FindActions.h>
#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionPinch.h>
#include <ROSEndEffector/ActionTrig.h>

namespace {

class testFindPinches: public ::testing::Test {


protected:

    testFindPinches() {
    }

    virtual ~testFindPinches() {
    }

    virtual void SetUp() override {

        char *argv[] = {"testFindPinches", "arg"};
        int argc = sizeof(argv) / sizeof(char*) - 1;
        
        ros::init ( argc, argv, "testFindPinches" );
        
        //add here if want to try other hands. HOW to run all togheter?
        std::string hand4Test = "svh-standalone"; // "two_finger", "test_ee"
        
        ///need to set param in this test, in non test they are setted with launch file
        ros::NodeHandle nh;
        
        // Parse the urdf file
        std::string urdf_file_name = ROSEE::Utils::getPackagePath() + 
            "/configs/urdf/" + hand4Test + ".urdf";
        std::ifstream f1(urdf_file_name); 
        std::stringstream ss1;
        ss1 << f1.rdbuf();
        nh.setParam("robot_description", ss1.str());
        
        // Parse the srdf file
        std::string srdf_file_name = ROSEE::Utils::getPackagePath() + 
            "/configs/srdf/" + hand4Test + ".srdf";
        std::ifstream f2 (srdf_file_name); 
        std::stringstream ss2;
        ss2 << f2.rdbuf();
        nh.setParam("robot_description_semantic", ss2.str());
    
        ROSEE::FindActions actionsFinder ("robot_description");

        //TODO test if this return and pinchParsedMap are equal
        actionsFinder.findPinch();

        //TODO getHandName should be in the parser
        ROSEE::YamlWorker yamlWorker(actionsFinder.getHandName());
   
        pinchParsedMap = yamlWorker.parseYaml("pinch.yaml", ROSEE::ActionType::Pinch);

    }

    virtual void TearDown() {
    }

    std::map < std::set < std::string>, std::shared_ptr<ROSEE::ActionPrimitive> > pinchParsedMap;
};


TEST_F ( testFindPinches, checkNumberLinks ) {
    
    for (auto &mapEl: pinchParsedMap ) {
        
        EXPECT_EQ (2, mapEl.first.size()); // the key
        EXPECT_EQ (2, mapEl.second->getLinksInvolved().size()); //the names inside the action
        EXPECT_EQ (2, mapEl.second->getnLinksInvolved()); //the int nLinkInvolved member of action
    }
}

TEST_F ( testFindPinches, checkSizeStatesInfoSet ) {
    
    for (auto &mapEl: pinchParsedMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second->getJointStatesSetMaxSize(); 
        
        //it must be equal to the real size of the statesInfoSet
        EXPECT_EQ (size, mapEl.second->getActionStates().size());
    }
}

TEST_F ( testFindPinches, checkName ) {
    
    for (auto &mapEl: pinchParsedMap ) {
        
        EXPECT_TRUE (mapEl.second->getName().compare("pinch"));
        EXPECT_EQ (ROSEE::ActionType::Pinch, mapEl.second->getActionType() );
    }
}

//this is an important test: check if the order of statesInfo in right according to depth
TEST_F ( testFindPinches, checkOrderStatesInfoSet ) {
    
    for (auto &mapEl: pinchParsedMap ) { 
        
        std::shared_ptr <ROSEE::ActionPinch> pinchCasted = 
            std::dynamic_pointer_cast < ROSEE::ActionPinch > (mapEl.second);
            
        ASSERT_FALSE (pinchCasted == nullptr);
        std::vector < ROSEE::ActionPinch::StateWithContact> statesInfo = 
            pinchCasted->getActionStatesWithContact();
        
        double oldDepth = std::numeric_limits<double>::infinity();
        
        for (auto &setEl : statesInfo) {
            EXPECT_LE (std::abs(setEl.second.depth), std::abs(oldDepth) ); //lesser or equal
            oldDepth = setEl.second.depth;
        }
    }
}



} //namespace

int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
