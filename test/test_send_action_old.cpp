#include <gtest/gtest.h>
#include "testUtils.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <ros_end_effector/FindActions.h>
#include <ros_end_effector/ParserMoveIt.h>
#include <ros_end_effector/Parser.h>
//#include <ros_end_effector/ActionPrimitive.h>
//#include <ros_end_effector/ActionTrig.h>
//#include <ros_end_effector/ActionPi.h>

#include <ros_end_effector/MapActionHandler.h>



namespace {

class testSendAction: public ::testing::Test {


protected:

    testSendAction() {
    }

    virtual ~testSendAction() {
    }

    virtual void SetUp() override {
        
        //the actions are emitted before in the main()
        std::string folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/" + _ee->getName();

        // get all action in the handler
        ROSEE::MapActionHandler mapActionHandler;
        mapActionHandler.parseAllActions(folderForActions);
       
    }

    virtual void TearDown() override {
    }

    
};


TEST_F ( testSendAction, checkNumberLinks ) {
    
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
    
    if ( ROSEE::TestUtils::prepareROSForTests ( argc, argv, "testSendAction" ) != 0 ) {
        
        std::cout << "[TEST ERROR] Prepare Funcion failed" << std::endl;
        return -1;
    }
    
    //extract the primitives, not the purpose of this test to test the extracting phase (there are other tests for this)
    ros::NodeHandle nh;
    ROSEE::Parser parser(nh);
    parser.init();
    //Load the ROS Server with urdf and srdf 
    nh.setParam("/robot_description", parser.getUrdfString());
    nh.setParam("/robot_description_semantic", parser.getSrdfString());
    ROS_INFO_STREAM("TEST_Send_action: Set urdf and srdf file in the param server from config file " << parser.getRoseeConfigPath());
    
    std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();
    if (! parserMoveIt->init ("robot_description") ) {
        ROS_ERROR_STREAM ("TEST_Send_action: FAILED parserMoveit Init, stopping execution");
        return -1;
    }
    
    std::string folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/tests/" + parserMoveIt->getHandName();
    
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

    
    
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
