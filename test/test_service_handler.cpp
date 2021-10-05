#include <gtest/gtest.h>
#include "testUtils.h"

#include <ros/ros.h>

#include <ros_end_effector/Parser.h>
#include <ros_end_effector/EEInterface.h>

#include <ros_end_effector/RosServiceHandler.h>
#include <ros_end_effector/MapActionHandler.h>

#include <rosee_msg/NewGenericGraspingActionSrv.h>



namespace {

/**
 * @brief Test for the RosServiceHandler Class
 * It simply create the server and some clients which will test the server functions.
 * Hence, in this test a lot of ROSEE main class are not used like the FindAction
 */
class testServiceHandler: public ::testing::Test {


protected:

    testServiceHandler()  {
    }

    virtual ~testServiceHandler() {
    }

    virtual void SetUp() override {

        if (! nh.hasParam("robot_name")) {
            std::cout << "[TEST FAIL: robot name not set on server]" << std::endl;
            return;
        }

        robot_name = "";
        nh.getParam("robot_name", robot_name);
        
        std::string handNameArg = "hand_name:=" + robot_name;
        roseeExecutor.reset(new ROSEE::TestUtils::Process({"roslaunch", "end_effector", "test_rosee_startup.launch", handNameArg}));

    }

    virtual void TearDown() override {
    }

    std::string robot_name;
    ros::NodeHandle nh;
    std::unique_ptr<ROSEE::TestUtils::Process> roseeExecutor;

    template <class clientType>
    bool initClient( ros::ServiceClient& rosee_client, std::string serviceName) {

        rosee_client = nh.serviceClient<clientType>(serviceName);

        rosee_client.waitForExistence();

        return true;

    };

};

/**
 * We call the service to include a new generic action multiple time, with both wrong and fallace requests, to see if the errors are 
 * detected by the server, and correct action are accepted
 */
TEST_F ( testServiceHandler, callNewAction ) {

    sleep(1); //without this joint state publisher crashes I do not know why (it is useless in this test, but annoying prints on traceback would appear)
    
    ros::ServiceClient rosee_client; 
    initClient<rosee_msg::NewGenericGraspingActionSrv>(rosee_client, "/ros_end_effector/new_generic_grasping_action");

    rosee_msg::NewGenericGraspingActionSrv newActionSrv;

    //NOTE the server respond always with a true, and fill an error msg in the request if error happened

    //empty request, error
    EXPECT_TRUE( rosee_client.call(newActionSrv) );
    EXPECT_FALSE( newActionSrv.response.accepted );
    EXPECT_FALSE( newActionSrv.response.emitted );
    EXPECT_TRUE( newActionSrv.response.error_msg.size() > 0 );
    
    //empty joint pos, error
    newActionSrv.request.newAction.action_name = "newAction";
    EXPECT_TRUE( rosee_client.call(newActionSrv) );
    EXPECT_FALSE( newActionSrv.response.accepted );
    EXPECT_FALSE( newActionSrv.response.emitted );
    EXPECT_TRUE( newActionSrv.response.error_msg.size() > 0 );

    //correct request
    newActionSrv.request.newAction.action_motor_position.name.push_back("joint_1");
    newActionSrv.request.newAction.action_motor_position.position.push_back(1);
    EXPECT_TRUE( rosee_client.call(newActionSrv) );
    EXPECT_TRUE( newActionSrv.response.accepted );
    EXPECT_FALSE( newActionSrv.response.emitted );
    EXPECT_FALSE( newActionSrv.response.error_msg.size() > 0 );

    //same request, this time is must fail because a "test1" action is already present
    EXPECT_TRUE( rosee_client.call(newActionSrv) );
    EXPECT_FALSE( newActionSrv.response.accepted );
    EXPECT_FALSE( newActionSrv.response.emitted );
    EXPECT_TRUE( newActionSrv.response.error_msg.size() > 0 );
    
    //  error, the joint names in motor pos and count  are not the same
    //NOTE we use the time so if this test is run multiple time on same machine, the action will have a different name
    newActionSrv.request.newAction.action_name = "newAction_" + std::to_string(ros::Time::now().toSec());
    newActionSrv.request.newAction.action_motor_count.name.push_back("error_joint_1");
    newActionSrv.request.newAction.action_motor_count.count.push_back(1);
    EXPECT_TRUE( rosee_client.call(newActionSrv) );
    EXPECT_FALSE( newActionSrv.response.accepted );
    EXPECT_FALSE( newActionSrv.response.emitted );
    EXPECT_TRUE( newActionSrv.response.error_msg.size() > 0 );
    
    //  correcting previous error, also emitting on yaml
    newActionSrv.request.newAction.action_motor_count.name.at(0) = "joint_1";
    newActionSrv.request.newAction.action_motor_count.count.at(0) = 1;
    newActionSrv.request.emitYaml = true;
    EXPECT_TRUE( rosee_client.call(newActionSrv) );
    EXPECT_TRUE( newActionSrv.response.accepted );
    EXPECT_TRUE( newActionSrv.response.emitted );
    EXPECT_FALSE( newActionSrv.response.error_msg.size() > 0 );
    
}

/***
 * Here we send some new action to newGraspingActionServer, and we see if we can retrieve them with another server, GraspingActionAvailable
 */
TEST_F ( testServiceHandler, callNewActionAndRetrieve ) {
    
    sleep(1); //without this joint state publisher crashes I do not know why (it is useless in this test, but annoying prints on traceback would appear)
    
    ros::ServiceClient rosee_client_new_action, rosee_client_actions_available; 
    initClient<rosee_msg::NewGenericGraspingActionSrv>(rosee_client_new_action, "/ros_end_effector/new_generic_grasping_action");
    initClient<rosee_msg::GraspingActionsAvailable>(rosee_client_actions_available, "/ros_end_effector/grasping_actions_available");

    rosee_msg::NewGenericGraspingActionSrv newActionSrv;
    
    std::string requestActionName = "newAction_TEST";
    newActionSrv.request.newAction.action_name = requestActionName;
    newActionSrv.request.newAction.action_motor_position.name.push_back("joint_1");
    newActionSrv.request.newAction.action_motor_position.position.push_back(1);
    newActionSrv.request.newAction.elements_involved.push_back("an_element");
    newActionSrv.request.emitYaml = false;
    EXPECT_TRUE( rosee_client_new_action.call(newActionSrv) );
    EXPECT_TRUE( newActionSrv.response.accepted );
    EXPECT_FALSE( newActionSrv.response.emitted );
    EXPECT_FALSE( newActionSrv.response.error_msg.size() > 0 );
    
    
    rosee_msg::GraspingActionsAvailable graspActionAvailable;
    
    //we ask for a not existing action, and check for the error
    graspActionAvailable.request.action_name = "newAction_TEST_NOT_EXISTENT";
    EXPECT_TRUE(rosee_client_actions_available.call(graspActionAvailable));
    EXPECT_EQ(graspActionAvailable.response.grasping_actions.size(), 0);
    
    //we ask for the requestActionName, and check if fields are the same sent by us
    graspActionAvailable.request.action_name = requestActionName;
    //Compulsory field, otherwise primitive is asked
    graspActionAvailable.request.action_type = 1;
    EXPECT_TRUE(rosee_client_actions_available.call(graspActionAvailable));
    EXPECT_EQ(1, graspActionAvailable.response.grasping_actions.size());
    if (graspActionAvailable.response.grasping_actions.size() > 0) {
        auto receivedAction = graspActionAvailable.response.grasping_actions.at(0);
        EXPECT_EQ(receivedAction.action_name, requestActionName);
        EXPECT_EQ(receivedAction.action_motor_positions.at(0).name.size(),  newActionSrv.request.newAction.action_motor_position.name.size());
        EXPECT_EQ(receivedAction.action_motor_positions.at(0).name.at(0),  newActionSrv.request.newAction.action_motor_position.name.at(0));
        EXPECT_EQ(receivedAction.action_motor_positions.at(0).position.at(0),  newActionSrv.request.newAction.action_motor_position.position.at(0));
        //motor count was not filled here but it is filled by default by action costructor
        EXPECT_EQ(1, receivedAction.action_motor_count.name.size());
        EXPECT_EQ(receivedAction.action_motor_count.name.at(0), newActionSrv.request.newAction.action_motor_position.name.at(0));
        EXPECT_EQ(1, receivedAction.action_motor_count.count.at(0));
        EXPECT_EQ(1, receivedAction.elements_involved.size());
        if (receivedAction.elements_involved.size() > 0){
            EXPECT_EQ(receivedAction.elements_involved.at(0), newActionSrv.request.newAction.elements_involved.at(0));
        }
    }

    
     
     
}



} //namespace



int main ( int argc, char **argv ) {

    if (argc < 2 ) {

        std::cout << "[TEST ERROR] Insert hand name as argument" << std::endl;
        return -1;
    }

    /******************* Run tests on an isolated roscore ********************************************************/
    /* COMMENT ALL THIS block if you want to use roscore command by hand (useful for debugging the test) */
    //TODO I do not really understand why everything fails if I put this block at the beginning of prepareROSForTests function
    if(setenv("ROS_MASTER_URI", "http://localhost:11322", 1) == -1)
    {
        perror("setenv");
        return 1;
    }

    //run roscore
    std::unique_ptr<ROSEE::TestUtils::Process> roscore;
    roscore.reset(new ROSEE::TestUtils::Process({"roscore", "-p", "11322"}));
    /****************************************************************************************************/

    ros::init ( argc, argv, "testServiceHandler" );

    ros::param::set("robot_name", argv[1]);

    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
