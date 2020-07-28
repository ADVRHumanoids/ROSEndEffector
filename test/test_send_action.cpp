#include <gtest/gtest.h>
#include "testUtils.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <ros_end_effector/Parser.h>
#include <ros_end_effector/EEInterface.h>
#include <ros_end_effector/ActionGeneric.h>
#include <ros_end_effector/Utils.h>
#include <ros_end_effector/YamlWorker.h>

#include <rosee_msg/ROSEECommandAction.h>
#include <actionlib/client/simple_action_client.h>



namespace {
    
struct ClbkHelper {
    
    ClbkHelper() : js(), completed(false) {};
    
    void jointStateClbk(const sensor_msgs::JointState::ConstPtr& msg) {
        
        js.name = msg->name;
        js.position = msg->position;
        js.velocity = msg->velocity;
        js.effort = msg->effort;
    
    }

    void actionDoneClbk(const actionlib::SimpleClientGoalState& state,
                const rosee_msg::ROSEECommandResultConstPtr& result) {
                
        completed = true;
        
    }

    void actionFeedbackClbk(const rosee_msg::ROSEECommandFeedbackConstPtr& feedback) {
        
        feedback_percentage = feedback->completation_percentage;

        
    }

    void actionActiveClbk() {
        completed = false;
    }
    
    sensor_msgs::JointState js;
    bool completed;
    double feedback_percentage;

        
};


/**
 * @brief This Tests are to check if actions are sent correctly. Check each TEST_F for better explanation
 * 
 * @warning @todo There are a lot of prints that seems to be not in the right order. Probably is because we use 
 *   the ROS STREAM that effectively prints only when the spin is called...
 *   BTW it is strange that the parser prints are printed at the end, and not while setup function is running (where they
 *   should be because we init the parser there. This does not cause problems at the test.
 */
class testSendAction: public ::testing::Test {

    

protected:

    testSendAction() {
    }

    virtual ~testSendAction() {
    }

    virtual void SetUp() override {
        
        if (! nh.hasParam("robot_name")) {
            std::cout << "[TEST FAIL: robot name not set on server]" << std::endl;
            return;
        }
        
        std::string robot_name = "";
        nh.getParam("robot_name", robot_name);
        
        ROSEE::Parser p ( nh );
        if (! p.init (  ROSEE::Utils::getPackagePath() + "/configs/" + robot_name + ".yaml" )) {
            
            std::cout << "[TEST parser FAIL: some config file missing]" << std::endl;
            return;
        }
        
        ee = std::make_shared<ROSEE::EEInterface>(p);
        
        folderForActions = p.getActionPath();
        if ( folderForActions.size() == 0 ){ //if no action path is set in the yaml file...
            folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/" + ee->getName();
        }
    
    }

    virtual void TearDown() override {
    }
    

    ClbkHelper clbkHelper;
    ros::NodeHandle nh;
    std::string folderForActions;
    std::unique_ptr<ROSEE::TestUtils::Process> roseeExecutor;
    ROSEE::EEInterface::Ptr ee;
    ros::Publisher sendActionPub;
    ros::Subscriber receiveRobStateSub;
    ROSEE::YamlWorker yamlWorker;

    
};

/**
 * @brief This Test create a Generic Action and see if it is sent correctly.
 *  - An ActionGeneric is created, where the actuated Joints are set to their upper limit
 *  - The ROSEE main node (which receives action commands and output specific joint pos references) is launched
 *       such that the action created is parsed by him
 *  - The action is commanded.
 * 
 *  This test check that, when the ROSEE says the action is completed, the joints positions are effectively the ones
 *   put in the actionGeneric created. So in someway it bypass ROSEE when checking directly the robot state.
 */
TEST_F ( testSendAction, sendSimpleGeneric ) {
    
    
    ROSEE::JointPos jp;
    ROSEE::JointsInvolvedCount jpc;

    //for now copy jp of another action 
    std::vector<std::string> actJoints = ee->getActuatedJoints();
    
    ASSERT_FALSE (actJoints.empty());
    
    for (int i = 0; i<actJoints.size(); i++) {
        
        std::cout << actJoints[i] << std::endl;
        
        int id = -1;
        ee->getInternalIdForJoint(actJoints.at(i), id);
        
        //create an action where all joints move toward their upper limit
        std::vector<double> one_dof;
        one_dof.push_back ( ee->getUpperPositionLimits()[id] );
        jp.insert ( std::make_pair(actJoints.at(i), one_dof) );
        jpc.insert (std::make_pair(actJoints.at(i), 1));
        
    }

    ROSEE::ActionGeneric simpleAction("testAllUpperLim", jp, jpc);
    //emit the yaml so roseeExecutor can find the action
    yamlWorker.createYamlFile( &simpleAction, folderForActions + "/generics/" );
    
    std::string handNameArg = "hand_name:=" + ee->getName();
    roseeExecutor.reset(new ROSEE::TestUtils::Process({"roslaunch", "ros_end_effector", "test_rosee_startup.launch", handNameArg}));

    //TODO put a checkReady service instead of sleeping?
    sleep(5); // lets wait for test_rosee_startup to be ready
    std::string topic_name_js;

    nh.param<std::string>("/rosee/joint_states_topic", topic_name_js, "");
    
    ASSERT_TRUE ( topic_name_js.size() > 0);
    
    receiveRobStateSub = nh.subscribe (topic_name_js, 1, &ClbkHelper::jointStateClbk, &clbkHelper);
    
    std::shared_ptr <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction> > action_client = 
        std::make_shared <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction>>
        (nh, "/ros_end_effector/action_command", true); //TODO check this action command

    action_client->waitForServer();
    
    rosee_msg::ROSEECommandGoal goal;
    goal.goal_action.seq = 0 ;
    goal.goal_action.stamp = ros::Time::now();
    goal.goal_action.percentage = 1;
    goal.goal_action.action_name = "testAllUpperLim";
    goal.goal_action.action_type = ROSEE::Action::Type::Generic ;
    goal.goal_action.actionPrimitive_type = ROSEE::ActionPrimitive::Type::None ; //because it is not a primitive
    
    action_client->sendGoal (goal, boost::bind(&ClbkHelper::actionDoneClbk, &clbkHelper, _1, _2),
        boost::bind(&ClbkHelper::actionActiveClbk, &clbkHelper), boost::bind(&ClbkHelper::actionFeedbackClbk, &clbkHelper, _1)) ;
         
    ros::Rate r(10); // 10 hz
    while (!clbkHelper.completed) {
        ROS_INFO_STREAM_ONCE("Test Send Actions: Waiting for action completion...");
        ros::spinOnce();
        r.sleep();
    }
    
    //finally, lets test if the pos set in the actions are the same of the robot when the action is completed
    for (int i=0; i < clbkHelper.js.name.size(); i++) {
        
        
        //create an action where all joints move toward their upper limit
        auto wantedJointsPosMap = simpleAction.getJointPos();
        
        auto findJoint = wantedJointsPosMap.find(clbkHelper.js.name[i]);
        
        if (findJoint == wantedJointsPosMap.end()){
            continue; //this is not an error, in clbkHelper we have joint pos of all joints, not only actuated
        }
        
        double wantedPos = findJoint->second.at(0);
        double realPos = clbkHelper.js.position[i];
        
        EXPECT_NEAR (wantedPos, realPos, 0.02);  //norm of the accepted error in roseeExecutor is <0.01

        //the percentage must be exaclty 100 instead (apart double precisions errors, handled by the macro)
        EXPECT_DOUBLE_EQ (clbkHelper.feedback_percentage, 100); 
        
    }
    
}



} //namespace

int main ( int argc, char **argv ) {
    
    if (argc < 2 ){
        
        std::cout << "[TEST ERROR] Insert hand name as argument" << std::endl;
        return -1;
    }
    
    /******************* Run tests on an isolated roscore ********************************************************/
    /* COMMENT ALL THIS block if you want to use roscore command by hand (useful for debugging the test) */
    if(setenv("ROS_MASTER_URI", "http://localhost:11322", 1) == -1)
    {
       perror("setenv");
       return 1;
    }

    //run roscore
    std::unique_ptr<ROSEE::TestUtils::Process> roscore;
    roscore.reset(new ROSEE::TestUtils::Process({"roscore", "-p", "11322"}));
    /****************************************************************************************************/
    
    if ( ROSEE::TestUtils::prepareROSForTests ( argc, argv, "testSendAction" ) != 0 ) {
        
        std::cout << "[TEST ERROR] Prepare Function failed" << std::endl;
        return -1;
    }
    
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
