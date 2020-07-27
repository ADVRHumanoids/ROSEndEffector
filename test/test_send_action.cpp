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
        
        
    }

    void actionActiveClbk() {
        completed = false;
    }
    
    sensor_msgs::JointState js;
    bool completed;

        
}

class testSendAction: public ::testing::Test {

    

protected:

    testSendAction() {
    }

    virtual ~testSendAction() {
    }

    virtual void SetUp() override {
        
        

        ROSEE::Parser p ( nh );
        p.init (  ROSEE::Utils::getPackagePath() + "/configs/test_ee.yaml" );
        
        ee = std::make_shared<ROSEE::EEInterface>(p);
        
        folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/" + ee->getName();
        




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
    sensor_msgs::JointState::ConstPtr& _msg;
    ROSEE::YamlWorker yamlWorker;

    
};


TEST_F ( testSendAction, sendSimpleGeneric ) {
    
    ROSEE::JointPos jp;

    //for now copy jp of another action 
    std::vector<std::string> actJoints = ee->getActuatedJoints();
    
    ASSERT_FALSE (actJoints.empty());
    
    for (int i = 0; i<actJoints.size(); i++) {
        
        int id = -1;
        ee->getInternalIdForJoint(actJoints.at(i), id);
        
        //create an action where all joints move toward their upper limit
        std::vector<double> one_dof;
        one_dof.push_back ( ee->getUpperPositionLimits()[id] );
        jp.insert ( std::make_pair(actJoints.at(i), one_dof) );
        
        
    }

    ROSEE::ActionGeneric simpleAction("testAllUpperLim", jp);
    //emit the yaml so roseeExecutor can find the action
    yamlWorker.createYamlFile( &simpleAction, folderForActions + "/generics/" );
    
    
    roseeExecutor.reset(new ROSEE::TestUtils::Process({"roslaunch", "ros_end_effector", "rosee_startup.launch", "hand_name:=test_ee"}));
    std::string topic_name_js;
    nh.param<std::string>("/rosee/joint_states_topic", topic_name_js, "/ros_end_effector/joint_states");
    
    receiveRobStateSub = nh.subscribe (topic_name_js, 1, &ClbkHelper::jointStateClbk, &clbkHelper);
    
    std::shared_ptr <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction> > action_client = 
        std::make_shared <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction>>
        (nh, "/action_command", false); //TODO check this action command
    
    rosee_msg::ROSEECommandGoal goal;
    goal.goal_action.seq = 0 ;
    goal.goal_action.stamp = ros::Time::now();
    goal.goal_action.percentage = 1;
    goal.goal_action.action_name = "testAllUpperLim";
    goal.goal_action.action_type = ROSEE::Action::Type::Generic ;
    goal.goal_action.actionPrimitive_type = ROSEE::ActionPrimitive::None ; //because it is not a primitive
    
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
        
        EXPECT_DOUBLE_EQ (wantedPos, realPos);
        
        
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
    
    if ( ROSEE::TestUtils::prepareROSForTests ( argc, argv, "testSendAction" ) != 0 ) {
        
        std::cout << "[TEST ERROR] Prepare Funcion failed" << std::endl;
        return -1;
    }

    
    
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
