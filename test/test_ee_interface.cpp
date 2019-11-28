#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/JointState.h>

#include <ROSEndEffector/Parser.h>
#include <ROSEndEffector/EEInterface.h>

namespace {

class testEEInterface: public ::testing::Test {


protected:

    testEEInterface() {
    }

    virtual ~testEEInterface() {
    }

    virtual void SetUp() {

        char *argv[] = {"testEEInterface", "arg"};
        int argc = sizeof(argv) / sizeof(char*) - 1;
        
        ros::init ( argc, argv, "testEEInterface" );
        ros::NodeHandle nh;

        ROSEE::Parser p ( nh );
        p.init ( "/home/lucamuratore/src/ros_end_effector__ws/src/ROSEndEffector/configs/test_ee.yaml" );
        p.printEndEffectorFingerJointsMap();

        ee = p.getEndEffectorInterface();
    }

    virtual void TearDown() {
    }

    ROSEE::EEInterface::Ptr ee;
};


TEST_F ( testEEInterface, checkFingers ) {

    std::vector<std::string> fingers;
    fingers = ee->getFingers();
    
    EXPECT_FALSE (fingers.empty());

    ROS_INFO_STREAM ( "Fingers in EEInterface: " );
    for ( auto& f : fingers ) {
        ROS_INFO_STREAM ( f );
    }
    
    EXPECT_TRUE ( ee->isFinger ( "finger_1" ) );

    EXPECT_FALSE ( ee->isFinger ( "finger_4" ) );
    
    

}

TEST_F ( testEEInterface, checkActuatedJointsNum ) {

    int joint_num = ee->getActuatedJointsNum();
    EXPECT_EQ ( 6, joint_num );
    
    EXPECT_FALSE ( joint_num < 0 );

}

TEST_F ( testEEInterface, checkEEFingerJoints ) {

    int joint_num_finger1 = ee->getActuatedJointsNumInFinger("finger_1");
    
    int joint_num = ee->getActuatedJointsNum();
    
    EXPECT_TRUE ( joint_num >= joint_num_finger1 );
    
    int joint_num_counter = 0;
    std::vector<std::string> fingers = ee->getFingers();
    
    
    
    for ( auto& f : fingers ) {
        joint_num_counter += ee->getActuatedJointsNumInFinger(f);
    }
    
     EXPECT_TRUE ( joint_num == joint_num_counter );

}



} //namespace

int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
