#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/JointState.h>

#include <ROSEndEffector/Parser.h>
#include <ROSEndEffector/EEInterface.h>
#include <ROSEndEffector/Utils.h>

namespace {

class testEEInterface: public ::testing::Test {


protected:

    testEEInterface() {
    }

    virtual ~testEEInterface() {
    }

    virtual void SetUp() {

        const char *argv[] = {"testEEInterface", "arg"};
        int argc = sizeof(argv) / sizeof(char*) - 1;
        
        ros::init ( argc, const_cast<char **> (argv), "testEEInterface" );
        ros::NodeHandle nh;

        ROSEE::Parser p ( nh );
        p.init ( ROSEE::Utils::getPackagePath() + "/configs/test_ee.yaml" );
        p.printEndEffectorFingerJointsMap();

        ee = std::make_shared<ROSEE::EEInterface>(p);
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

TEST_F ( testEEInterface, checkJointLimits) {

    Eigen::VectorXd upperLimits = ee->getUpperPositionLimits();
    Eigen::VectorXd lowerLimits = ee->getLowerPositionLimits();
    
    ASSERT_EQ (upperLimits.size(), lowerLimits.size()); //stop if it fails here
    
    EXPECT_TRUE (upperLimits.size() > 0);
    
    for (int i=0; i<upperLimits.size(); i++) {
    
        EXPECT_GE (upperLimits(i), lowerLimits(i)); //greater or equal than
        ROS_INFO_STREAM ( "Joint " << std::to_string(i) << " limits:  " <<
                          upperLimits(i) <<  ", " << lowerLimits(i) );

    }
    
}

TEST_F ( testEEInterface, checkIdJoints ) {

    std::vector<std::string> actJoints;
    ee->getActuatedJoints(actJoints);
   // ASSERT_FALSE (actJoints.empty());  //a hand can have no actuated joints?
    
    // check if ids are unique
    int id = -1;
    int idPrevious = -1;
    for ( auto& j : actJoints ) {
        EXPECT_TRUE ( ee->getInternalIdForJoint (j, id) ); //return false if joint does not exist
        EXPECT_NE ( id, idPrevious );
        idPrevious = id;
    }
    
}

} //namespace

int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
