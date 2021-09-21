#include <gtest/gtest.h>
#include "testUtils.h"

#include <ros/ros.h>

#include <ros_end_effector/FindActions.h>
#include <ros_end_effector/ParserMoveIt.h>
#include <ros_end_effector/GraspingActions/ActionPrimitive.h>
#include <ros_end_effector/GraspingActions/ActionPinchTight.h>
#include <ros_end_effector/GraspingActions/ActionPinchLoose.h>


namespace {

class testFindPinches: public ::testing::Test {


protected:

    testFindPinches() {
    }

    virtual ~testFindPinches() {
    }

    virtual void SetUp() override {
    
        std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();
        //if return false, models are not found and it is useless to continue the test
        ASSERT_TRUE(parserMoveIt->init ("robot_description", false)) ;
        ROSEE::FindActions actionsFinder (parserMoveIt);
        
        std::string folderForActions = ROSEE::Utils::getPackagePath() + "/configs/actions/tests/" + parserMoveIt->getHandName();

        auto theTwoMaps = actionsFinder.findPinch(folderForActions + "/primitives/");

        pinchMap = theTwoMaps.first;
        pinchLooseMap = theTwoMaps.second;

        ROSEE::YamlWorker yamlWorker;

        if (pinchMap.size() > 0 ) {
            pinchParsedMap = yamlWorker.parseYamlPrimitive(folderForActions + "/primitives/" + "pinchTight.yaml" );
        }
        
        if (pinchLooseMap.size() > 0 ) {
            pinchLooseParsedMap = yamlWorker.parseYamlPrimitive(folderForActions + "/primitives/" + "pinchLoose.yaml");
        }
        
    }

    virtual void TearDown() {
    }

    std::map < std::pair < std::string, std::string >, ROSEE::ActionPinchTight > pinchMap;
    std::map < std::set < std::string >, std::shared_ptr<ROSEE::ActionPrimitive> > pinchParsedMap;
    
    std::map < std::pair < std::string, std::string >, ROSEE::ActionPinchLoose > pinchLooseMap;
    std::map < std::set < std::string >, std::shared_ptr<ROSEE::ActionPrimitive> > pinchLooseParsedMap;
};


TEST_F ( testFindPinches, checkNumberLinks ) {
    
    for (auto &mapEl: pinchMap ) {
        
        //being a pair the .first has always dimension 2
        EXPECT_EQ (2, mapEl.second.getFingersInvolved().size() ); //the names inside the action
        EXPECT_EQ (2, mapEl.second.getnFingersInvolved() ); //the int nLinkInvolved member of action
    }
    
    for (auto &mapEl: pinchParsedMap ) {
        
        EXPECT_EQ (2, mapEl.first.size()); // the key
        EXPECT_EQ (2, mapEl.second->getFingersInvolved().size()); //the names inside the action
        EXPECT_EQ (2, mapEl.second->getnFingersInvolved()); //the int nLinkInvolved member of action
    }
}

TEST_F ( testFindPinches, checkSizeStatesInfoSet ) {
    
    for (auto &mapEl: pinchMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second.getMaxStoredActionStates(); 
        
        //it must be equal to the real size of the actionState set
        EXPECT_EQ ( size, mapEl.second.getActionStates().size() );
        EXPECT_EQ ( size, mapEl.second.getAllJointPos().size() );
    }
    
    for (auto &mapEl: pinchParsedMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second->getMaxStoredActionStates(); 
        
        EXPECT_EQ (size, mapEl.second->getAllJointPos().size());
    }
}

TEST_F ( testFindPinches, checkName ) {
    
    for (auto &mapEl: pinchMap ) {
        
        EXPECT_TRUE (mapEl.second.getName().compare("pinchTight") == 0);
        EXPECT_EQ (ROSEE::ActionPrimitive::Type::PinchTight, mapEl.second.getPrimitiveType() );
    }
    
    for (auto &mapEl: pinchParsedMap ) {
        
        EXPECT_TRUE (mapEl.second->getName().compare("pinchTight") == 0);
        EXPECT_EQ (ROSEE::ActionPrimitive::Type::PinchTight, mapEl.second->getPrimitiveType() );
    }
}

//this is an important test: check if the order of statesInfo in right according to depth
TEST_F ( testFindPinches, checkOrderStatesInfoSet ) {
    
    for (auto &mapEl: pinchMap ) { 
        
        std::vector < ROSEE::ActionPinchTight::StateWithContact> statesInfo = 
            mapEl.second.getActionStates();
        
        double oldDepth = std::numeric_limits<double>::infinity();
        
        for (auto &setEl : statesInfo) {
            EXPECT_LE (std::abs(setEl.second.depth), std::abs(oldDepth) ); //lesser or equal
            oldDepth = setEl.second.depth;
        }
    }
    
    
    for (auto &mapEl: pinchParsedMap ) { 
        
        std::shared_ptr <ROSEE::ActionPinchTight> pinchCasted = 
            std::dynamic_pointer_cast < ROSEE::ActionPinchTight > (mapEl.second);
            
        ASSERT_FALSE (pinchCasted == nullptr);
        std::vector < ROSEE::ActionPinchTight::StateWithContact> statesInfo = 
            pinchCasted->getActionStates();
        
        double oldDepth = std::numeric_limits<double>::infinity();
        
        for (auto &setEl : statesInfo) {
            EXPECT_LE (std::abs(setEl.second.depth), std::abs(oldDepth) ); //lesser or equal
            oldDepth = setEl.second.depth;
        }
    }
}

// to check if the found map is the same map that is emitted in the file and then parsed
TEST_F ( testFindPinches, checkEmitParse ) {
    
    ASSERT_EQ (pinchMap.size(), pinchParsedMap.size() );
    
    for (auto &mapEl: pinchParsedMap ) { 
        
        std::shared_ptr <ROSEE::ActionPinchTight> pinchCasted = 
            std::dynamic_pointer_cast < ROSEE::ActionPinchTight > (mapEl.second);
            
        ASSERT_FALSE (pinchCasted == nullptr);
        ASSERT_EQ (2, mapEl.first.size() );
        std::pair <std::string, std::string> keyPair;
        std::set<std::string>::iterator it = mapEl.first.begin();
        keyPair.first = *it;
        std::advance ( it, 1 );
        keyPair.second = *it;
                
        //std::string is ok to compare with _EQ
        EXPECT_EQ (pinchCasted->getName(), pinchMap.at(keyPair).getName() );
        EXPECT_EQ (pinchCasted->getType(), pinchMap.at(keyPair).getType() );
        EXPECT_EQ (pinchCasted->getnFingersInvolved(), pinchMap.at(keyPair).getnFingersInvolved() );
        EXPECT_EQ (pinchCasted->getMaxStoredActionStates(), pinchMap.at(keyPair).getMaxStoredActionStates());
        EXPECT_EQ (pinchCasted->getPrimitiveType(), pinchMap.at(keyPair).getPrimitiveType() );
        EXPECT_EQ (pinchCasted->getFingersInvolved(), pinchMap.at(keyPair).getFingersInvolved());

        unsigned int i = 0;
        for (auto as: pinchCasted->getActionStates() ) {
            
            //check equality of joint states (as.first)
            for (auto joint : as.first) {
                ASSERT_EQ ( joint.second.size(), 
                            pinchMap.at(keyPair).getAllJointPos().at(i).at(joint.first).size() );
                //loop the eventually multiple joint pos (when dofs > 1)
                for (int j=0; j<joint.second.size(); ++j){
                    EXPECT_DOUBLE_EQ ( joint.second.at(j),
                        pinchMap.at(keyPair).getAllJointPos().at(i).at(joint.first).at(j) ); 
                }
            }   
            
            //check equality of contact (as.second)
            collision_detection::Contact thisCont = as.second;
            collision_detection::Contact otherCont =
                pinchMap.at(keyPair).getActionStates().at(i).second;
            // Tricky here, body colliding names can be swapped.
            // BUT it seems that depth, normal and pos refer to a fixed something, so they must
            // not be swapped (I don't see that EXPECT fails if I don't swap them when names are swapped)
            if (thisCont.body_name_1 > thisCont.body_name_2) {
                std::swap(thisCont.body_name_1, thisCont.body_name_2);
                std::swap(thisCont.body_type_1, thisCont.body_type_2);
            }
            if (otherCont.body_name_1 > otherCont.body_name_2) {
                std::swap(otherCont.body_name_1, otherCont.body_name_2);
                std::swap(otherCont.body_type_1, otherCont.body_type_2);                               

            }
            EXPECT_EQ (thisCont.body_name_1, otherCont.body_name_1 );
            EXPECT_EQ (thisCont.body_name_2, otherCont.body_name_2 );
            EXPECT_EQ (thisCont.body_type_1, otherCont.body_type_1 );
            EXPECT_EQ (thisCont.body_type_2, otherCont.body_type_2 );
            EXPECT_NEAR (thisCont.depth, otherCont.depth, 0.00001);
            EXPECT_NEAR (thisCont.pos.x(), otherCont.pos.x(), 0.00001);
            EXPECT_NEAR (thisCont.pos.y(), otherCont.pos.y(), 0.00001);
            EXPECT_NEAR (thisCont.pos.z(), otherCont.pos.z(), 0.00001);
            EXPECT_NEAR (thisCont.normal.x(), otherCont.normal.x(), 0.00001);
            EXPECT_NEAR (thisCont.normal.y(), otherCont.normal.y(), 0.00001);
            EXPECT_NEAR (thisCont.normal.z(), otherCont.normal.z(), 0.00001);
            
            // TODO HOW TO print this once and only if any of the expect at this loop iteration fails?
            if ( (std::abs(thisCont.depth - otherCont.depth)) > 0.00001) {
                std::cout << "EXPECT equal depths fails: error is on the actionState_" << i << std::endl;
                pinchCasted->print(); 
                pinchMap.at(keyPair).print();
                std::cout << std::endl;
            }
            
            i++;
        }
    }
}


////*********************************   LOOSE PINCH TESTS *********************************************************
TEST_F ( testFindPinches, checkNumberLinksLoose ) {
    
    for (auto &mapEl: pinchLooseMap ) {
        
        //being a pair the .first has always dimension 2
        EXPECT_EQ (2, mapEl.second.getFingersInvolved().size() ); //the names inside the action
        EXPECT_EQ (2, mapEl.second.getnFingersInvolved() ); //the int nLinkInvolved member of action
    }
    
    for (auto &mapEl: pinchLooseParsedMap ) {
        
        EXPECT_EQ (2, mapEl.first.size()); // the key
        EXPECT_EQ (2, mapEl.second->getFingersInvolved().size()); //the names inside the action
        EXPECT_EQ (2, mapEl.second->getnFingersInvolved()); //the int nLinkInvolved member of action
    }
}

TEST_F ( testFindPinches, checkSizeStatesInfoSetLoose ) {
    
    for (auto &mapEl: pinchLooseMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second.getMaxStoredActionStates(); 
        
        //it must be equal to the real size of the action state set
        EXPECT_EQ ( size, mapEl.second.getActionStates().size() );
        EXPECT_EQ ( size, mapEl.second.getAllJointPos().size() );
    }
    
    for (auto &mapEl: pinchLooseParsedMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second->getMaxStoredActionStates(); 
        
        EXPECT_EQ (size, mapEl.second->getAllJointPos().size());
    }
}

TEST_F ( testFindPinches, checkNameLoose ) {
    
    for (auto &mapEl: pinchLooseMap ) {
        
        EXPECT_TRUE (mapEl.second.getName().compare("pinchLoose") == 0);
        EXPECT_EQ (ROSEE::ActionPrimitive::Type::PinchLoose, mapEl.second.getPrimitiveType() );
    }
    
    for (auto &mapEl: pinchLooseParsedMap ) {
        
        EXPECT_TRUE (mapEl.second->getName().compare("pinchLoose") == 0);
        EXPECT_EQ (ROSEE::ActionPrimitive::Type::PinchLoose, mapEl.second->getPrimitiveType() );
    }
}

//this is an important test: check if the order of statesInfo in right according to distance
TEST_F ( testFindPinches, checkOrderStatesInfoSetLoose ) {
    
    for (auto &mapEl: pinchLooseMap ) { 
        
        std::vector < ROSEE::ActionPinchLoose::StateWithDistance> statesInfo = 
            mapEl.second.getActionStates();
        
        double oldDist = 0;
        for (auto &setEl : statesInfo) {
            EXPECT_GE ( setEl.second, oldDist ); //greater or equal
            oldDist = setEl.second;
        }
    }
    
    
    for (auto &mapEl: pinchLooseParsedMap ) { 
        
        std::shared_ptr <ROSEE::ActionPinchLoose> pinchLooseCasted = 
            std::dynamic_pointer_cast < ROSEE::ActionPinchLoose > (mapEl.second);
            
        ASSERT_FALSE (pinchLooseCasted == nullptr);
        std::vector < ROSEE::ActionPinchLoose::StateWithDistance> statesInfo = 
            pinchLooseCasted->getActionStates();
                
        double oldDist = 0;
        for (auto &setEl : statesInfo) {
            EXPECT_GE ( setEl.second, oldDist ); //greater or equal
            oldDist = setEl.second;
        }
    }
}

// to check if the found map is the same map that is emitted in the file and then parsed
TEST_F ( testFindPinches, checkEmitParseLoose ) {
    
    ASSERT_EQ (pinchLooseMap.size(), pinchLooseParsedMap.size() );
    
    for (auto &mapEl: pinchLooseParsedMap ) { 
        
        std::shared_ptr <ROSEE::ActionPinchLoose> pinchCasted = 
            std::dynamic_pointer_cast < ROSEE::ActionPinchLoose > (mapEl.second);
            
        ASSERT_FALSE (pinchCasted == nullptr);
        ASSERT_EQ (2, mapEl.first.size() ); // the key set must have dim 2 (the tip pair)
        std::pair <std::string, std::string> keyPair;
        std::set<std::string>::iterator it = mapEl.first.begin();
        keyPair.first = *it;
        std::advance ( it, 1 );
        keyPair.second = *it;
                
        //std::string is ok to compare with _EQ
        EXPECT_EQ (pinchCasted->getName(), pinchLooseMap.at(keyPair).getName() );
        EXPECT_EQ (pinchCasted->getnFingersInvolved(), pinchLooseMap.at(keyPair).getnFingersInvolved() );
        EXPECT_EQ (pinchCasted->getMaxStoredActionStates(), pinchLooseMap.at(keyPair).getMaxStoredActionStates());
        EXPECT_EQ (pinchCasted->getPrimitiveType(), pinchLooseMap.at(keyPair).getPrimitiveType() );
        EXPECT_EQ (pinchCasted->getFingersInvolved(), pinchLooseMap.at(keyPair).getFingersInvolved());
        
        unsigned int i = 0;
        for (auto as: pinchCasted->getActionStates() ) {

            //check equality of joint states (as.first)
            for (auto joint : as.first) {
                ASSERT_EQ ( joint.second.size(), 
                            pinchLooseMap.at(keyPair).getAllJointPos().at(i).at(joint.first).size() );
                //loop the eventually multiple joint pos (when dofs > 1)
                for (int j=0; j<joint.second.size(); ++j){
                    EXPECT_DOUBLE_EQ ( joint.second.at(j),
                        pinchLooseMap.at(keyPair).getAllJointPos().at(i).at(joint.first).at(j) ); 
                }
            }   
            
            //check equality of distance (as.second)
            EXPECT_DOUBLE_EQ (as.second, pinchLooseMap.at(keyPair).getActionStates().at(i).second);
            
            i++;
        }
    }
}


//A tight pinch cant be a loose pinch and viceversa
//here we check if all entries of one map are not present in the other (and viceversa)
//we check only the not parsed maps, other tests are done for correctness of parsing (checkEmitParse)
TEST_F ( testFindPinches, checkTightLooseExclusion ) {
    
    for (auto mapEl : pinchMap ) {
        EXPECT_EQ (0, pinchLooseMap.count(mapEl.first)) << mapEl.first.first << ", " << mapEl.first.second 
            << "  " << " is also present in Loose Pinches" << std::endl;
    }
    
    for (auto mapEl : pinchLooseMap ) {
        EXPECT_EQ (0, pinchMap.count(mapEl.first)) << mapEl.first.first << ", " << mapEl.first.second 
            << "  " << " is also present in Tight Pinches" << std::endl;
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
    
    if ( ROSEE::TestUtils::prepareROSForTests ( argc, argv, "testFindPinches" ) != 0 ) {
        
        std::cout << "[TEST ERROR] Prepare Funcion failed" << std::endl;
        return -1;
    }
    
    
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
