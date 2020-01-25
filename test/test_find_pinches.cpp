#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/console.h>


#include <ROSEndEffector/FindActions.h>
#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionPinch.h>

namespace {

class testFindPinches: public ::testing::Test {


protected:

    testFindPinches() {
    }

    virtual ~testFindPinches() {
    }

    virtual void SetUp() override {

        const char *argv[] = {"testFindPinches", "arg"};
        int argc = sizeof(argv) / sizeof(char*) - 1;
        
        //is this cast correct?
        ros::init ( argc, (char**)argv, "testFindPinches" );
    
        ROSEE::FindActions actionsFinder ("robot_description");
        
        auto theTwoMaps = actionsFinder.findPinch("/configs/actions/tests/");
        pinchMap = theTwoMaps.first;
        pinchWeakMap = theTwoMaps.second;

        //TODO getHandName should be in the parser
        ROSEE::YamlWorker yamlWorker(actionsFinder.getHandName(), "/configs/actions/tests/");
        pinchParsedMap = yamlWorker.parseYaml("pinch.yaml", ROSEE::ActionType::Pinch);
        
        pinchWeakParsedMap = yamlWorker.parseYaml("pinchWeak.yaml", ROSEE::ActionType::PinchWeak);
    }

    virtual void TearDown() {
    }

    std::map < std::pair < std::string, std::string >, ROSEE::ActionPinch > pinchMap;
    std::map < std::set < std::string >, std::shared_ptr<ROSEE::ActionPrimitive> > pinchParsedMap;
    
    std::map < std::pair < std::string, std::string >, ROSEE::ActionPinchWeak > pinchWeakMap;
    std::map < std::set < std::string >, std::shared_ptr<ROSEE::ActionPrimitive> > pinchWeakParsedMap;
};


TEST_F ( testFindPinches, checkNumberLinks ) {
    
    for (auto &mapEl: pinchMap ) {
        
        //being a pair the .first has always dimension 2
        EXPECT_EQ (2, mapEl.second.getLinksInvolved().size() ); //the names inside the action
        EXPECT_EQ (2, mapEl.second.getnLinksInvolved() ); //the int nLinkInvolved member of action
    }
    
    for (auto &mapEl: pinchParsedMap ) {
        
        EXPECT_EQ (2, mapEl.first.size()); // the key
        EXPECT_EQ (2, mapEl.second->getLinksInvolved().size()); //the names inside the action
        EXPECT_EQ (2, mapEl.second->getnLinksInvolved()); //the int nLinkInvolved member of action
    }
}

TEST_F ( testFindPinches, checkSizeStatesInfoSet ) {
    
    for (auto &mapEl: pinchMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second.getJointStatesSetMaxSize(); 
        
        //it must be equal to the real size of the statesInfoSet
        EXPECT_EQ ( size, mapEl.second.getActionStates().size() );
    }
    
    for (auto &mapEl: pinchParsedMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second->getJointStatesSetMaxSize(); 
        
        //it must be equal to the real size of the statesInfoSet
        EXPECT_EQ (size, mapEl.second->getActionStates().size());
    }
}

TEST_F ( testFindPinches, checkName ) {
    
    for (auto &mapEl: pinchMap ) {
        
        EXPECT_TRUE (mapEl.second.getName().compare("pinch") == 0);
        EXPECT_EQ (ROSEE::ActionType::Pinch, mapEl.second.getActionType() );
    }
    
    for (auto &mapEl: pinchParsedMap ) {
        
        EXPECT_TRUE (mapEl.second->getName().compare("pinch") == 0);
        EXPECT_EQ (ROSEE::ActionType::Pinch, mapEl.second->getActionType() );
    }
}

//this is an important test: check if the order of statesInfo in right according to depth
TEST_F ( testFindPinches, checkOrderStatesInfoSet ) {
    
    for (auto &mapEl: pinchMap ) { 
        
        std::vector < ROSEE::ActionPinch::StateWithContact> statesInfo = 
            mapEl.second.getActionStatesWithContact();
        
        double oldDepth = std::numeric_limits<double>::infinity();
        
        for (auto &setEl : statesInfo) {
            EXPECT_LE (std::abs(setEl.second.depth), std::abs(oldDepth) ); //lesser or equal
            oldDepth = setEl.second.depth;
        }
    }
    
    
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

// to check if the found map is the same map that is emitted in the file and then parsed
TEST_F ( testFindPinches, checkEmitParse ) {
    
    ASSERT_EQ (pinchMap.size(), pinchParsedMap.size() );
    
    for (auto &mapEl: pinchParsedMap ) { 
        
        std::shared_ptr <ROSEE::ActionPinch> pinchCasted = 
            std::dynamic_pointer_cast < ROSEE::ActionPinch > (mapEl.second);
            
        ASSERT_FALSE (pinchCasted == nullptr);
        ASSERT_EQ (2, mapEl.first.size() );
        std::pair <std::string, std::string> keyPair;
        std::set<std::string>::iterator it = mapEl.first.begin();
        keyPair.first = *it;
        std::advance ( it, 1 );
        keyPair.second = *it;
                
        //std::string is ok to compare with _EQ
        EXPECT_EQ (pinchCasted->getName(), pinchMap.at(keyPair).getName() );
        EXPECT_EQ (pinchCasted->getnLinksInvolved(), pinchMap.at(keyPair).getnLinksInvolved() );
        EXPECT_EQ (pinchCasted->getJointStatesSetMaxSize(), pinchMap.at(keyPair).getJointStatesSetMaxSize());
        EXPECT_EQ (pinchCasted->getActionType(), pinchMap.at(keyPair).getActionType() );
        EXPECT_EQ (pinchCasted->getLinksInvolved(), pinchMap.at(keyPair).getLinksInvolved());

        unsigned int i = 0;
        for (auto as: pinchCasted->getActionStatesWithContact() ) {
            
            //check equality of joint states (as.first)
            for (auto joint : as.first) {
                ASSERT_EQ ( joint.second.size(), 
                            pinchMap.at(keyPair).getActionStates().at(i).at(joint.first).size() );
                //loop the eventually multiple joint pos (when dofs > 1)
                for (int j=0; j<joint.second.size(); ++j){
                    EXPECT_DOUBLE_EQ ( joint.second.at(j),
                        pinchMap.at(keyPair).getActionStates().at(i).at(joint.first).at(j) ); 
                }
            }   
            
            //check equality of contact (as.second)
            collision_detection::Contact thisCont = as.second;
            collision_detection::Contact otherCont =
                pinchMap.at(keyPair).getActionStatesWithContact().at(i).second;
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
                pinchCasted->printAction(); 
                pinchMap.at(keyPair).printAction();
                std::cout << std::endl;
            }
            
            i++;
        }
    }
}


////*********************************   WEAK PINCH TESTSSS *********************************************************
TEST_F ( testFindPinches, checkNumberLinksWeak ) {
    
    for (auto &mapEl: pinchWeakMap ) {
        
        //being a pair the .first has always dimension 2
        EXPECT_EQ (2, mapEl.second.getLinksInvolved().size() ); //the names inside the action
        EXPECT_EQ (2, mapEl.second.getnLinksInvolved() ); //the int nLinkInvolved member of action
    }
    
    for (auto &mapEl: pinchWeakParsedMap ) {
        
        EXPECT_EQ (2, mapEl.first.size()); // the key
        EXPECT_EQ (2, mapEl.second->getLinksInvolved().size()); //the names inside the action
        EXPECT_EQ (2, mapEl.second->getnLinksInvolved()); //the int nLinkInvolved member of action
    }
}

TEST_F ( testFindPinches, checkSizeStatesInfoSetWeak ) {
    
    for (auto &mapEl: pinchWeakMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second.getJointStatesSetMaxSize(); 
        
        //it must be equal to the real size of the statesInfoSet
        EXPECT_EQ ( size, mapEl.second.getActionStates().size() );
    }
    
    for (auto &mapEl: pinchWeakParsedMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second->getJointStatesSetMaxSize(); 
        
        //it must be equal to the real size of the statesInfoSet
        EXPECT_EQ (size, mapEl.second->getActionStates().size());
    }
}

TEST_F ( testFindPinches, checkNameWeak ) {
    
    for (auto &mapEl: pinchWeakMap ) {
        
        EXPECT_TRUE (mapEl.second.getName().compare("pinchWeak") == 0);
        EXPECT_EQ (ROSEE::ActionType::Pinch, mapEl.second.getActionType() );
    }
    
    for (auto &mapEl: pinchWeakParsedMap ) {
        
        EXPECT_TRUE (mapEl.second->getName().compare("pinchWeak") == 0);
        EXPECT_EQ (ROSEE::ActionType::Pinch, mapEl.second->getActionType() );
    }
}

//this is an important test: check if the order of statesInfo in right according to distance
TEST_F ( testFindPinches, checkOrderStatesInfoSetWeak ) {
    
    for (auto &mapEl: pinchWeakMap ) { 
        
        std::vector < ROSEE::ActionPinchWeak::StateWithDistance> statesInfo = 
            mapEl.second.getActionStatesWithDistance();
        
        double oldDist = 0;
        for (auto &setEl : statesInfo) {
            EXPECT_GE ( setEl.second, oldDist ); //greater or equal
            oldDist = setEl.second;
        }
    }
    
    
    for (auto &mapEl: pinchWeakParsedMap ) { 
        
        std::shared_ptr <ROSEE::ActionPinchWeak> pinchWeakCasted = 
            std::dynamic_pointer_cast < ROSEE::ActionPinchWeak > (mapEl.second);
            
        ASSERT_FALSE (pinchWeakCasted == nullptr);
        std::vector < ROSEE::ActionPinchWeak::StateWithDistance> statesInfo = 
            pinchWeakCasted->getActionStatesWithDistance();
                
        double oldDist = 0;
        for (auto &setEl : statesInfo) {
            EXPECT_GE ( setEl.second, oldDist ); //greater or equal
            oldDist = setEl.second;
        }
    }
}

// to check if the found map is the same map that is emitted in the file and then parsed
TEST_F ( testFindPinches, checkEmitParseWeak ) {
    
    ASSERT_EQ (pinchWeakMap.size(), pinchWeakParsedMap.size() );
    
    for (auto &mapEl: pinchWeakParsedMap ) { 
        
        std::shared_ptr <ROSEE::ActionPinchWeak> pinchCasted = 
            std::dynamic_pointer_cast < ROSEE::ActionPinchWeak > (mapEl.second);
            
        ASSERT_FALSE (pinchCasted == nullptr);
        ASSERT_EQ (2, mapEl.first.size() ); // the key set must have dim 2 (the tip pair)
        std::pair <std::string, std::string> keyPair;
        std::set<std::string>::iterator it = mapEl.first.begin();
        keyPair.first = *it;
        std::advance ( it, 1 );
        keyPair.second = *it;
                
        //std::string is ok to compare with _EQ
        EXPECT_EQ (pinchCasted->getName(), pinchWeakMap.at(keyPair).getName() );
        EXPECT_EQ (pinchCasted->getnLinksInvolved(), pinchWeakMap.at(keyPair).getnLinksInvolved() );
        EXPECT_EQ (pinchCasted->getJointStatesSetMaxSize(), pinchWeakMap.at(keyPair).getJointStatesSetMaxSize());
        EXPECT_EQ (pinchCasted->getActionType(), pinchWeakMap.at(keyPair).getActionType() );
        EXPECT_EQ (pinchCasted->getLinksInvolved(), pinchWeakMap.at(keyPair).getLinksInvolved());
        
        unsigned int i = 0;
        for (auto as: pinchCasted->getActionStatesWithDistance() ) {

            //check equality of joint states (as.first)
            for (auto joint : as.first) {
                ASSERT_EQ ( joint.second.size(), 
                            pinchWeakMap.at(keyPair).getActionStates().at(i).at(joint.first).size() );
                //loop the eventually multiple joint pos (when dofs > 1)
                for (int j=0; j<joint.second.size(); ++j){
                    EXPECT_DOUBLE_EQ ( joint.second.at(j),
                        pinchWeakMap.at(keyPair).getActionStates().at(i).at(joint.first).at(j) ); 
                }
            }   
            
            //check equality of distance (as.second)
            EXPECT_DOUBLE_EQ (as.second, pinchWeakMap.at(keyPair).getActionStatesWithDistance().at(i).second);
            
            i++;
        }
    }
}


//A strong pinch cant be a weak pinch and viceversa
//here we check if all entries of one map are not present in the other (and viceversa)
//we check only the not parsed maps, other tests are done for correctness of parsing (checkEmitParse)
TEST_F ( testFindPinches, checkStrongWeakExclusion ) {
    
    for (auto mapEl : pinchMap ) {
        EXPECT_EQ (0, pinchWeakMap.count(mapEl.first)) << mapEl.first.first << ", " << mapEl.first.second 
            << "  " << " is also present in Weak Pinches" << std::endl;
    }
    
    for (auto mapEl : pinchWeakMap ) {
        EXPECT_EQ (0, pinchMap.count(mapEl.first)) << mapEl.first.first << ", " << mapEl.first.second 
            << "  " << " is also present in Strong Pinches" << std::endl;
    }
    
}



} //namespace

int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
