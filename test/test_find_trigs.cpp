#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/console.h>


#include <ROSEndEffector/FindActions.h>
#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionTrig.h>

namespace {

class testFindTrigs: public ::testing::Test {


protected:

    testFindTrigs() {
    }

    virtual ~testFindTrigs() {
    }

    virtual void SetUp() override {

        const char *argv[] = {"testFindTrigs", "arg"};
        int argc = sizeof(argv) / sizeof(char*) - 1;
        
        //is this cast correct?
        ros::init ( argc, (char**)argv, "testFindTrigs" );
        
        //add here if want to try other hands. HOW to run all togheter?
        std::string hand4Test = "svh-standalone"; // "svh-standalone", "two_finger", "test_ee"
        
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
        
        trigMap = actionsFinder.findTrig("/configs/actions/tests/");

        //TODO getHandName should be in the parser
        ROSEE::YamlWorker yamlWorker(actionsFinder.getHandName(), "/configs/actions/tests/");
        trigParsedMap = yamlWorker.parseYaml("trig.yaml", ROSEE::ActionType::Trig);
    }

    virtual void TearDown() {
    }

    std::map < std::string , ROSEE::ActionTrig > trigMap;
    std::map < std::set < std::string >, std::shared_ptr<ROSEE::ActionPrimitive> > trigParsedMap;
};


TEST_F ( testFindTrigs, checkNumberLinks ) {
    
    for (auto &mapEl: trigMap ) {
        
        //the .first has always dimension 1
        EXPECT_EQ (1, mapEl.second.getLinksInvolved().size() ); //the names inside the action
        EXPECT_EQ (1, mapEl.second.getnLinksInvolved() ); //the int nLinkInvolved member of action
    }
    
    for (auto &mapEl: trigParsedMap ) {
        
        EXPECT_EQ (1, mapEl.first.size()); // the key
        EXPECT_EQ (1, mapEl.second->getLinksInvolved().size()); //the names inside the action
        EXPECT_EQ (1, mapEl.second->getnLinksInvolved()); //the int nLinkInvolved member of action
    }
}

TEST_F ( testFindTrigs, checkSizeStatesInfoSet ) {
    
    for (auto &mapEl: trigMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second.getJointStatesSetMaxSize(); 
        
        //it must be equal to the real size of the statesInfoSet
        EXPECT_EQ ( size, mapEl.second.getActionStates().size() );
    }
    
    for (auto &mapEl: trigParsedMap ) {
        
        //get the member which is set in costructor
        unsigned int size = mapEl.second->getJointStatesSetMaxSize(); 
        
        //it must be equal to the real size of the statesInfoSet
        EXPECT_EQ (size, mapEl.second->getActionStates().size());
    }
}

TEST_F ( testFindTrigs, checkName ) {
    
    for (auto &mapEl: trigMap ) {
        EXPECT_TRUE (mapEl.second.getName().compare("trig") == 0);
        EXPECT_EQ (ROSEE::ActionType::Trig, mapEl.second.getActionType() );
    }
    
    for (auto &mapEl: trigParsedMap ) {
        EXPECT_TRUE (mapEl.second->getName().compare("trig") == 0);
        EXPECT_EQ (ROSEE::ActionType::Trig, mapEl.second->getActionType() );
    }
}

// to check if the found map is the same map that is emitted in the file and then parsed
TEST_F ( testFindTrigs, checkEmitParse ) {
    
    ASSERT_EQ (trigMap.size(), trigParsedMap.size() );
    
    for (auto &mapEl: trigParsedMap ) { 
        
        std::shared_ptr <ROSEE::ActionTrig> trigCasted = 
            std::dynamic_pointer_cast < ROSEE::ActionTrig > (mapEl.second);
            
        ASSERT_FALSE (trigCasted == nullptr);
        ASSERT_EQ (1, mapEl.first.size() );
        std::string key;
        std::set<std::string>::iterator it = mapEl.first.begin();
        key = *it;
                
        //std::string is ok to compare with _EQ
        EXPECT_EQ (trigCasted->getName(), trigMap.at(key).getName() );
        EXPECT_EQ (trigCasted->getnLinksInvolved(), trigMap.at(key).getnLinksInvolved() );
        EXPECT_EQ (trigCasted->getJointStatesSetMaxSize(), trigMap.at(key).getJointStatesSetMaxSize());
        EXPECT_EQ (trigCasted->getActionType(), trigMap.at(key).getActionType() );
        EXPECT_EQ (trigCasted->getLinksInvolved(), trigMap.at(key).getLinksInvolved());

        unsigned int i = 0;
        for (auto jointStates: trigCasted->getActionStates() ) {
            
            //loop the map "jointStates"
            for (auto joint : jointStates) {
                ASSERT_EQ ( joint.second.size(),  trigMap.at(key).getActionState().at(joint.first).size() );
                //loop the eventually multiple joint pos (when dofs > 1)
                for (int j=0; j<joint.second.size(); ++j){
                    EXPECT_DOUBLE_EQ ( joint.second.at(j),
                        trigMap.at(key).getActionState().at(joint.first).at(j) ); 
                }
            }
           
            i++;
        }
    }
}



} //namespace

int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
