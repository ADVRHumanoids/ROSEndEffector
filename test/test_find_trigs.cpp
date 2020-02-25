#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/console.h>


#include <ROSEndEffector/FindActions.h>
#include <ROSEndEffector/ParserMoveIt.h>
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
    
        std::shared_ptr <ROSEE::ParserMoveIt> parserMoveIt = std::make_shared <ROSEE::ParserMoveIt> ();
        parserMoveIt->init ("robot_description") ;
        ROSEE::FindActions actionsFinder (parserMoveIt);
        
        trigMap.push_back( actionsFinder.findTrig(ROSEE::ActionPrimitive::Type::Trig, "/configs/actions/tests/") );
        trigMap.push_back( actionsFinder.findTrig(ROSEE::ActionPrimitive::Type::TipFlex, "/configs/actions/tests/") );
        trigMap.push_back( actionsFinder.findTrig(ROSEE::ActionPrimitive::Type::FingFlex, "/configs/actions/tests/") );

        ROSEE::YamlWorker yamlWorker(parserMoveIt->getHandName(), "/configs/actions/tests/");
        trigParsedMap.push_back( yamlWorker.parseYamlPrimitive("trig.yaml", ROSEE::ActionPrimitive::Type::Trig) );
        trigParsedMap.push_back( yamlWorker.parseYamlPrimitive("tipFlex.yaml", ROSEE::ActionPrimitive::Type::TipFlex) );
        trigParsedMap.push_back( yamlWorker.parseYamlPrimitive("fingFlex.yaml", ROSEE::ActionPrimitive::Type::FingFlex) );
    }

    virtual void TearDown() override {
    }

    std::vector < std::map < std::string , ROSEE::ActionTrig >  > trigMap;
    std::vector < std::map < std::set < std::string >, std::shared_ptr<ROSEE::ActionPrimitive> >  > trigParsedMap;
};


TEST_F ( testFindTrigs, checkNumberLinks ) {
    
    for (int k = 0; k< trigMap.size(); ++k) {
        for (auto &mapEl: trigMap.at(k) ) {
            
            //the .first has always dimension 1
            EXPECT_EQ (1, mapEl.second.getFingersInvolved().size() ); //the names inside the action
            EXPECT_EQ (1, mapEl.second.getnFingersInvolved() ); //the int nLinkInvolved member of action
        }

        for (auto &mapEl: trigParsedMap.at(k) ) {
            
            EXPECT_EQ (1, mapEl.first.size()); // the key
            EXPECT_EQ (1, mapEl.second->getFingersInvolved().size()); //the names inside the action
            EXPECT_EQ (1, mapEl.second->getnFingersInvolved()); //the int nLinkInvolved member of action
        }
    }
}

TEST_F ( testFindTrigs, checkSizeStatesInfoSet ) {
    
    for (int k = 0; k< trigMap.size(); ++k) {

        for (auto &mapEl: trigMap.at(k) ) {
            
            //get the member which is set in costructor
            unsigned int size = mapEl.second.getMaxStoredActionStates(); 
            
            //it must be equal to the real size of the statesInfoSet
            EXPECT_EQ ( size, mapEl.second.getAllJointPos().size() );
        }
        
        for (auto &mapEl: trigParsedMap.at(k) ) {
            
            //get the member which is set in costructor
            unsigned int size = mapEl.second->getMaxStoredActionStates(); 
            
            //it must be equal to the real size of the statesInfoSet
            EXPECT_EQ (size, mapEl.second->getAllJointPos().size());
        }
    }
}

TEST_F ( testFindTrigs, checkNameTypeConsistency ) {
    
    for (int k = 0; k < trigMap.size(); ++k) {
        
        ROSEE::ActionPrimitive::Type actionType = trigMap.at(k).begin()->second.getType(); 
    
        for (auto &mapEl: trigMap.at(k) ) {
            EXPECT_EQ (actionType, mapEl.second.getType() ); //in the map all el must be of same ActionType

            switch (mapEl.second.getType()) {
            case ROSEE::ActionPrimitive::Type::Trig : 
                EXPECT_EQ (mapEl.second.getName(), "trig");
                break;
            case ROSEE::ActionPrimitive::Type::TipFlex : 
                EXPECT_EQ (mapEl.second.getName(), "tipFlex");
                break;
            case ROSEE::ActionPrimitive::Type::FingFlex :
                EXPECT_EQ (mapEl.second.getName(), "fingFlex");
                break;
            default:
                FAIL() << mapEl.second.getType() << " not a know type" << std::endl ;
            }
        }
    
        actionType = trigParsedMap.at(k).begin()->second->getType(); 
        for (auto &mapEl: trigParsedMap.at(k) ) {
            EXPECT_EQ (actionType, mapEl.second->getType() ); //in the map all el must be of same ActionType


            switch (mapEl.second->getType()) {
            case ROSEE::ActionPrimitive::Type::Trig : 
                EXPECT_EQ (mapEl.second->getName(), "trig");
                break;
            case ROSEE::ActionPrimitive::Type::TipFlex :
                EXPECT_EQ (mapEl.second->getName(), "tipFlex");
                break;
            case ROSEE::ActionPrimitive::Type::FingFlex :
                EXPECT_EQ (mapEl.second->getName(), "fingFlex");
                break;
            default:
                FAIL() << mapEl.second->getType() << " not a know type" << std::endl ;
            }
        }
    }
}

// to check if the found map is the same map that is emitted in the file and then parsed
TEST_F ( testFindTrigs, checkEmitParse ) {
    
    for (int k = 0; k< trigMap.size(); ++k) {
    
        ASSERT_EQ (trigMap.at(k).size(), trigParsedMap.at(k).size() );
        
        for (auto &mapEl: trigParsedMap.at(k) ) { 
                        
            std::shared_ptr <ROSEE::ActionTrig> trigCasted = 
                std::dynamic_pointer_cast < ROSEE::ActionTrig > (mapEl.second);
                
            ASSERT_FALSE (trigCasted == nullptr);
            ASSERT_EQ (1, mapEl.first.size() );
            std::string key;
            std::set<std::string>::iterator it = mapEl.first.begin();
            key = *it;
                    
            //std::string is ok to compare with _EQ
            EXPECT_EQ (trigCasted->getName(), trigMap.at(k).at(key).getName() );
            EXPECT_EQ (trigCasted->getnFingersInvolved(), trigMap.at(k).at(key).getnFingersInvolved() );
            EXPECT_EQ (trigCasted->getMaxStoredActionStates(), trigMap.at(k).at(key).getMaxStoredActionStates());
            EXPECT_EQ (trigCasted->getType(), trigMap.at(k).at(key).getType() );
            EXPECT_EQ (trigCasted->getFingersInvolved(), trigMap.at(k).at(key).getFingersInvolved());
            EXPECT_EQ (trigCasted->getJointsInvolvedCount(), trigMap.at(k).at(key).getJointsInvolvedCount());

            for (auto jointStates: trigCasted->getAllJointPos() ) {
                
                //loop the map "jointStates"
                for (auto joint : jointStates) {
                    ASSERT_EQ ( joint.second.size(), trigMap.at(k).at(key).getJointPos().at(joint.first).size() );
                    //loop the eventually multiple joint pos (when dofs > 1)
                    for (int j=0; j<joint.second.size(); ++j){
                        EXPECT_DOUBLE_EQ ( joint.second.at(j),
                            trigMap.at(k).at(key).getJointPos().at(joint.first).at(j) ); 
                    }
                }       

            }
        }
    }
}

/** TipFlex and FingerFlex, for definition, must have the unique setted joints that are different joints
 * 
 * @NOTE : These consideration are valid because we send the joint in the biggest bound, so a 0 position always 
 * means that is a "not setted" joint. 
 * @WARNING what happens if the define in findAction.h DEFAULT_JOINT_POS 0.0 changes
 */
TEST_F ( testFindTrigs, checkJointPosTipAndFing ) {
    
    // we assume the order in trigmap : 0 = trig, 1 = tipflex, 2 = fingflex
    // otherwise we have to check which one is what that is useless
    ASSERT_EQ ( trigMap.at(0).begin()->second.getType(), ROSEE::ActionPrimitive::Type::Trig);
    ASSERT_EQ ( trigMap.at(1).begin()->second.getType(), ROSEE::ActionPrimitive::Type::TipFlex);
    ASSERT_EQ ( trigMap.at(2).begin()->second.getType(), ROSEE::ActionPrimitive::Type::FingFlex);
    
    //compare tip and fing flex
    for (auto &mapTipEl: trigMap.at(1) ) {
        
        ROSEE::JointPos tipJs = mapTipEl.second.getJointPos();
        
        for (auto &mapFingEl : trigMap.at(2) ) {
            
            ROSEE::JointPos fingJs = mapFingEl.second.getJointPos();
            ASSERT_EQ ( tipJs.size(), fingJs.size() );
            
            for (auto tipJoint: tipJs) {
                
                //at(0): 1dof joint
                if (tipJoint.second.at(0) != 0.0) {
                    //if so, it is the setted joint, and the correspondent of fingerAction must be zero
                    EXPECT_EQ ( fingJs.at(tipJoint.first).at(0), 0.0);
                } 
            }
        }
    }
}

/** If a tipFlex is present, the unique setted joint must be also setted in the trig action (taking the same
 * fingertip), because with trig we move ALL the joint on the finger. (so the opposite can be not true).
 * 
 * (third for loop) If some joint is not setted in the trig, it must be also non setted in 
 * the tipAction of the same fingertip
 * 
 * Similar consideration exists for fingFlex
 * 
 * @NOTE : These consideration are valid because we send the joint in the biggest bound, so a 0 position always 
 * means that is a "not setted" joint. 
 * @WARNING what happens if the define in findAction.h DEFAULT_JOINT_POS 0.0 changes
 * @WARNING the third for must be changed if for trig another key (like the finger group name) is used instead
 * of the tip
 */
TEST_F ( testFindTrigs, checkJointPosFlexsAndTrig ) {
    
    // we assume the order in trigmap : 0 = trig, 1 = tipflex, 2 = fingflex
    // otherwise we have to check which one is what that is useless
    ASSERT_EQ ( trigMap.at(0).begin()->second.getType(), ROSEE::ActionPrimitive::Type::Trig);
    ASSERT_EQ ( trigMap.at(1).begin()->second.getType(), ROSEE::ActionPrimitive::Type::TipFlex);
    ASSERT_EQ ( trigMap.at(2).begin()->second.getType(), ROSEE::ActionPrimitive::Type::FingFlex);
    
    
   // If a tipFlex is present, the unique setted joint must be also setted (equal pos) in the trig action 
    for (auto &mapTipEl: trigMap.at(1) ) {                
        for ( auto &tipJs : mapTipEl.second.getJointPos() ) {
            if (tipJs.second.at(0) != 0 ) {
                //if a tipFlex exist for a tip, also a trig for that tip exist
                EXPECT_TRUE (trigMap.at(0).find ( mapTipEl.first ) != trigMap.at(0).end());
                EXPECT_DOUBLE_EQ ( tipJs.second.at(0),
                        trigMap.at(0).at(mapTipEl.first).getJointPos().at(tipJs.first).at(0) );
            }
            
        }
    }
    
    // If a FingFlex is present, the unique setted joint must be also setted (equal pos) in the trig action 
    for (auto &mapFingEl: trigMap.at(2) ) {                
        for ( auto &fingJs : mapFingEl.second.getJointPos() ) {
            if (fingJs.second.at(0) != 0 ) {
                //if a fingFlex exist for a tip, also a trig for that tip exist
                EXPECT_TRUE ( trigMap.at(0).find ( mapFingEl.first ) != trigMap.at(0).end() );
                EXPECT_DOUBLE_EQ ( fingJs.second.at(0),
                        trigMap.at(0).at(mapFingEl.first).getJointPos().at(fingJs.first).at(0) );
            }
        }
    }
    
    // If some joint is not setted in the trig, it must be also non setted in 
    // the tipAction of the same fingertip
    for (auto &mapTrigEl: trigMap.at(0) ) {    
        for ( auto &trigJs : mapTrigEl.second.getJointPos() ) {
            if (trigJs.second.at(0) == 0.0 ) {
                
                // if a trig exist, it is not assured that a tip flex exist for that tip
                if (trigMap.at(1).find ( mapTrigEl.first ) != trigMap.at(1).end()) {
                    EXPECT_EQ ( 0.0,
                            trigMap.at(1).at(mapTrigEl.first).getJointPos().at(trigJs.first).at(0) );
                }
                
                // if a trig exist, it is not assured that a fing flex exist for that tip
                if (trigMap.at(2).find ( mapTrigEl.first ) != trigMap.at(2).end()) {
                    EXPECT_EQ ( 0.0,
                            trigMap.at(2).at(mapTrigEl.first).getJointPos().at(trigJs.first).at(0) );
                }
            }
        }
    }
}


/** For the tip and fing flex action, for definition, only one joint must be set (ie position != 0).
 */
TEST_F ( testFindTrigs, checkFlexsSingleJoint ) {
    
    for (int k = 0; k< trigMap.size(); ++k) {
        
        if ( trigMap.at(k).begin()->second.getType() == ROSEE::ActionPrimitive::Type::Trig ) {
            continue;
        }
        
        for (auto &mapFlexEl: trigMap.at(k) ) {    

            unsigned int nJSet = 0;  
          
            for ( auto &flexJs : mapFlexEl.second.getJointPos() ) { //iteration over the jointstates map
                
                if ( flexJs.second.at(0) != 0.0 ) {
                    nJSet++;
                }                
            }
            EXPECT_EQ (1, nJSet);
            
            //test also the jointInvolvedBool vector
            unsigned int jointsInvolvedSum = 0;
            for ( auto flexJs : mapFlexEl.second.getJointsInvolvedCount() ) { //iteration over the jointstates map
                
                jointsInvolvedSum += flexJs.second;               
            }
            EXPECT_EQ (1, jointsInvolvedSum);   
            
        }
    }
}


} //namespace

int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}