#ifndef __ROSEE_FIND_ACTIONS_
#define __ROSEE_FIND_ACTIONS_

#include <moveit/planning_scene/planning_scene.h>

#include <ROSEndEffector/YamlWorker.h>
#include <ROSEndEffector/ParserMoveIt.h>
#include <ROSEndEffector/ActionPinchStrong.h>
#include <ROSEndEffector/ActionPinchWeak.h>
#include <ROSEndEffector/ActionTrig.h>


#define N_EXP_COLLISION 5000 //5000 is ok
#define N_EXP_DISTANCES 5000 //? is ok
#define DEFAULT_JOINT_POS 0.0

namespace ROSEE
{
    
/**
 * @brief Class to check which fingertips collide (for the pinch action at the moment)
 * 
 * @warning there is a problem with collisions: with the schunk hand, if we only the middle (base phalange)
 * toward the hand, a collision between index tip, middle tip and ring tip is detected. Easy reproducible with the 
 * moveit assistant, in the set pose section (it find a collision when visually is not present, when we move the 
 * middle). There are some caotic printing in bugmoveit branch, to replicate the problem also with this code.
 * I dont know if it is a problem of schunk model, moveit, or both.
 * 
 */
class FindActions
{
public:
    FindActions ( std::shared_ptr < ROSEE::ParserMoveIt > parserMoveit ) ;
    std::pair <  std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >, 
                 std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >  > 
                 findPinch ( std::string path2saveYaml = "" );
    std::map <std::string, ROSEE::ActionTrig> findTrig (  ROSEE::ActionType actionType,
        std::string path2saveYaml = "" );
    
private:
    
    std::shared_ptr < ROSEE::ParserMoveIt > parserMoveIt;
    
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >  checkCollisions();
    void checkDistances (std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* );

    /**
     * @brief Given the contact, we want to know the state of the joint to replicate it. But we want to know
     * only the state of the joints that effectively act on the contact, that are the ones which moves one of the two tips (or both). So the other joints are put to the DEFAULT_JOINT_POS value
     * @return a vector of bool, where each element is relative at one joint (joint order is assured being
     * JointStates a map). True means that the joint is used in the action, otherwise there is false
     */
    std::vector<bool> setOnlyDependentJoints(std::pair < std::string, std::string > tipsNames, JointStates *Jstates);
    
    //trig etc
    std::map <std::string, ActionTrig> trig();
    std::map <std::string, ROSEE::ActionTrig> tipFlex();
    std::map <std::string, ROSEE::ActionTrig> fingFlex();

    
    //other utilities used by this class
    bool insertJointPosForTrigInMap ( std::map <std::string, ActionTrig>& trigMap, 
        ROSEE::ActionTrig action, std::string jointName, double trigValue);  

    void fillNotCollidingTips ( std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >*, 
             const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >* );
    JointStates getConvertedJointStates(const robot_state::RobotState* kinematic_state);
    void removeBoundsOfNotCollidingTips ( const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >*, robot_model::RobotModelPtr );
    void checkWhichTipsCollideWithoutBounds (
        std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >*);

};
    
}


#endif //__ROSEE_FIND_ACTIONS_
