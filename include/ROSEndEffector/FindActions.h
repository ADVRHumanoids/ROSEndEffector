#ifndef __ROSEE_FIND_ACTIONS_
#define __ROSEE_FIND_ACTIONS_

#include <moveit/planning_scene/planning_scene.h>

#include <ROSEndEffector/YamlWorker.h>
#include <ROSEndEffector/ParserMoveIt.h>
#include <ROSEndEffector/Action.h>
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
 * @warning there is a problem with collisions: with the schunk hand, if we move only the middle (base phalange)
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
    std::map <std::string, ROSEE::ActionTrig> findTrig (  ROSEE::ActionPrimitive::Type actionType,
        std::string path2saveYaml = "" );
    
private:
    
    std::shared_ptr < ROSEE::ParserMoveIt > parserMoveIt;
    
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >  checkCollisions();
    void checkDistances (std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* );

    /**
     * @brief Given the contact, we want to know the state of the joint to replicate it. But we want to know
     * only the state of the joints that effectively act on the contact, that are the ones which moves one of the two tips (or both). So the other joints are put to the DEFAULT_JOINT_POS value
     * @return JointsInvolvedCount, the map where each element is relative at one joint (joint name is the key).
     * The value is the number of times that joint is used, for primitive actions can be only 0 or 1
     */
    ROSEE::JointsInvolvedCount setOnlyDependentJoints(std::pair < std::string, std::string > tipsNames, JointPos *Jstates);
    
   /** 
    * @brief trig is the action of closing a SINGLE finger towards the palm.
    * The position is the bound which is farther from 0 (considered as default pos). All hands have more range of motion
    * in the flexion respect to extension (as human finger). NOT valid for other motion, like finger spread or
    * thumb addition/abduction.
    * @note If a joint is continuos, it is excluded from the trig action. (because I cant think about a continuos 
    * joint that is useful for a trig action, but can be present in theory)
    * @return std::map <std::string, ROSEE::ActionTrig> the map witch key the tip/finger and as value its ActionTrig
    */
    std::map <std::string, ActionTrig> trig();
    
    /**
    * @brief We start from each tip. Given a tip, we look for all the joints that move this tip. If it has 2 
    * or more joints that move exclusively that tip ( we count this number with \ref ParserMoveIt::getNExclusiveJointsOfTip ), 
    * we say that a tipFlex is possible. If not, we can't move the tip independently from the rest of the 
    * finger, so we have a trig action (if \ref ParserMoveIt::getNExclusiveJointsOfTip returns 1 ) or 
    * nothing (\ref ParserMoveIt::getNExclusiveJointsOfTip returns 0).
    * If \ref ParserMoveIt::getNExclusiveJointsOfTip return >= 2, starting from the tip, we explore the parents joints, 
    * until we found the first actuated joint. This one will be \ref theInterestingJoint which pose we must 
    * set. All the other joints (actuated) will have the default position (if no strange errors).
    * @return std::map <std::string, ROSEE::ActionTrig> the map witch key the tip/finger and as value its ActionTipFlex
    */
    std::map <std::string, ROSEE::ActionTrig> tipFlex();
    
    /** 
    * @brief We start from each tip. Given a tip, we check if \ref ParserMoveIt::getNExclusiveJointsOfTip >= 2 
    *  (see \ref tipFlex function).
    *  If so, we continue exploring the chain from the tip going up through the parents. We stop when a parent has
    *  more than 1 joint as child. This means that the last link is the first of the finger. Meanwhile we have 
    *  stored the actuated, not continuos joint (in \ref joint) that we were founding along the chain. The last stored
    *  is exaclty \ref theInterestingJoint, which pose of we must set.
    *  All the other joints (actuated) will have the default position (if no strange errors).
    */
    std::map <std::string, ROSEE::ActionTrig> fingFlex();

    
    //other utilities used by this class
    bool insertJointPosForTrigInMap ( std::map <std::string, ActionTrig>& trigMap, 
        ROSEE::ActionTrig action, std::string jointName, double trigValue);  

    void fillNotCollidingTips ( std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >*, 
             const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >* );
    JointPos getConvertedJointPos(const robot_state::RobotState* kinematic_state);
    void removeBoundsOfNotCollidingTips ( const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >*, robot_model::RobotModelPtr );
    void checkWhichTipsCollideWithoutBounds (
        std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >*);

};
    
}


#endif //__ROSEE_FIND_ACTIONS_
