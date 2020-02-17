#ifndef __ROSEE_FIND_ACTIONS_
#define __ROSEE_FIND_ACTIONS_

#include <moveit/planning_scene/planning_scene.h>

#include <ROSEndEffector/YamlWorker.h>
#include <ROSEndEffector/ParserMoveIt.h>
#include <ROSEndEffector/Action.h>
#include <ROSEndEffector/ActionPinchStrong.h>
#include <ROSEndEffector/ActionPinchWeak.h>
#include <ROSEndEffector/ActionTrig.h>
#include <ROSEndEffector/ActionMoreTips.h>


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
    
    /**
     * @brief Function to look for pinches, both Strong and Weak ones. It fill the maps (returned as pair), but also 
     * print the infos in two yaml files (one for strong, one for weak) using a \ref YamlWorker
     * @param path2saveYaml the path where to create/overwrite the yaml files. 
     * @return a pair of maps. The first is the map of \ref ActionPinchStrong, the second the map of \ref ActionPinchWeak
     */
    std::pair <  std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >, 
                 std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >  > 
                 findPinch ( std::string path2saveYaml = "" );
                 
    /**
     * @brief Function to look for trigs (trig, tipFlex and fingFlex). The type of trig to be looked for is choosen thanks
     * to the argument \p actionType. This function return the map of wanted trig but also print info about that on a yaml file 
     * using a \ref YamlWorker
     * 
     * @param actionType the type of trig to look for (Trig, TipFlex, FingFlex)
     * @param path2saveYaml the path where to create/overwrite the yaml file.
     * @return the map of chosen type of trig filled with infos about the possible actions.
     */
    std::map <std::string, ROSEE::ActionTrig> findTrig (  ROSEE::ActionPrimitive::Type actionType,
        std::string path2saveYaml = "" );
    
    std::map < std::set<std::string>, ROSEE::ActionMoreTips> findMoreTips ( unsigned int nFinger, std::string path2saveYaml = "");
    
private:
    
    std::shared_ptr < ROSEE::ParserMoveIt > parserMoveIt;
    
    /**
     * @brief principal function which check for collisions with moveit functions when looking for strong pinches
     * @return the map of ActionPinchStrong
     * 
     */
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >  checkCollisions();
    
    /**
     * @brief Principal function which fill the \p mapOfWeakPinches basing on minumun distance between tips
     * @param mapOfWeakPinches [out] pointer to the mapOfWeakPinches to be filled
     */
    void checkDistances (std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* mapOfWeakPinches);
    
    /**
     * @brief Function similar to \ref checkCollisions but used for Weak Pinches.
     * First, we temporarily remove bounds of joints linked to the non-colliding tips (with \ref removeBoundsOfNotCollidingTips),
     * and we check for collision with this function.
     * If some collision are found, this means that tips movements make them go towards each other, (also with the bounds) but
     * the joint limits do not permit them to touch. This is a weak pinch. 
     * If they do not collide even without bounds, this means that they never go towards each other, so this is not a strong neither weak
     * pinch. 
     * We also create new kinematic_model object so we dont modify the original kinematic_model, and we can change the joint limits 
     * of the new copy safely
     * 
     * @param mapOfWeakPinches [out] map of weak pinches that will be filled with info about these particular actions
     */
    void checkWhichTipsCollideWithoutBounds (
        std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* mapOfWeakPinches);

    /**
     * @brief Support function to remove the joint limits from the model. This is done when looking for Weak Pinches.
     * @param mapOfWeakPinches [in] pointer to the map of weak pinches
     * @param RobotModelPtr the pointer to the robot model
     */
    void removeBoundsOfNotCollidingTips ( const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* mapOfWeakPinches, 
                                          robot_model::RobotModelPtr );
    
    /**
     * @brief function to "initialize" the map of ActionPinchWeak \p mapOfWeakPinches. It is done adding all the tips pairs and then removing 
     * the pairs that are present in the map of ActionPinchStrong \p mapOfPinches. Note that the values of the map, the \ref ActionPinchWeak
     * are action with only tips name (so no info right now).
     * 
     * @param mapOfWeakPinches [out] Pointer to the map of \ref ActionPinchWeak to be initialized
     * @param mapOfPinches [in] pointer to the map of \ref ActionPinchStrong, already filled before, that is used to erase the get the tips that collide
     *      and to remove them from the \p mapOfWeakPinches
     */
    void fillNotCollidingTips ( std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* mapOfWeakPinches,
        const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >* mapOfPinches );
    
    
    
    
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

    
    /**
     * @brief Insert/update an ActionTrig in the \p trigMap. This is done setting the \p jointName position to the given
     * \p jointName. So, for a single action this function can be executed more than once (because more joint can be set).
     * The Action \p action can be already present in the map; in this case it is updated setting the \p jointName position to the given \p jointName.
     * If the Action \p action was not present before, it is inserted in the \p trigMap.
     * 
     * @param trigMap [out] The map of ActionTrig to be updated
     * @param action The action involved in the updating
     * @param jointName the name of the joint of the action that must be set
     * @param trigValue the value of the position of the joint
     * @return TRUE if \p action was not present before in the map and it is inserted now;
     *         FALSE if the action was already present and only the \p jointName value is updated to \p trigValue
     */
    bool insertJointPosForTrigInMap ( std::map <std::string, ActionTrig>& trigMap, 
        ROSEE::ActionTrig action, std::string jointName, double trigValue);  


    
    /**
     * @brief Given the contact, we want to know the state of the joint to replicate it. But we want to know
     * only the state of the joints that effectively act on the contact, that are the ones which moves one of the two tips (or both). So the other joints are put to the DEFAULT_JOINT_POS value
     * @return JointsInvolvedCount, the map where each element is relative at one joint (joint name is the key).
     * The value is the number of times that joint is used, for primitive actions can be only 0 or 1
     */
    ROSEE::JointsInvolvedCount setOnlyDependentJoints(std::pair < std::string, std::string > tipsNames, JointPos *Jstates);
    
    /**
     * @brief Utility function to take the actuated joint positions from a \p kinematic_state and returns the same info as a \ref JointPos map
     * @param kinematic_state [in] pointer to the robot_state class
     * @return JointPos the map with the joint positions info
     */
    JointPos getConvertedJointPos(const robot_state::RobotState* kinematic_state);
    



};
    
}


#endif //__ROSEE_FIND_ACTIONS_
