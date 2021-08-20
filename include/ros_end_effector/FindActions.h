#ifndef __ROSEE_FIND_ACTIONS_
#define __ROSEE_FIND_ACTIONS_

#include <moveit/planning_scene/planning_scene.h>

#include <ros_end_effector/YamlWorker.h>
#include <ros_end_effector/ParserMoveIt.h>
#include <ros_end_effector/GraspingActions/Action.h>
#include <ros_end_effector/GraspingActions/ActionPinchTight.h>
#include <ros_end_effector/GraspingActions/ActionPinchLoose.h>
#include <ros_end_effector/GraspingActions/ActionTrig.h>
#include <ros_end_effector/GraspingActions/ActionSingleJointMultipleTips.h>
#include <ros_end_effector/GraspingActions/ActionMultiplePinchTight.h>

#include <muParser.h>


#define N_EXP_COLLISION 5000 //5000 is ok
#define N_EXP_DISTANCES 5000 //? is ok
#define N_EXP_COLLISION_MULTPINCH 3000
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
     * @brief Function to look for pinches, both Tight and Loose ones. It fill the maps (returned as pair), but also 
     * print the infos in two yaml files (one for tight, one for loose) using a \ref YamlWorker
     * @param path2saveYaml the path where to create/overwrite the yaml files. 
     * @return a pair of maps. The first is the map of \ref ActionPinchTight, the second the map of \ref ActionPinchLoose
     */
    std::pair <  std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchTight >, 
                 std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose >  > 
                 findPinch ( std::string path2saveYaml );
            
    /**
     * @brief Finder for MultiplePinch (a pinch done with more than 2 finger). This function
     * return the found multpinch primitive but it also print this result in a yaml file.
     * See \ref checkCollisionsForMultiplePinch doc for more info.
     * 
     * @param nFinger (2 < nFinger <= max_finger). The type of the multiple pinch that we want.
     *      The name of the returned action will be based on this param : 
     *      "MultiplePinchTight-(nFinger)"
     * @param strict true to look only for "strict" multiple pinch, i.e. where all tips collide
     *  with each other (and do not collide in "line") 
     *  See \ref checkCollisionsForMultiplePinch doc for more info.
     * @param path2saveYaml the path where to create/overwrite the yaml files. 
     * @return map of founded \ref ActionMultiplePinchTight, 
     */
    std::map < std::set <std::string>, ROSEE::ActionMultiplePinchTight > findMultiplePinch (
        unsigned int nFinger, std::string path2saveYaml, bool strict = true );
                 
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
        std::string path2saveYaml );
    
    std::map < std::string, ROSEE::ActionSingleJointMultipleTips> findSingleJointMultipleTips ( unsigned int nFinger, std::string path2saveYaml );
    
private:
    
    std::shared_ptr < ROSEE::ParserMoveIt > parserMoveIt;
    
    //lets store this, we access at each setRandomPos
    std::map<std::string, std::pair<std::string, std::string>> mimicNLRelMap;
    
    /**
     * @brief principal function which check for collisions with moveit functions when looking for tight pinches
     * @return the map of ActionPinchTight
     * 
     */
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchTight >  checkCollisions();
    
    /**
     * @brief Principal function which fill the \p mapOfLoosePinches basing on minumun distance between tips
     * @param mapOfLoosePinches [out] pointer to the mapOfLoosePinches to be filled
     */
    void checkDistances (std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose >* mapOfLoosePinches);
    
    /**
     * @brief Function similar to \ref checkCollisions but used for Loose Pinches.
     * First, we temporarily remove bounds of joints linked to the non-colliding tips (with \ref removeBoundsOfNotCollidingTips),
     * and we check for collision with this function.
     * If some collision are found, this means that tips movements make them go towards each other, (also with the bounds) but
     * the joint limits do not permit them to touch. This is a loose pinch. 
     * If they do not collide even without bounds, this means that they never go towards each other, so this is not a tight neither loose
     * pinch. 
     * We also create new kinematic_model object so we dont modify the original kinematic_model, and we can change the joint limits 
     * of the new copy safely
     * 
     * @param mapOfLoosePinches [out] map of loose pinches that will be filled with info about these particular actions
     */
    void checkWhichTipsCollideWithoutBounds (
        std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose >* mapOfLoosePinches);
    
    /**
     * @brief support function for \ref findMultiplePinch (a pinch done with more than 2 finger).
     *      This is done similarly to normal pinch, but there are more checks to see
     *      if the collision is among more than only two fingertips. 
     * 
     * Moveit, when looking for collisions, return only pairs of links that collide. 
     * So we need to handle all the found pairs and "put them togheter" in someway.
     * We need at least \param nFinger - 1 collision: eg. for triPinch -> 2collision,
     * for 4finger pinch -> 3 collision. But, with only this check, we can also find 
     * a configuration whith two distint normal pinch. To solve, we also check if the number of
     * tips that collide in this configuration is exaclty \param nFinger ,
     * eg with 2 collision we can have 4 finger colliding because there are two
     * normal distinct pinch and not a 3-pinch... so we exlude these collisions.
     * The \param strict can solves another problem. If it is true (default) we take only
     * the multiple pinch where each finger collide with all the other finger involved in 
     * the pinch. If it is false, we can find also pinch where the tips are "in line" :
     * finger_2 collide with finger_1 and finger_3, but finger_1 and 3 do not collide.
     * With strict we will find less groups of fingers that collide, but, in someway, they
     * collide "better".
     * 
     * @param nFinger (2 < nFinger <= max_finger). The type of the multiple pinch that we want.
     *      The name of the returned action will be based on this param : 
     *      "MultiplePinchTight-(nFinger)"
     * @param strict true to look only for "strict" multiple pinch. Look this funcion description
     * @return A map with as keys set of size nFinger, and as value an 
     *      \ref ActionMultiplePinchTight object
     */
    std::map <std::set<std::string>, ROSEE::ActionMultiplePinchTight>
        checkCollisionsForMultiplePinch(unsigned int nFinger, bool strict);

    /**
     * @brief Support function to remove the joint limits from the model. This is done when looking for Loose Pinches.
     * @param mapOfLoosePinches [in] pointer to the map of loose pinches
     * @param kinematic_model_noBound the pointer to the robot model
     */
    void removeBoundsOfNotCollidingTips ( const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose >* mapOfLoosePinches, 
                                          robot_model::RobotModelPtr kinematic_model_noBound);
    
    /**
     * @brief function to "initialize" the map of ActionPinchLoose \p mapOfLoosePinches. 
     * It is done adding all the tips pairs and then removing 
     * the pairs that are present in the map of ActionPinchTight \p mapOfPinches. Note that the values of the map, the \ref ActionPinchLoose
     * are action with only tips name (so no info right now).
     * 
     * @param mapOfLoosePinches [out] Pointer to the map of \ref ActionPinchLoose to be initialized
     * @param mapOfPinches [in] pointer to the map of \ref ActionPinchTight, already filled before, that is used to erase the get the tips that collide
     *      and to remove them from the \p mapOfLoosePinches
     */
    void fillNotCollidingTips ( std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose >* mapOfLoosePinches,
        const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchTight >* mapOfPinches );
    
    /**
     * @brief this function take the two tight and loose maps and change the keys from fingertips
     *  names to their finger names. 
     * 
     * @param mapOfLoosePinches [out] Pointer to the map of \ref ActionPinchLoose
     * @param mapOfPinches [out] pointer to the map of \ref ActionPinchTight
     * 
     * @warning The order in the pair is lexicographical so the first finger in the 
     *  pair can refer to the second tip in the old key pair
     */
    void changeFingertipsToFingerNames ( 
        std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchTight >* mapOfPinches, 
        std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose >* mapOfLoosePinches) ;

    
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
    ROSEE::JointsInvolvedCount setOnlyDependentJoints(std::pair < std::string, std::string > tipsNames, JointPos *jPos);
    
    
    /**
     * @brief Set to default pos the joints that do not move any of the tip included in the 
     * set \param tipsNames. Used by \ref findMultiplePinch function
     * @param tipsNames the tips involved
     * @param jPos pointer to the map \ref JointPos with value to be setted if necessary
     * @return JointsInvolvedCount map, where value are 0 or 1 according to the usage of joint
     */
    ROSEE::JointsInvolvedCount setOnlyDependentJoints( std::set< std::string > tipsNames, 
                                                       JointPos *jPos);
    
    /**
     * @brief Utility function to take the actuated joint positions from a \p kinematic_state and returns the same info as a \ref JointPos map
     * @param kinematic_state [in] pointer to the robot_state class
     * @return JointPos the map with the joint positions info
     */
    JointPos getConvertedJointPos(const robot_state::RobotState* kinematic_state);
    
    
    /**
     * @brief set to \ref DEFAULT_JOINT_POS all the passive joints (defined so in
     * the urdf file). this is necessary because moveit setToRandomPositions modify the position of passive joints,
     * we do not want that
     */
    void setToDefaultPositionPassiveJoints(moveit::core::RobotState * kinematic_state);
    
    /**
     * @brief Giving as argument a pair of fingertips, this function return a pair of fingers that
     *   are the fingers which the two tips belong to.
     *  
     * @return a pair of string containing the fingers which the passed tips belong to
     */
    std::pair < std::string, std::string > getFingersPair (std::pair <std::string, std::string> tipsPair) const;
    
    /**
     * @brief Function used when looking for multiple pinches. It returns the set containing
     *   the fingers of the passed fingertips. 
     * @param tipsSet the set of fingertips names
     * @return the set of fingers. Empty set if the some tips in the \ref tipsSet are in
     *   the same finger (that is an error)
     */
    std::set <std::string> getFingersSet (std::set <std::string> tipsSet) const;
    
    /**
     * @brief Given the \ref fingersPair, this function return the pair of their fingers, in 
     *   lexicographical order
     * 
     * @return a pair of string containing the fingers which the passed tips belong to
     * 
     */
    std::pair < std::string, std::string > getFingertipsPair (std::pair <std::string, std::string> fingersPair) const;

    /**
     * @brief This function set the random position of joint considering:
     * 
     *   - Non linear mimic joint relationship, if present
     *   - Passive joints, which default position will be assured
     *   - Positional limit of also mimic joint will be enforced
     * 
     *   These three things are not present in the moveit setToRandomPositions. So we use the moveit one but then 
     *   we change a bit the things.
     */
    void setToRandomPositions(robot_state::RobotState* kinematic_state);


};
    
}


#endif //__ROSEE_FIND_ACTIONS_
