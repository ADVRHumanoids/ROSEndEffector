#ifndef __ROSEE_MOVEIT_COLLIDER_
#define __ROSEE_MOVEIT_COLLIDER_

#include <ros/console.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

//TEst distance
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>


#include <ROSEndEffector/YamlWorker.h>
#include <ROSEndEffector/ActionPrimitive.h>
#include <ROSEndEffector/ActionPinch.h>
#include <ROSEndEffector/ActionTrig.h>


#define N_EXP_COLLISION 5000 //5000 is ok
#define N_EXP_DISTANCES 5000 //? is ok
#define DEFAULT_JOINT_POS 0.0
/** Max contact stored in the set for each pair */

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
    FindActions ( std::string ) ;
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinch > findPinch ( 
        std::string path2saveYaml = "" );
    std::map <std::string, ROSEE::ActionTrig> findTrig (  ROSEE::ActionType actionType,
        std::string path2saveYaml = "" );

    
    void printFingertipLinkNames();
    void printAllLinkNames();
    void printActuatedJoints();
    void printBestCollisions();
    void printJointsOfFingertips();
    void printFingertipsOfJoints();
    
    //TODO should be in the parser
    std::string getHandName();
    
private:
    
    robot_model::RobotModelPtr kinematic_model;
    std::vector<std::string> fingertipNames;
    
    /** The map with as key the name of the fingertip and as value all the joints (actuated) that can modify its pose*/
    std::map<std::string, std::vector<std::string>> jointsOfFingertipMap;
    /** The map with as key the name of the actuated joint and as value all the fingertips which pose can be modified by the joint */
    std::map<std::string, std::vector<std::string>> fingertipsOfJointMap;
    
    /**
     * @brief This function explore the kinematic_model (which was built from urdf and srdf files), 
     *  and fills the fingerTipNames vector.
     *  In particular, the function explores only the groups specified in the srdf, and prints infos
     *  about each link it finds (eg. not a fingertin, not a chain group, and so on).
     *  A fingertip is a link with the following conditions:
     *  - It is part of a group (defined in the srdf)
     *  - The group which the tip is part of is a chain (not a tree)
     *  - It is a "leaf" link, ie it has not children joints/links
     * @todo Check also if it is unique in the group?
     * @warning Only link belonging to a group are explored (and printed), so other links (if present) 
     *  are not considered 
     */
    void lookForFingertips();
    
    /** 
     * @brief Here, we find for each tip, which are all the joints (active) that can modifies its position
     * It is easier to start from each joint and see which tips has as its descendands, because there is the
     * getDescendantLinkModels() function in moveit that gives ALL the child links.
     * There is not a function like getNonFixedParentJointModels from the tip, there is only the one to take the 
     * FIRST parent joint (getParentJointModel())
     * Meanwhile, we find also, for each joint, all the tips that are influenced by the joint movement. This map, 
     * fingertipOfJointMap, is used in setOnlyDependentJoints()
     */
    void lookJointsTipsCorrelation();
        
    
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinch >  checkCollisions();
    void checkDistances (std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* );

    /**
     * @brief Given the contact, we want to know the state of the joint to replicate it. But we want to know
     * only the state of the joints that effectively act on the contact, that are the ones which moves one of the two tips (or both). So the other joints are put to the DEFAULT_JOINT_POS value
     */
    void setOnlyDependentJoints(std::pair < std::string, std::string > tipsNames, JointStates *Jstates);
    
    //trig etc
    std::map <std::string, ActionTrig> trig();
    std::map <std::string, ROSEE::ActionTrig> tipFlex();
    std::map <std::string, ROSEE::ActionTrig> fingFlex();

    
    //other utilities used by this class
    bool insertJointPosForTrigInMap ( std::map <std::string, ActionTrig>& trigMap, 
        ROSEE::ActionTrig action, std::string jointName, double trigValue);  
    bool checkIfContinuosJoint ( std::string jointName) ;
    bool checkIfContinuosJoint ( const moveit::core::JointModel* joint ) ;
    double getBiggestBound (std::string jointName ) ;
    double getBiggestBound ( const moveit::core::JointModel* joint ) ;
    unsigned int getNExclusiveJointsOfTip (std::string tipName);

    
    void fillNotCollidingTips ( std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >*, 
             const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinch >* );
    JointStates getConvertedJointStates(const robot_state::RobotState* kinematic_state);


};
    
}


#endif //__ROSEE_MOVEIT_COLLIDER_
