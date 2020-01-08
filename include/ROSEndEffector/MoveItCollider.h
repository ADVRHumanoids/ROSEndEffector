#ifndef __ROSEE_MOVEIT_COLLIDER_
#define __ROSEE_MOVEIT_COLLIDER_

#include <unordered_set>

#include <ros/console.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <ROSEndEffector/Utils.h>

#define N_EXP_COLLISION 5000 //5000 is ok

namespace ROSEE
{
    
/**
 * @brief Class to check which fingertips collide (for the pinch action at the moment)
 * 
 */
class MoveItCollider
{
public:
    MoveItCollider();
    MoveItCollider(std::string);
    void run();
    void printFingertipLinkNames();
    void printAllLinkNames();
    void printActuatedJoints();
    void printBestCollisions();
    void printJointsOfFingertips();
    void printFingertipsOfJoints();

    
private:
    /** a vector containing pairs jointNames-jointValues. vector of double because a joint can have more than 1 dof */
    typedef std::vector < std::pair<std::string, std::vector <double> > > JointStates; 

    /**Contact informations for a contact that happens with a particular joint states*/
    typedef std::pair <collision_detection::Contact, JointStates> ContactWithJointStates;     
    
    /** The object that contains all the "best" contact for each possible pair. 
     It is a map with key the pair of the two tips colliding, and as value a ContactWithJointStates object*/
    std::map < std::pair < std::string, std::string >, ContactWithJointStates> contactWithJointStatesMap; 
    
    robot_model::RobotModelPtr kinematic_model;
    std::vector<std::string> fingertipNames;
    /** The map with as key the name of the fingertip and as value all the joints (actuated) that can modify its pose*/
    std::map<std::string, std::vector<std::string>> jointsOfFingertipsMap;
    /** The map with as key the name of the actuated joint and as value all the fingertips which pose can be modified by the joint */
    std::map<std::string, std::vector<std::string>> fingertipOfJointMap;

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
    void checkCollisions();
    bool checkBestCollision(std::pair < std::string, std::string > tipsNames, ContactWithJointStates contactJstates);
    void setOnlyDependentJoints();

};
    
}



#endif //__ROSEE_MOVEIT_COLLIDER_
