#ifndef __ROSEE_MOVEIT_COLLIDER_
#define __ROSEE_MOVEIT_COLLIDER_

#include <ros/console.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

//yaml
#include <yaml-cpp/yaml.h>

#include <ROSEndEffector/Utils.h>

#define N_EXP_COLLISION 5000 //5000 is ok
#define DEFAULT_JOINT_POS 0.0
/** Max contact stored in the set for each pair */
#define MAX_CONTACT_STORED 3
#define COLLIDER_REL_PATH "/configs/moveItCollider/"

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
    /** a map containing pairs jointNames-jointValues. vector of double because a joint can have more than 1 dof @NOTE: being a map the order (given by joint names) is assured*/
    typedef std::map <std::string, std::vector <double> > JointStates;

    /**Contact informations for a contact that happens with a particular joint states*/
    typedef std::pair <collision_detection::Contact, JointStates> ContactWithJointStates;     
        
    /** struct to put in order the set of ContactWithJointStates. The first elements are the ones 
     * with greater depth
     * @FIX, even if is almost impossible, two different contact with same depth will be considered equal
     * with this definition of depthComp. Theoretically they are equal only if the joint status are equal 
     * (of only joints that act for the collision). In fact, we should have the possibility to have two contact
     * with the same depth (if joint statuses are different), they will be equally good
     */
    struct depthComp {
        bool operator() (const ContactWithJointStates& a, const ContactWithJointStates& b) const
        {return (std::abs(a.first.depth) > std::abs(b.first.depth) );}
    };
    /** The object that contains all the "best" MAX_CONTACT_STORED contact for each possible pair. 
     It is a map with as key the pair of the two tips colliding, 
     and as value a fixed size set of ContactWithJointStates object*/
    std::map < std::pair < std::string, std::string >, std::set<ContactWithJointStates, depthComp>> pinchMap; 
        
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
    
    /**
     * @brief Given the contact, we want to know the state of the joint to replicate it. But we want to know
     * only the state of the joints that effectively act on the contact, that are the ones which moves one of the two tips (or both). So the other joints are put to the DEFAULT_JOINT_POS value
     */
    void setOnlyDependentJoints(std::pair < std::string, std::string > tipsNames, ContactWithJointStates *contactJstates);

    /** 
     * @brief insert the new contact in the map, if it is among the best ones
     */
    bool checkBestCollision(std::pair < std::string, std::string > tipsNames, ContactWithJointStates contactJstates);
    

    std::string emitYaml();
    void parseYaml(std::string);
};
    
}


#endif //__ROSEE_MOVEIT_COLLIDER_
