#ifndef __ROSEE_MOVEIT_COLLIDER_
#define __ROSEE_MOVEIT_COLLIDER_

#include <ros/console.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

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
private:
    robot_model::RobotModelPtr kinematic_model;
    std::vector<std::string> fingertipNames;

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
    void checkCollisions();

};
    
}



#endif //__ROSEE_MOVEIT_COLLIDER_
