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
    void lookForFingertips();
    std::vector<std::string> fingertipNames;
    void checkCollisions();

};
    
}



#endif //__ROSEE_MOVEIT_COLLIDER_
