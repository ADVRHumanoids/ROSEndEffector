#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collisionTest");
    ros::AsyncSpinner spinner(1);
    spinner.start();
  
    //it is a ros param in the launch, take care that also sdrf is read (param: robot_description_semantic)
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); 
  
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();


    for ( auto it : kinematic_model->getEndEffectors() ){
            std::vector < std::string> tips;

        it->getEndEffectorTips(tips);
        std::cout << "number:  " << tips.size() << std::endl; 
        for (auto itt : tips) {
        
            std::cout << "\t" << itt << std::endl;
            
        }
    }
    
    std::cout << "baaaaaa\n";
    for (auto it: kinematic_model->getJointModelGroups()) {
        
        std::string logGroupInfo;
        logGroupInfo = "Found Joint Group '" + it->getName() + "', " ;
        
        if (it->getSubgroupNames().size() != 0 ) {
            logGroupInfo.append("but it has some subgroups \n");
            
        } else {
            if (! it->isChain()) {
                logGroupInfo.append("but it is not a chain \n");
                
            } else {
                logGroupInfo.append("with links: \n");
                for (auto itt : it->getLinkModels()) {
                   
                    logGroupInfo.append("\t'" + itt->getName() + "' ");
                    if (itt->getChildJointModels().size() != 0) {
                       
                       logGroupInfo.append("not a leaf link (not a fingertip)\n");
                       
                    } else { //TODO check if only a leaf link per finger
                       logGroupInfo.append("a leaf link (a fingertip)\n");
                    }
               }
                
            }
        }
        
        ROS_INFO_STREAM (logGroupInfo);
        
        
    }
    
    
    ros::shutdown();
    return 0;
  
}
