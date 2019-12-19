#include <ROSEndEffector/MoveItCollider.h>

ROSEE::MoveItCollider::MoveItCollider(){

    //it is a ros param in the launch, take care that also sdrf is read (param: robot_description_semantic)
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); 
    kinematic_model = robot_model_loader.getModel();
}

ROSEE::MoveItCollider::MoveItCollider(std::string robot_description){

    //it is a ros param in the launch, take care that also sdrf is read (param: robot_description_semantic)
    robot_model_loader::RobotModelLoader robot_model_loader(robot_description); 
    kinematic_model = robot_model_loader.getModel();
}

void ROSEE::MoveItCollider::printFingertipNames(){

    std::cout << "Fingertips list:" << std::endl;
    for (auto it: fingertipNames) {
        std::cout << it << std::endl;
    }
    std::cout << std::endl;
    
}

void ROSEE::MoveItCollider::run(){
    
    lookForFingertips();
    
}

void ROSEE::MoveItCollider::lookForFingertips(){

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
                       fingertipNames.push_back(itt->getName());
                    }
               }
                
            }
        }
        std::cout << logGroupInfo;        
    }    
}
