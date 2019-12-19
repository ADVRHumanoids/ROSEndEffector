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

void ROSEE::MoveItCollider::printNotFingertipNames(){

    std::cout << "Not-Fingertips list:" << std::endl;
    for (auto it: notFingertipNames) {
        std::cout << it << std::endl;
    }
    std::cout << std::endl;
    
}

void ROSEE::MoveItCollider::run(){
    
    lookForFingertips();
    printNotFingertipNames();
    checkCollisions();
    
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
                       notFingertipNames.push_back(itt->getName());
                       
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

void ROSEE::MoveItCollider::checkCollisions(){
        
    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;  //set to compute collisions
    collision_request.max_contacts = 1000;
    
    const std::vector<moveit::core::JointModel*> actJoints = kinematic_model->getActiveJointModels();
    
    for (auto it: actJoints){
    
        std::cout << "jointname: " << it->getName() << std::endl;
        
    }
    
    // Consider only collisions among fingertips 
    // If an object pair does not appear in the acm, it is assumed that collisions between those 
    // objects is no tolerated. So we must fill it with all the nonFingertips
    
    collision_detection::AllowedCollisionMatrix acm;
    acm.setEntry(notFingertipNames, kinematic_model->getLinkModelNames(), true); //true==not considered collisions
    robot_state::RobotState kinematic_state(kinematic_model);
    
    for (int i=0; i<100; i++){
        collision_result.clear();
        kinematic_state.setToRandomPositions();

        planning_scene.checkSelfCollision(collision_request, collision_result, kinematic_state, acm);
        for (auto it : collision_result.contacts){
            //contacts is a map between a pair (2 strings with link names) and a vector of Contact object            
            std::cout << "Collision between " << it.first.first.c_str() << " and " << 
                                                it.first.second.c_str() << std::endl;
            std::cout << "With the configuration:" << std::endl ;
            kinematic_state.printStatePositions();
            for (auto itt : it.second){ 
                std::cout << "With a depth of contact: " << itt.depth << std::endl;
            }
            std::cout << std::endl;


        }
    }
    
    //TODO, IDEA: for each couple which collide at least once, do other checkcollisionf with
    //set randomPosNEAR, so more probability to find more depth contact for that pair
    
    
}
