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

void ROSEE::MoveItCollider::printFingertipLinkNames(){

    std::cout << "Fingertips list:" << std::endl;
    for (const auto &it: fingertipNames) {
        std::cout << it << std::endl;
    }
    std::cout << std::endl;
    
}

void ROSEE::MoveItCollider::printAllLinkNames(){

    std::cout << "All Links  list:" << std::endl;
    for (const auto &it: kinematic_model->getLinkModelNames()) {
        std::cout << it << std::endl;
    }
    std::cout << std::endl;    
}

void ROSEE::MoveItCollider::printActuatedJoints(){

    const std::vector<moveit::core::JointModel*> actJoints = kinematic_model->getActiveJointModels();
    for (const auto &it: actJoints){
        std::cout << "jointname: " << it->getName() << std::endl;
    }  
    std::cout << std::endl;

}

void ROSEE::MoveItCollider::printBestCollisions(){
    
    for (const auto &it : contactWithJointStatesVect){
        std::cout  << "contact list: " <<
            it.first.body_name_1 << ", " <<
            it.first.body_name_2 << ": " << 
            it.first.depth << 
            std::endl;
    }
    std::cout << std::endl;
}

void ROSEE::MoveItCollider::run(){
    
    lookForFingertips();
    printFingertipLinkNames();
    checkCollisions();
    printBestCollisions();

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
                       
                    } else {
                       logGroupInfo.append("a leaf link (a fingertip)\n");
                       fingertipNames.push_back(itt->getName());
                    }
                }
            }
        }
        std::cout << logGroupInfo; 
    }
    

    if (fingertipNames.size() > 1){
        // reserve space, we know the max size (that is when all fingertips can be in contact)
        //TODO or not reserve space??
        contactWithJointStatesVect.reserve(ROSEE::Utils::binomial_coefficent(fingertipNames.size(),2));
    } else {
        //TODO no pinch with only one fingertip, print some message
    }
}

void ROSEE::MoveItCollider::checkCollisions(){
        
    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;  //set to compute collisions
    collision_request.max_contacts = 1000;
    

    // Consider only collisions among fingertips 
    // If an object pair does not appear in the acm, it is assumed that collisions between those 
    // objects is no tolerated. So we must fill it with all the nonFingertips
    
    collision_detection::AllowedCollisionMatrix acm;
    acm.setEntry(kinematic_model->getLinkModelNames(), kinematic_model->getLinkModelNames(), true); //true==not considered collisions
    acm.setEntry(fingertipNames, fingertipNames, false); //true==not considered collisions

    robot_state::RobotState kinematic_state(kinematic_model);
    
    
    for (int i=0; i<5000; i++){
        collision_result.clear();
        kinematic_state.setToRandomPositions();

        planning_scene.checkSelfCollision(collision_request, collision_result, kinematic_state, acm);
        for (auto cont : collision_result.contacts){
            //contacts is a map between a pair (2 strings with link names) and a vector of Contact object            
            std::cout << "Collision between " << cont.first.first.c_str() << " and " << 
                                                cont.first.second.c_str() << std::endl;
            std::cout << "With the configuration:" << std::endl ;
            //kinematic_state.printStatePositions();
        
            JointStates jointStates;
            ContactWithJointStates contactJstates;
            
            
            for (auto actJ : kinematic_model->getActiveJointModels()){
                std::string logInfo = "\tJoint " + actJ->getName() + " : " ;
                //joint can have multiple pos (eg planar joint?), so double*
                const double* pos = kinematic_state.getJointPositions(actJ); 
                for (unsigned int i =0; i< sizeof(pos)/sizeof(pos[0]); i++){
                    logInfo.append(std::to_string(pos[i]) + ", ");
                }
                std::cout << logInfo << std::endl;
                jointStates.push_back(std::make_pair(actJ->getName(), pos));
                
            }
            
            //?? I don't know why the contact is a vector, I have always find only one element
            for (auto contInfo : cont.second){ 
                std::cout << "With a depth of contact: " << contInfo.depth << std::endl;
            }
            
            //store contact and joint states, to pass as argument to checkBestCollision()
            //?? I don't know why the cont.second is a vector, I have always find only one element
            contactJstates = std::make_pair(cont.second.at(0), jointStates); 
            
            // be sure to have two links name in order, so comparison in checkBestCollision is easier
            if (contactJstates.first.body_name_1.compare ( contactJstates.first.body_name_2 ) > 0 ){
                //swap names
                std::swap(contactJstates.first.body_name_1, contactJstates.first.body_name_2);
                //invert depth
                contactJstates.first.depth *= -1;
                //I would keep the normal of contact and the contact position as they are
            }
            
            //Check if it is the best depth among the found collision among that pair
            checkBestCollision(contactJstates);
            
        }
    }
    
    //TODO, IDEA: for each couple which collide at least once, do other checkcollisionf with
    //set randomPosNEAR, so more probability to find more depth contact for that pair
        
}

bool ROSEE::MoveItCollider::checkBestCollision(ContactWithJointStates contactJstates){
        
    for ( auto &savedContactJstate : contactWithJointStatesVect){
            
        //order is assured by function checkCollisions
        if ( contactJstates.first.body_name_1.compare(savedContactJstate.first.body_name_1) == 0 && 
             contactJstates.first.body_name_2.compare(savedContactJstate.first.body_name_2) == 0 ) {
            //if so, we already have this contact. We need to check if the new one is better
            //At the moment, to check the best, the one with greater depth of compenetration is taken
            
            if (std::abs(contactJstates.first.depth) > std::abs(savedContactJstate.first.depth) ) { 
                savedContactJstate = contactJstates;
                return true;
                
            } else {
                return false; //no new contact has been added
            }
        }
    }
    
    //if here, new contact (otherwise function would have returned before
    contactWithJointStatesVect.push_back(contactJstates);
        
    return true;
}
