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

void ROSEE::MoveItCollider::run(){
    
    lookForFingertips();
    //printFingertipLinkNames();
    lookJointsTipsCorrelation();
    printJointsOfFingertips();
    printFingertipsOfJoints();
    checkCollisions();
    printBestCollisions();
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

    for (const auto &it: kinematic_model->getActiveJointModels()){
        std::cout << "jointname: " << it->getName() << std::endl;
    }  
    std::cout << std::endl;

}

void ROSEE::MoveItCollider::printBestCollisions(){
    std::stringstream logStream;
    logStream << "Contact list: " << std::endl ;
    
    for (const auto &contact : contactWithJointStatesMap){
        logStream << "\t" << contact.first.first << ", " <<
            contact.first.second << ": \n \t\tdepth = " <<
            contact.second.first.depth << "\n \t\tJointStates =\n" ;
        for (const auto &jointState : contact.second.second) {
            logStream << "\t\t" << jointState.first << " : "; //joint name
            for(const auto &jointValue : jointState.second){
                logStream << jointValue << ", "; //joint position (vector because can have multiple dof)
            }
            logStream << std::endl;       
        }
    }
    std::cout << logStream.str() << std::endl;
}

void ROSEE::MoveItCollider::printJointsOfFingertips(){

    std::stringstream logInfo;

    for (const auto &it : jointsOfFingertipsMap) {
        logInfo << it.first << " parent joints: \n";
        for (const auto jointName : it.second) {
            logInfo << "\t" << jointName << std::endl;
        }
    }
    std::cout << logInfo.str() << std::endl;
}

void ROSEE::MoveItCollider::printFingertipsOfJoints(){

    std::stringstream logInfo;

    for (const auto &it : fingertipOfJointMap) {
        logInfo << it.first << " child fingertips: \n";
        for (const auto linkName : it.second) {
            logInfo << "\t" << linkName << std::endl;
        }
    }
    std::cout << logInfo.str() << std::endl;
}



void ROSEE::MoveItCollider::lookForFingertips(){

    for (auto it: kinematic_model->getJointModelGroups()) {
        
        std::string logGroupInfo;
        logGroupInfo = "Found Group '" + it->getName() + "', " ;
        
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
        std::cout << logGroupInfo << std::endl; 
    }

    if (fingertipNames.size() <= 1){
        //TODO no pinch with only one fingertip, print some message and exit pinch program
    }
}

void ROSEE::MoveItCollider::lookJointsTipsCorrelation(){
    
    //initialize the map with all tips and with empty vectors of its joints
    for (const auto &it: fingertipNames) { 
        jointsOfFingertipsMap.insert ( std::make_pair (it, std::vector<std::string>() ) );
    }
    
    //initialize the map with all the actuated joints and an empty vector for the tips that the joint move
    for (const auto &it: kinematic_model->getActiveJointModels()) { 
        fingertipOfJointMap.insert ( std::make_pair (it->getName(), std::vector<std::string>() ) );
    }
    
    for ( const auto &joint: kinematic_model->getActiveJointModels()){ //for each actuated joint        
        for (const auto &link : joint->getDescendantLinkModels()) { //for each descendand link
            
            if (std::find(fingertipNames.begin(), fingertipNames.end(), link->getName()) != fingertipNames.end()){
                jointsOfFingertipsMap.at ( link->getName() ).push_back( joint->getName() );
                fingertipOfJointMap.at ( joint->getName() ).push_back( link->getName() );
            }
        }
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
    
    for (int i = 0; i < N_EXP_COLLISION; i++){
        
        std::stringstream logCollision;
        collision_result.clear();
        kinematic_state.setToRandomPositions();
        planning_scene.checkSelfCollision(collision_request, collision_result, kinematic_state, acm);
        
        if (collision_result.collision) { 

            //store joint states
            JointStates jointStates;
            for (auto actJ : kinematic_model->getActiveJointModels()){
                //joint can have multiple pos, so double*, but we want to store in a vector 
                const double* pos = kinematic_state.getJointPositions(actJ); 
                unsigned posSize = sizeof(pos) / sizeof(double);
                std::vector <double> vecPos(pos, pos + posSize);
                jointStates.push_back(std::make_pair(actJ->getName(), vecPos));
            }
            
            //for each collision with this joints state...
            for (auto cont : collision_result.contacts){

                //moveit contacts is a map between a pair (2 strings with link names) and a vector of Contact object            
                logCollision << "Collision between " << cont.first.first.c_str() << " and " << 
                                                    cont.first.second.c_str() << std::endl;                                
                //?? I don't know why the contact is a vector, I have always find only one element
                for (auto contInfo : cont.second){ 
                    logCollision << "\tWith a depth of contact: " << contInfo.depth;
                }
                
                //Check if it is the best depth among the found collision among that pair
                if ( checkBestCollision ( cont.first, std::make_pair(cont.second.at(0), jointStates)) ) {                        
                    logCollision << ", NEW BEST";
                }
                logCollision << std::endl;
                
            }
            
            for (auto actJ : jointStates){
                logCollision << "\tJoint " << actJ.first << " : " ;
                for (auto &jv : actJ.second){
                    logCollision << jv << ", ";
                }
                logCollision << std::endl;                
            }
        
            //std::cout << logCollision.str() << std::endl;
        }            
    }

    //TODO, IDEA: for each couple which collide at least once, do other checkcollisionf with
    //set randomPosNEAR, so more probability to find more depth contact for that pair
    // and also set random position for only the joints that effectively move the two tips
    
}

bool ROSEE::MoveItCollider::checkBestCollision(
    std::pair < std::string, std::string > tipsNames, ContactWithJointStates contactJstates){
    
    //check if pair already present
    auto it = contactWithJointStatesMap.find(tipsNames);
    if (it == contactWithJointStatesMap.end()) { //new pair
        contactWithJointStatesMap.insert(std::make_pair(tipsNames, contactJstates));
        
    } else { 
        // For now the "best" is the one with more depth
        if ( std::abs(contactJstates.first.depth) > std::abs(it->second.first.depth) ) { 
            
            setOnlyDependentJoints(tipsNames, &contactJstates);
            it->second = contactJstates;
            
        } else {
            return false; //no new added
        }
        
    }
    return true;
}

void ROSEE::MoveItCollider::setOnlyDependentJoints(
    std::pair < std::string, std::string > tipsNames, ContactWithJointStates *contactJstates) {
        
    for (auto &js : contactJstates->second) {
        
        std::vector < std::string> tips = fingertipOfJointMap.at(js.first); //the tips of the joint
        
        //check if the two tips that collide are among the ones that the joint move
        if (std::find (tips.begin(), tips.end(), tipsNames.first) == tips.end() &&
            std::find (tips.begin(), tips.end(), tipsNames.second) == tips.end() ) {
            // not dependant, set to zero the position
            std::fill ( js.second.begin(), js.second.end(), DEFAULT_JOINT_POS);               
        }
    }
    
   
}

