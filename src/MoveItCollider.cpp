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
    printFingertipLinkNames();
    lookJointsTipsCorrelation();
    printJointsOfFingertips();
    printFingertipsOfJoints();
    checkCollisions();
    for (auto it : mapOfPinches ){
        it.second.printAction();
    }
    
    //emit the yaml file
    ROSEE::YamlWorker yamlWorker(kinematic_model->getName());

    std::map < std::set <std::string> , ActionPrimitive* > mapForWorker;
    for (auto& it : mapOfPinches) {  // auto& and not auto alone!

        ActionPrimitive* pointer = &(it.second);
        
        std::set < std::string > keys ;
        keys.insert (it.first.first) ;
        keys.insert (it.first.second) ;

        mapForWorker.insert (std::make_pair ( keys, pointer ) );
                
    }
    
    yamlWorker.createYamlFile(mapForWorker);
    
    
    //auto pinchParsedMap = yamlWorker.parseYaml(actionPinch.name + ".yaml");
    
    //print to check if parse is correct, DEBUG
//     for (auto i : pinchParsedMap) {
//         std::cout << i.first.first << " " << i.first.second << std::endl;
//         for (auto j : i.second) {
//             std::cout << "\t" << j.first << ":" << std::endl;
//             for (auto y : j.second) {
//                 std::cout << "\t\t" <<y.first << ": " << y.second.at(0) << std::endl;                
//             }
//         }
//     }
    
    
    //Trigger actions etc
    //trig();

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


///TODO move i look for in Parser
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
                jointStates.insert(std::make_pair(actJ->getName(), vecPos));
            }
            
            //for each collision with this joints state...
            for (auto cont : collision_result.contacts){

                //moveit contacts is a map between a pair (2 strings with link names) and a vector of Contact object ?? I don't know why the contact is a vector, I have always find only one element            
                logCollision << "Collision between " << cont.first.first << " and " << 
                                                    cont.first.second << std::endl;                                

                for (auto contInfo : cont.second){ 
                    logCollision << "\tWith a depth of contact: " << contInfo.depth;
                }
                
                setOnlyDependentJoints(cont.first, &jointStates);
                
                //create the actionPinch
                ActionPinch pinch (cont.first, jointStates, cont.second.at(0) );
                std::pair <std::string, std::string> key4map = pinch.tipsPair;
                auto itFind = mapOfPinches.find ( key4map );
                if ( itFind == mapOfPinches.end() ) {
                    mapOfPinches.insert ( std::make_pair (key4map, pinch) );
                    logCollision << ", NEW INSERTION";

                    
                } else { //Check if it is the best depth among the found collision among that pair
                    
                    if (itFind->second.insertActionState( jointStates, cont.second.at(0)) ) {
                         logCollision << ", NEW INSERTION";
                    }
                    
                }
                logCollision << std::endl;
            }
            
            //last print for joint states
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
    
    //print if no collision at all 
    if (mapOfPinches.size() == 0 ) {
        std::cout << "WARNING: I found no collisions between tips. Are you sure your hand"
            << " has some fingertips that collide? If yes, check your urdf/srdf, or"
            << " set a bigger value in N_EXP_COLLISION." << std::endl;
    }
}


void ROSEE::MoveItCollider::setOnlyDependentJoints(
    std::pair < std::string, std::string > tipsNames, JointStates *jStates) {
    
    for (auto &js : *jStates) { //for each among ALL joints
        
        /** other way around, second is better?
        std::vector <std::string> jointOfTips1 = jointsOfFingertipsMap.at(tipsNames.first);
        std::vector <std::string> jointOfTips2 = jointsOfFingertipsMap.at(tipsNames.second);
        
        // if the joint is not linked with neither of the two colliding tips...
        if ( std::find( jointOfTips1.begin(), jointOfTips1.end(), js.first) == jointOfTips1.end() &&
             std::find( jointOfTips2.begin(), jointOfTips2.end(), js.first) == jointOfTips2.end() ) {
              
            std::fill ( js.second.begin(), js.second.end(), DEFAULT_JOINT_POS);                     
        }
        */
        
        std::vector < std::string> tips = fingertipOfJointMap.at(js.first); //the tips of the joint
        
        //check if the two tips that collide are among the ones that the joint moves
        if (std::find (tips.begin(), tips.end(), tipsNames.first) == tips.end() &&
            std::find (tips.begin(), tips.end(), tipsNames.second) == tips.end() ) {
            // not dependant, set to zero the position
            std::fill ( js.second.begin(), js.second.end(), DEFAULT_JOINT_POS); 
        }
    } 
    
}

/**
/// trig is the action of closing a SINGLE finger towards the palm
/// to know the joint direction, the position is set to the limit which is different from 0
/// if no limit is 0? TODO think, now solution is to take user info.
/// if a joint is continuos, it is excluded from the trig action. (because I cant think about a continuos joint
/// that is useful for a trig action, but can be present in theory)
void ROSEE::MoveItCollider::trig() {

    std::map < std::string, JointStates > trigMap;    
    for (auto mapEl : fingertipOfJointMap) {
        
        if (mapEl.second.size() == 1) { //the joint must move ONLY a fingertip
                        
            moveit::core::JointModel::Bounds limits = 
                kinematic_model->getJointModel(mapEl.first)->getVariableBounds();

            //HACK consider only 2 bounds now, because 1dof joint
            if ( limits.at(0).max_position_ - limits.at(0).min_position_ >= 6.28 ) { //continuos joint
                std::cout << limits.at(0).max_position_ - limits.at(0).min_position_ << std::endl;
                break;
            }
            //TODO if neither == 0, take trig value from file? or param
            double trigMax = (limits.at(0).max_position_ == 0) ? limits.at(0).min_position_ :   
                                                                 limits.at(0).max_position_ ;
                                                                
            auto itTrigMap = trigMap.find(  mapEl.second.at(0) );
            if (itTrigMap == trigMap.end() ) {
                ActionPinch::JointStates js;
                for (auto it : kinematic_model->getActiveJointModels()){
                    std::vector <double> jPos (it->getVariableCount());
                    std::fill (jPos.begin(), jPos.end(), 0.0);
                    js.insert ( std::make_pair ( it->getName(), jPos ));
                }
                
                //HACK consider only 2 bounds now, because 1dof joint
                js.at ( mapEl.first ).at(0) = trigMax;
                trigMap.insert ( std::make_pair ( mapEl.second.at(0), js ) );

            } else {
                //HACK consider only 2 bounds now, because 1dof joint
                itTrigMap->second.at (mapEl.first).at(0) = trigMax;
            }
        }  
    }
    
    //print debug
    for (auto i : trigMap) {
        std::cout << i.first << std::endl;
        for (auto j : i.second) {
            std::cout << "\t" << j.first << " : " ;
            for(const auto &jointValue : j.second){
                std::cout << jointValue << ", "; //joint position (vector because can have multiple dof)
            }
            std::cout << std::endl;
        }
    }
    
    
}

*/
