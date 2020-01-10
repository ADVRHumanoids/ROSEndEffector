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
    //printJointsOfFingertips();
    //printFingertipsOfJoints();
    checkCollisions();
    //printBestCollisions();
    
    //emit the yaml file
    ROSEE::Utils::create_directory(ROSEE::Utils::getPackagePath() + COLLIDER_REL_PATH);
    std::string output = emitYaml();
    //std::cout << output;
    ROSEE::Utils::out2file(ROSEE::Utils::getPackagePath() + COLLIDER_REL_PATH + "/pinch.yaml", output);
    
    parseYaml(ROSEE::Utils::getPackagePath() + COLLIDER_REL_PATH + "/pinch.yaml");

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

void ROSEE::MoveItCollider::printBestCollisions(){
    std::stringstream logStream;
    logStream << "Contact list: " << std::endl ;
    
    for (const auto &item : pinchMap){
        logStream << "\t" << item.first.first << ", " << item.first.second << ": "<< std::endl;
            
        for (auto contact : item.second) {  //the element in the set
            logStream << "\tWith depth of: " << contact.first.depth << " and joint states:" << std::endl;
            
            for (const auto &jointState : contact.second) {
                logStream << "\t\t" << jointState.first << " : "; //joint name
                for(const auto &jointValue : jointState.second){
                    logStream << jointValue << ", "; //joint position (vector because can have multiple dof)
                }
            logStream << std::endl;       
            }
        }
        logStream << std::endl;
    }    
    std::cout << logStream.str() << std::endl;
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
                jointStates.insert(std::make_pair(actJ->getName(), vecPos));
            }
            
            //for each collision with this joints state...
            for (auto cont : collision_result.contacts){

                //moveit contacts is a map between a pair (2 strings with link names) and a vector of Contact object ?? I don't know why the contact is a vector, I have always find only one element            
                logCollision << "Collision between " << cont.first.first.c_str() << " and " << 
                                                    cont.first.second.c_str() << std::endl;                                

                for (auto contInfo : cont.second){ 
                    logCollision << "\tWith a depth of contact: " << contInfo.depth;
                }
                
                //Check if it is the best depth among the found collision among that pair
                if ( checkBestCollision ( cont.first, std::make_pair(cont.second.at(0), jointStates)) ) {                        
                    logCollision << ", NEW INSERTION";
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
}


void ROSEE::MoveItCollider::setOnlyDependentJoints(
    std::pair < std::string, std::string > tipsNames, ContactWithJointStates *contactJstates) {
    
    for (auto &js : contactJstates->second) { //for each among ALL joints
        
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


bool ROSEE::MoveItCollider::checkBestCollision(
    std::pair < std::string, std::string > tipsNames, ContactWithJointStates contactJstates){
    
    auto it = pinchMap.find(tipsNames);
    if (it == pinchMap.end()) { //new pair
        
        setOnlyDependentJoints(tipsNames, &contactJstates);
        std::set<ContactWithJointStates, depthComp> newSet;
        newSet.insert(contactJstates);
        
        pinchMap.insert(std::make_pair(tipsNames, newSet));
        
    } else if (it->second.size() < MAX_CONTACT_STORED){
            
        setOnlyDependentJoints(tipsNames, &contactJstates);
        it->second.insert(contactJstates); // the set will insert in order for us
            
    } else if (contactJstates.first.depth > it->second.rbegin()->first.depth) {

        setOnlyDependentJoints(tipsNames, &contactJstates);
        it->second.insert(contactJstates);
        //delete the last element
        std::set<ContactWithJointStates, depthComp>::iterator lastElem = it->second.end();
        --lastElem;
        it->second.erase(lastElem);
        
    } else {
        return false; //no new added
    }
    return true;
}

//TODO make in a function of another class (parser?)
std::string ROSEE::MoveItCollider::emitYaml() {

    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto & tipPair : pinchMap) {
    
        //yaml does not accept a pair, we have to "convert" it into a vector
        const std::vector <std::string> tipNamesStr { tipPair.first.first, tipPair.first.second };
        out << YAML::Key << YAML::Flow << tipNamesStr;
        
        unsigned int nCont = 1;
        out << YAML::Value << YAML::BeginMap;
        for (const auto & contact : tipPair.second) {
            std::string contSeq = "Contact_" + std::to_string(nCont);
            out << YAML::Key << contSeq << YAML::Value;
            //contact.first, the moveit Contact obj
            out << YAML::BeginMap;
                out << YAML::Key << "MoveItContact" << YAML::Value << YAML::BeginMap;
                    out << YAML::Key << "body_name_1";
                    out << YAML::Value << contact.first.body_name_1;
                    out << YAML::Key << "body_name_2";
                    out << YAML::Value << contact.first.body_name_2;
                    out << YAML::Key << "body_type_1";
                    out << YAML::Value << contact.first.body_type_1;
                    out << YAML::Key << "body_type_2";
                    out << YAML::Value << contact.first.body_type_2;
                    out << YAML::Key << "depth";
                    out << YAML::Value << contact.first.depth;
                    //TODO print these two eigen vect
                    //out << YAML::Key << "normal";
                    //out << YAML::Value << contact.first.normal;
                    //out << YAML::Key << "pos";
                    //out << YAML::Value << contact.first.pos;
                    out << YAML::EndMap;

                //contact.second, the jointstates map
                out << YAML::Key << "JointStates" << YAML::Value << YAML::BeginMap;
                for (const auto &joint : contact.second) {
                    out << YAML::Key << joint.first;
                    out << YAML::Value << YAML::Flow << joint.second; //vector of double is emit like Seq
                }
                out << YAML::EndMap;
                   
            out << YAML::EndMap;
            nCont++;
        }
        out << YAML::EndMap;
        out << YAML::Newline << YAML::Newline; //double to insert a blanck line between tips pair
    }
    out << YAML::EndMap;
    return out.c_str();
}


//TODO this function will be then in another part, it is not an "offline" part
void ROSEE::MoveItCollider::parseYaml ( std::string filename ){
    
    std::map < std::pair < std::string, std::string >, std::map < std::string, JointStates> > pinchParsedMap; 
    YAML::Node node = YAML::LoadFile(filename);
        
    for(YAML::const_iterator tipPair = node.begin(); tipPair != node.end(); ++tipPair) {
        std::pair <std::string, std::string> tipNames = tipPair->first.as<std::pair<std::string, std::string>>();
        auto insResult = pinchParsedMap.insert ( std::make_pair( tipNames, std::map<std::string, JointStates> () ) );
        
        //TODO check if new insertion, for security reason
        if (!insResult.second) {
            //PAIR already present, some error with the yaml file
        }
        
        for ( YAML::const_iterator setElem = tipPair->second.begin(); setElem != tipPair->second.end(); ++setElem) {
            
            for(YAML::const_iterator cont = setElem->second.begin(); cont != setElem->second.end(); ++cont) {
                //cont can be the map MoveItContact or JointStates
                
                if (cont->first.as<std::string>().compare ("JointStates") == 0 ) {
                    
                    JointStates jointMap = cont->second.as < JointStates >(); 
                    insResult.first->second.insert(
                        std::make_pair (setElem->first.as<std::string>(), jointMap)); //map insert return also the iterator to the added element
                }
            }
        }
    }
    
    //print to check if parse is correct, DEBUG
    for (auto i : pinchParsedMap) {
        std::cout << i.first.first << " " << i.first.second << std::endl;
        for (auto j : i.second) {
            std::cout << "\t" << j.first << ":" << std::endl;
            for (auto y : j.second) {
                std::cout << "\t\t" <<y.first << ": " << y.second.at(0) << std::endl;                
            }
        }
    }
}


