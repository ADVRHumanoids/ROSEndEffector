#include <ROSEndEffector/FindActions.h>

ROSEE::FindActions::FindActions ( std::string robot_description ){
    
    this->robot_description = robot_description;

    //it is a ros param in the launch, take care that also sdrf is read (param: robot_description_semantic)
    robot_model_loader::RobotModelLoader robot_model_loader(robot_description); 
    std::cout << "AAAAAAAAAAAAAAAAA \n " << robot_description << "BBBBBBBBBBb" << std::endl;
    kinematic_model = robot_model_loader.getModel();
    
    std::cout << "look for fingertips... " << std::endl;
    lookForFingertips();
    lookJointsTipsCorrelation();
    
    std::cout << "PARSING with MOVEIT PRINTS RESULT START ***********************************************************************" << std::endl;
    printFingertipLinkNames();
    printJointsOfFingertips();
    printFingertipsOfJoints();
    std::cout << "PARSING with MOVEIT PRINTS RESULT END ***********************************************************************" << std::endl << std::endl;

}

//TODO shoudl be in parser
std::string ROSEE::FindActions::getHandName() {      
    return kinematic_model->getName();
}


std::pair <  std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >, 
             std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak > >
             ROSEE::FindActions::findPinch ( std::string path2saveYaml ){
    
    std::map < std::pair <std::string, std::string> , ActionPinchStrong > mapOfPinches = checkCollisions();
    
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak > mapOfWeakPinches;
    fillNotCollidingTips(&mapOfWeakPinches, &mapOfPinches);
    
    checkWhichTipsCollideWithoutBounds ( &mapOfWeakPinches ) ;

    if (mapOfWeakPinches.size() != 0 ){
        checkDistances (&mapOfWeakPinches) ;
    }
    
    
    /// EMITTING PART ................
    if (mapOfPinches.size() == 0 ) {  //print if no collision at all
        //Remove here after checking pinches further with method said
        std::cout << "WARNING: I found no collisions between tips. Are you sure your hand"
            << " has some fingertips that collide? If yes, check your urdf/srdf, or"
            << " set a bigger value in N_EXP_COLLISION. I am creating a empty file" << std::endl;
    } 
    
    ROSEE::YamlWorker yamlWorker(kinematic_model->getName(), path2saveYaml);
    
    std::map < std::set <std::string> , ActionPrimitive* > mapForWorker;
    for (auto& it : mapOfPinches) {  // auto& and not auto alone!

        ActionPrimitive* pointer = &(it.second);
        std::set < std::string > keys ;
        keys.insert (it.first.first) ;
        keys.insert (it.first.second) ;
        mapForWorker.insert (std::make_pair ( keys, pointer ) );                
    }

    
    yamlWorker.createYamlFile(mapForWorker, "pinchStrong");
    
    if (mapOfWeakPinches.size() == 0 ) { 
        std::cout << "WARNING: I found no weak pinches. This mean that some error happened or that" <<
        " all the tips collide each other for at least one hand configuration. I am creating a empty file" << std::endl;
        
    } 
        
    mapForWorker.clear();
    for (auto& it : mapOfWeakPinches) {  // auto& and not auto alone!

        ActionPrimitive* pointer = &(it.second);
        std::set < std::string > keys ;
        keys.insert (it.first.first) ;
        keys.insert (it.first.second) ;
        mapForWorker.insert (std::make_pair ( keys, pointer ) );                
    }

    yamlWorker.createYamlFile(mapForWorker, "pinchWeak");
    
    
    return std::make_pair(mapOfPinches, mapOfWeakPinches);
}

std::map <std::string, ROSEE::ActionTrig> ROSEE::FindActions::findTrig ( ROSEE::ActionType actionType,
    std::string path2saveYaml) {
    

    std::map <std::string, ActionTrig> trigMap;
    
    switch (actionType) {
    case ROSEE::ActionType::Trig : {
        trigMap = trig();
        break;
    }
    case ROSEE::ActionType::TipFlex : {
        trigMap = tipFlex();
        break;
    }
    case ROSEE::ActionType::FingFlex : {
        trigMap = fingFlex();
        break;
    }
    default: {
        //errror
    }
    }
    
    //for involvedJoints. Ok here because I know that for the trigs, a non setted joint is 
    //a joint which is in a default position
    for (auto & mapEl : trigMap) {
        unsigned int iJoint = 0;
        std::vector <bool> jointsInvolved;
        for ( auto joint : mapEl.second.getActionState() ) {
            jointsInvolved.push_back(false);
            for (auto dof : joint.second) {
                if (dof != DEFAULT_JOINT_POS){
                    jointsInvolved.at(iJoint) = true;
                    break;
                }
            }            
            iJoint++;
        }
        mapEl.second.setJointsInvolved (jointsInvolved);
    }

    std::map < std::set <std::string> , ActionPrimitive* > mapForWorker;

    for (auto& it : trigMap) {  // auto& and not auto alone!

        ActionPrimitive* pointer = &(it.second);
        std::set < std::string > keys ;
        keys.insert (it.first) ;
        mapForWorker.insert (std::make_pair ( keys, pointer ) );
    }
    
    ROSEE::YamlWorker yamlWorker(kinematic_model->getName(), path2saveYaml);
    yamlWorker.createYamlFile(mapForWorker, trigMap.begin()->second.getName());

    return trigMap;
}  

std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong > ROSEE::FindActions::checkCollisions () {
        
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong > mapOfPinches;
    
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

            //for each collision with this joints state...
            for (auto cont : collision_result.contacts){
                
                //store joint states
                JointStates jointStates = getConvertedJointStates(&kinematic_state);

                //moveit contacts is a map between a pair (2 strings with link names) and a vector of Contact object ?? I don't know why the contact is a vector, I have always find only one element            
                logCollision << "Collision between " << cont.first.first << " and " << 
                                                    cont.first.second << std::endl;                                

                for (auto contInfo : cont.second){ 
                    logCollision << "\tWith a depth of contact: " << contInfo.depth;
                }

                std::vector<bool> jointsInvolved = setOnlyDependentJoints(cont.first, &jointStates);
                
                //create the actionPinch
                ActionPinchStrong pinch (cont.first, jointStates, cont.second.at(0) );
                pinch.setJointsInvolved(jointsInvolved);
                auto itFind = mapOfPinches.find ( cont.first );
                if ( itFind == mapOfPinches.end() ) {
                    //if here, we have to create store the new created action
                    mapOfPinches.insert ( std::make_pair (cont.first, pinch) );
                    logCollision << ", NEW INSERTION";

                } else { //Check if it is the best depth among the found collision among that pair
                    if (itFind->second.insertActionState( jointStates, cont.second.at(0)) ) {
                         logCollision << ", NEW INSERTION";
                    }
                }
                logCollision << std::endl;
                logCollision << jointStates;
            }
           // std::cout << logCollision.str() << std::endl;
        }            
    }
    
    return mapOfPinches;
}


void ROSEE::FindActions::checkDistances (std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* mapOfWeakPinches) {
        
    robot_state::RobotState kinematic_state(kinematic_model);
        
    for (int i = 0; i < N_EXP_DISTANCES; i++){
        
        kinematic_state.setToRandomPositions();

        //for each pair remaining in notCollidingTips, check if a new min distance is found
        for (auto &mapEl : *mapOfWeakPinches) { 
            
                            // restore all joint pos
            JointStates jointStatesWeak = getConvertedJointStates(&kinematic_state);
            
            std::vector<bool> jointsInvolved = setOnlyDependentJoints(mapEl.first, &jointStatesWeak);
            
            Eigen::Affine3d tip1Trasf = kinematic_state.getGlobalLinkTransform(mapEl.first.first);
            Eigen::Affine3d tip2Trasf = kinematic_state.getGlobalLinkTransform(mapEl.first.second);
            double distance = (tip1Trasf.translation() - tip2Trasf.translation() ) .norm() ;
                                
            mapEl.second.insertActionState( jointStatesWeak, distance ) ;
            mapEl.second.setJointsInvolved(jointsInvolved);
        }
            
    }

}


std::vector<bool> ROSEE::FindActions::setOnlyDependentJoints(
    std::pair < std::string, std::string > tipsNames, JointStates *jStates) {
    
    std::vector<bool> jointsInvolved (jStates->size(), true); 
    
    unsigned int iJoint = 0;
    for (auto &js : *jStates) { //for each among ALL joints
         
        /** other way around, second is better?
        std::vector <std::string> jointOfTips1 = jointsOfFingertipMap.at(tipsNames.first);
        std::vector <std::string> jointOfTips2 = jointsOfFingertipMap.at(tipsNames.second);
        
        // if the joint is not linked with neither of the two colliding tips...
        if ( std::find( jointOfTips1.begin(), jointOfTips1.end(), js.first) == jointOfTips1.end() &&
             std::find( jointOfTips2.begin(), jointOfTips2.end(), js.first) == jointOfTips2.end() ) {
              
            std::fill ( js.second.begin(), js.second.end(), DEFAULT_JOINT_POS);   
        
            IF USE THIS JOINTINVOLVED REMEMBER
        }
        */
        
        std::vector < std::string> tips = fingertipsOfJointMap.at(js.first); //the tips of the joint
        
        //check if the two tips that collide are among the ones that the joint moves
        if (std::find (tips.begin(), tips.end(), tipsNames.first) == tips.end() &&
            std::find (tips.begin(), tips.end(), tipsNames.second) == tips.end() ) {
            // not dependant, set to default the position
            std::fill ( js.second.begin(), js.second.end(), DEFAULT_JOINT_POS); 
            jointsInvolved.at(iJoint) = false;
        }
        
        iJoint ++;
    } 
    
    return jointsInvolved;
    
}







/// trig is the action of closing a SINGLE finger towards the palm
/// to know the joint direction, the position is set to the limit which is different from 0
// nikos easy solution: go in the direction of the max range. All hands have more range of motion
// in the flexion respect to extension (as human finger). NOT valid for other motion, like finger spread or
// thumb addition/abduction. PROBLEM IS: what is the default position????? WE have to assume 0?
/// if a joint is continuos, it is excluded from the trig action. (because I cant think about a continuos 
/// joint that is useful for a trig action, but can be present in theory)
std::map <std::string, ROSEE::ActionTrig> ROSEE::FindActions::trig() {

    std::map <std::string, ActionTrig> trigMap;

    for (auto mapEl : fingertipsOfJointMap) {
        
        if (mapEl.second.size() != 1) { //the joint must move ONLY a fingertip
            continue;
        }
        
        if ( checkIfContinuosJoint(mapEl.first) == true ) {
            continue; //we dont want to use a continuos joint for the trig
        }

        /// Go in the max range 
        double trigMax = getBiggestBound(mapEl.first) ;

        ActionTrig action ("trig", ActionType::Trig);
        action.setLinkInvolved (mapEl.second.at(0)) ;

        // mapEl.second.at(0) : sure to have only 1 element for the if before
        insertJointPosForTrigInMap(trigMap, action, mapEl.first, trigMax); 

    }
    
    return trigMap;
}

/** We start from each tip. Given a tip, we look for all the joints that move this tip. If it has 2 
 * or more joints that move exclusively that tip (we count this number with @getNExclusiveJointsOfTip() ), 
 * we say that a tipFlex is possible. If not, we cant move the tip indepently from the rest of the 
 * finger, so we have a trig action (if @getNExclusiveJointsOfTip() == 1 ) or 
 * nothing (if @getNExclusiveJointsOfTip() == 0) 
 * If @getNExclusiveJointsOfTip() >= 2, starting from the tip, we explore the parents joints, 
 * until we found the first actuated joint. This one will be @theInterestingJoint which pose of we must 
 * set. All the other joints (actuated) will have the default position (if no strange errors).
 */
std::map <std::string, ROSEE::ActionTrig> ROSEE::FindActions::tipFlex() {
    
    std::map <std::string, ROSEE::ActionTrig> tipFlexMap;
    
    for (auto mapEl : jointsOfFingertipMap) {
        
        if (getNExclusiveJointsOfTip ( mapEl.first ) < 2 ) { 
        //if so, we have a simple trig (or none if 0) and not also a tip/finger flex
            continue;
        }
        
        // starting from the tip, we explore the parents joint, until we found the first actuated (and
        // not continuos). This ones will be the interesting joint 
        // constant is the data pointed and not the pointer itself
        const moveit::core::LinkModel* linkModel = kinematic_model->getLinkModel(mapEl.first);

        while ( linkModel->getParentJointModel()->getMimic() != NULL || 
                linkModel->parentJointIsFixed() ||
                linkModel->getParentJointModel()->isPassive() || 
                checkIfContinuosJoint(linkModel->getParentJointModel()) ) {
            
            //an active and not continuos joint is not any of these condition.
            //passive is an attribute of the joint in the srdf, so it may be not setted (default is not passive)
            //, so we need also the getMimic == NULL (ie: an actuated joint dont mimic anything)
            //WARNING these 4 conditions should be enough I think
            
            linkModel = linkModel->getParentLinkModel();
        }
        
        if (linkModel == NULL ) {
            std::cout << "[FATAL ERROR] Strange Error, jointsOfFingertipMap, " << 
                "fingertipsOfJointMap and/or other things may have been built badly" << std::endl;
        }
        
        std::string theInterestingJoint = linkModel->getParentJointModel()->getName();
        double tipFlexMax = getBiggestBound(linkModel->getParentJointModel()) ;
        

        ActionTrig action ("tipFlex", ActionType::TipFlex);
        action.setLinkInvolved (mapEl.first) ;
        if (! insertJointPosForTrigInMap(tipFlexMap, action, theInterestingJoint, tipFlexMax) ) {
            //if here, we have updated the joint position for a action that was already present in the map.
            //this is ok for normal trig because more joints are included in the action, but for the
            //tipflex and fingflex for definition only a joint is involved (the active one nearer to the tip)
            std::cout << "[FATAL ERROR]: Inserting in tipFlexMap a tip already present??" << std::endl;
        }
    }
    
    return tipFlexMap;
}

/** We start from each tip. Given a tip, we check if @getNExclusiveJointsOfTip() >= 2 (see @tipFlex() function).
 *  If so, we continue exploring the chain from the tip going up through the parents. We stop when a parent has
 *  more than 1 joint as child. This means that the last link is the first of the finger. Meanwhile we have 
 *  stored the actuated, not continuos joint (in @joint) that we were founding along the chain. The last stored
 *  is exaclty @theInterestingJoint, which pose of we must set.
 *  All the other joints (actuated) will have the default position (if no strange errors).
 */
std::map <std::string, ROSEE::ActionTrig> ROSEE::FindActions::fingFlex() {
    
    std::map <std::string, ROSEE::ActionTrig> fingFlexMap;
    
    for (auto mapEl : jointsOfFingertipMap) {
        
        if (getNExclusiveJointsOfTip ( mapEl.first ) < 2 ) { 
        //if so, we have a simple trig (or none if 0) and not also a tip/finger flex
            continue;
        }
        
        //explore parent links until we found a link with more than 1 child joint
        const moveit::core::LinkModel* linkModel = kinematic_model->getLinkModel(mapEl.first);
        const moveit::core::JointModel* joint;

        // we stop when the link has more than 1 joint: so linkModel will be the parent of the first 
        // link of the finger group and in joint we have stored the actuated (not continuos) 
        // child joint most near to linkModel
        while ( (linkModel != NULL) && (linkModel->getChildJointModels().size() < 2) ) {
            
            //if the parent joint is an actuated (not cont) joint, store it (or overwrite the previous stored)
            if ( linkModel->getParentJointModel()->getMimic() == NULL && 
                 (!linkModel->parentJointIsFixed()) &&
                 (!linkModel->getParentJointModel()->isPassive()) &&
                 (!checkIfContinuosJoint(linkModel->getParentJointModel() ))  ) {
                
                joint = linkModel->getParentJointModel();
            }
            
            linkModel = linkModel->getParentLinkModel();
        }
        
        std::string theInterestingJoint = joint->getName();
        double fingFlexMax = getBiggestBound(joint) ;

        ActionTrig action ("fingFlex", ActionType::FingFlex);
        action.setLinkInvolved (mapEl.first) ;
        if (! insertJointPosForTrigInMap(fingFlexMap, action, theInterestingJoint, fingFlexMax) ) {
            //if here, we have updated the joint position for a action that was already present in the map.
            //this is ok for normal trig because more joints are included in the action, but for the
            //tipflex and fingflex for definition only a joint is involved (the active one farther from the tip
            //but still inside the finger)
            std::cout << "[FATAL ERROR]: Inserting in fingFlexMap a tip already present??" << std::endl;
        }
    }
    return fingFlexMap;
}



bool ROSEE::FindActions::insertJointPosForTrigInMap ( std::map <std::string, ActionTrig>& trigMap, 
    ROSEE::ActionTrig action, std::string jointName, double trigValue) {
    
    auto itMap = trigMap.find(action.getLinkInvolved());
        if ( itMap == trigMap.end() ) {
            //still no action for this tip in the map
            
            JointStates js;
            for (auto it : kinematic_model->getActiveJointModels()){
                std::vector <double> jPos (it->getVariableCount(), DEFAULT_JOINT_POS);
                js.insert ( std::make_pair ( it->getName(), jPos ));
            }
            
            //HACK at(0) because 1dof joint
            js.at ( jointName ).at(0) = trigValue;

            action.setActionState(js);
            trigMap.insert ( std::make_pair ( action.getLinkInvolved(), action ) );
    
            return true;

        } else {
            //action already created, but we have to modify the position of a joint
            //itMap->second is an iterator to the already present element
            JointStates js = itMap->second.getActionState();
            //HACK at(0) because 1dof joint
            js.at (jointName).at(0) = trigValue;
            itMap->second.setActionState(js);
            
            return false;
        }
}




ROSEE::JointStates ROSEE::FindActions::getConvertedJointStates(const robot_state::RobotState* kinematic_state) {
    
    JointStates js;
    for (auto actJ : kinematic_model->getActiveJointModels()){
        //joint can have multiple pos, so double*, but we want to store in a vector 
        const double* pos = kinematic_state->getJointPositions(actJ); 
        unsigned posSize = sizeof(pos) / sizeof(double);
        std::vector <double> vecPos(pos, pos + posSize);
        js.insert(std::make_pair(actJ->getName(), vecPos));
    }
    return js;
}


void ROSEE::FindActions::fillNotCollidingTips ( 
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* mapOfWeakPinches,
    const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >* mapOfPinches) {
    

    // first fill mapOfWeakPinches with all pairs ...
    for (auto tip1 : fingertipNames) {
        for (auto tip2 : fingertipNames) { 
            
            // important to put in order in the pair, then in the set thing are autoordered
            if (tip1 < tip2) {
                mapOfWeakPinches->insert (std::make_pair (std::make_pair (tip1, tip2), ActionPinchWeak(tip1, tip2)));
                
            } else if (tip1 > tip2) {
                mapOfWeakPinches->insert (std::make_pair (std::make_pair (tip2, tip1), ActionPinchWeak(tip2, tip1)));
            }    
        }
    }  
    
    // ... then remove all the colliding tips
    for (const auto mapEl : *mapOfPinches){
        mapOfWeakPinches->erase(mapEl.first);
    }
}

void ROSEE::FindActions::removeBoundsOfNotCollidingTips ( 
    const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* mapOfWeakPinches,
    robot_model::RobotModelPtr kinematic_model_noBound) {

    for (auto mapEl : *mapOfWeakPinches ) {
        
        for ( auto &joint : jointsOfFingertipMap.at (mapEl.first.first) ) { //for each joint of first tip...
            
            auto jointModel = kinematic_model_noBound ->getJointModel(joint);
            auto type = jointModel->getType() ;
            if (type == moveit::core::JointModel::REVOLUTE ) {
                //at(0) because we are sure to have 1 dof being revolute
                auto bound = jointModel->getVariableBounds().at(0);
                bound.max_position_ = EIGEN_PI;
                bound.min_position_ = -EIGEN_PI;
                //at(0) because we are sure to have 1 dof being revolute
                jointModel->setVariableBounds ( jointModel->getVariableNames().at(0), bound );
                
            } else if ( type == moveit::core::JointModel::PRISMATIC ) {
                // we cant set infinite here... lets double the limits?
                std::cout << "[WARNING FINDACTIONS WEAKPINCHES] Im doubling the bounds for your prismatic joint " 
                    << "but I am not sure it is enough to make the tips colliding to find the weak pinches " << std::endl;
                auto bound = jointModel->getVariableBounds().at(0);
                bound.max_position_ *= 2;
                bound.min_position_ *= 2;
                //at(0) because we are sure to have 1 dof being revolute
                jointModel->setVariableBounds ( jointModel->getVariableNames().at(0), bound );
                
            } else {
                    
                std::cout << "[WARNING] Why are you using a type " 
                    << kinematic_model_noBound ->getJointModel(joint)->getType()
                    << " joint? Code not ready to temporarily delete the multiple dof bounds"
                    << " in the working done to find the weak pinches " << std::endl << std::endl;
                
                continue;
            }
        }
        
        for ( auto joint : jointsOfFingertipMap.at (mapEl.first.second) ) { //for each joint of second tip...
            
            auto jointModel = kinematic_model_noBound ->getJointModel(joint);
            auto type = jointModel->getType() ;
            if (type == moveit::core::JointModel::REVOLUTE ) {
                //at(0) because we are sure to have 1 dof being revolute
                auto bound = jointModel->getVariableBounds().at(0);
                bound.max_position_ = EIGEN_PI;
                bound.min_position_ = -EIGEN_PI;
                //at(0) because we are sure to have 1 dof being revolute
                jointModel->setVariableBounds ( jointModel->getVariableNames().at(0), bound );
                
            } else if ( type == moveit::core::JointModel::PRISMATIC ) {
                // we cant set infinite here... lets double the limits?
                std::cout << "[WARNING FINDACTIONS WEAKPINCHES] Im doubling the bounds for your prismatic joint " 
                    << "but I am not sure it is enough to make the tips colliding to find the weak pinches " << std::endl;
                auto bound = jointModel->getVariableBounds().at(0);
                bound.max_position_ *= 2;
                bound.min_position_ *= 2;
                //at(0) because we are sure to have 1 dof being revolute
                jointModel->setVariableBounds ( jointModel->getVariableNames().at(0), bound );
                
            } else {
                    
                std::cout << "[WARNING] Why are you using a type " 
                    << kinematic_model_noBound ->getJointModel(joint)->getType()
                    << " joint? Code not ready to temporarily delete the multiple dof bounds"
                    << " in the working done to find the weak pinches " << std::endl << std::endl;
                
                continue;
            }
            
        }
        
    }
}


void ROSEE::FindActions::checkWhichTipsCollideWithoutBounds (
    std::map < std::pair <std::string, std::string>, ROSEE::ActionPinchWeak >* mapOfWeakPinches ) {
    
    // first, we temporarily remove bounds of joints linked to the non-colliding tips, and we check for collision
    // if some collision are found, so their movements make them go towards each other, (also with the bounds) but
    // the bounds do not permit them to touch. This is a weak pinch. If they do not collide even without bounds,
    // this means that they never go towards each other, so this is not a pinch at AllowedCollisionMatrix
    // create new object so we dont modify the original kinematic_model
    
    //it is a ros param in the launch, take care that also sdrf is read (param: robot_description_semantic)
    robot_model_loader::RobotModelLoader robot_model_loader(robot_description); 
    robot_model::RobotModelPtr kinematic_model_noBound = robot_model_loader.getModel();
    removeBoundsOfNotCollidingTips (mapOfWeakPinches, kinematic_model_noBound );
    
        collision_detection::AllowedCollisionMatrix acm;
    acm.setEntry(kinematic_model_noBound->getLinkModelNames(), kinematic_model_noBound->getLinkModelNames(), true); 
    //true==not considered collisions
    for( auto  it : *mapOfWeakPinches) {
        //we want to look for collision only on the pair inside the map
        acm.setEntry(it.first.first, it.first.second, false); //false == considered collisions   
    }

    planning_scene::PlanningScene planning_scene(kinematic_model_noBound);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;  //set to compute collisions
    collision_request.max_contacts = 1000;

    robot_state::RobotState kinematic_state(kinematic_model_noBound);

    // similar to checkcollisions here, but we dont want to store anything, only check if collision happen
    std::set < std::pair<std::string, std::string> > collidingTips ;
    for (int i = 0; i < N_EXP_COLLISION; i++){
        collision_result.clear();
        kinematic_state.setToRandomPositions();
        planning_scene.checkSelfCollision(collision_request, collision_result, kinematic_state, acm);
        
        for (auto cont : collision_result.contacts){
            //moveit contacts is a map between a pair (2 strings with link names) and a vector of Contact object 
            collidingTips.insert ( std::make_pair (cont.first.first, cont.first.second ) );   
        }
    }

    //erase from weak map the not colliding tips 
    for (auto mapEl = mapOfWeakPinches->cbegin(); mapEl != mapOfWeakPinches->cend() ; /*no increment*/ ) {
        if (collidingTips.count(mapEl->first) == 0 ) {
            mapOfWeakPinches->erase(mapEl++);
        } else { 
            ++mapEl;
        }
    }
}
