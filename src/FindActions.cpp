#include <ROSEndEffector/FindActions.h>

ROSEE::FindActions::FindActions ( std::shared_ptr < ROSEE::ParserMoveIt > parserMoveIt ){

    this->parserMoveIt = parserMoveIt;
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
        std::cout << "[FINDACTIONS::" << __func__ << "]: I found no collisions between tips. Are you sure your hand"
            << " has some fingertips that collide? If yes, check your urdf/srdf, or"
            << " set a bigger value in N_EXP_COLLISION" << std::endl;
            
    } else {
    
        ROSEE::YamlWorker yamlWorker;
        std::map < std::set <std::string> , ActionPrimitive* > mapForWorker;
        
        for (auto& it : mapOfPinches) {  // auto& and not auto alone!

            ActionPrimitive* pointer = &(it.second);
            std::set < std::string > keys ;
            keys.insert (it.first.first) ;
            keys.insert (it.first.second) ;
            mapForWorker.insert (std::make_pair ( keys, pointer ) );                
        }

        yamlWorker.createYamlFile(mapForWorker, "pinchStrong", path2saveYaml);
    }
    
    if (mapOfWeakPinches.size() == 0 ) { 
        std::cout << "[FINDACTIONS::" << __func__ << "]: I found no weak pinches. This mean that some error happened or that" <<
        " all the tips pairs collide with each other for at least one hand configuration." << std::endl;
        
    } else {
        
        ROSEE::YamlWorker yamlWorker;
        std::map < std::set <std::string> , ActionPrimitive* > mapForWorker;  
        
        for (auto& it : mapOfWeakPinches) {  // auto& and not auto alone!

            ActionPrimitive* pointer = &(it.second);
            std::set < std::string > keys ;
            keys.insert (it.first.first) ;
            keys.insert (it.first.second) ;
            mapForWorker.insert (std::make_pair ( keys, pointer ) );                
        }

        yamlWorker.createYamlFile(mapForWorker, "pinchWeak", path2saveYaml);
    }
    
    return std::make_pair(mapOfPinches, mapOfWeakPinches);
}

std::map <std::string, ROSEE::ActionTrig> ROSEE::FindActions::findTrig ( ROSEE::ActionPrimitive::Type actionType,
    std::string path2saveYaml) {
    

    std::map <std::string, ActionTrig> trigMap;
    
    switch (actionType) {
    case ROSEE::ActionPrimitive::Type::Trig : {
        trigMap = trig();
        break;
    }
    case ROSEE::ActionPrimitive::Type::TipFlex : {
        trigMap = tipFlex();
        break;
    }
    case ROSEE::ActionPrimitive::Type::FingFlex : {
        trigMap = fingFlex();
        break;
    }
    default: {
        std::cout << "[ERROR FINDACTIONS::" << __func__ << "]: Passing as argument a action type which is not a type of trig. " << std::endl 
        << "I am returing an empty map" << std::endl;
        return trigMap;
    }
    }
    
    if (trigMap.size() == 0 ) { //if so, no sense to continue
        return trigMap;
    }
    
    //for involvedJoints. Ok here because I know that for the trigs, a non setted joint is 
    //a joint which is in a default position
    for (auto & mapEl : trigMap) {

        ROSEE::JointsInvolvedCount jointsInvolvedCount;
        for ( auto joint : mapEl.second.getJointPos() ) {
            jointsInvolvedCount.insert (std::make_pair(joint.first, 0));
            for (auto dof : joint.second) {
                if (dof != DEFAULT_JOINT_POS){
                    jointsInvolvedCount.at(joint.first) = 1;
                    break;
                }
            }            
        }
        mapEl.second.setJointsInvolvedCount (jointsInvolvedCount);
    }
        
    std::map < std::set <std::string> , ActionPrimitive* > mapForWorker;

    for (auto& it : trigMap) {  // auto& and not auto alone!

        ActionPrimitive* pointer = &(it.second);
        std::set < std::string > keys ;
        keys.insert (it.first) ;
        mapForWorker.insert (std::make_pair ( keys, pointer ) );
    }
    
    ROSEE::YamlWorker yamlWorker;
    yamlWorker.createYamlFile(mapForWorker, trigMap.begin()->second.getName(), path2saveYaml);

    return trigMap;
}  


std::map <std::string, ROSEE::ActionMoreTips> ROSEE::FindActions::findMoreTips(unsigned int nFinger, std::string path2saveYaml) {
    
    std::map <std::string, ROSEE::ActionMoreTips> mapOfMoreTips;
    
    if (nFinger == 1) {
        std::cout << "[ERROR FINDACTIONS::" << __func__ << "]  with 1 finger, you are looking for a ActionTrig, "
            << "and not a ActionMoreTips. Returning an empty map" << std::endl;
        return mapOfMoreTips;
    }
    
    if (nFinger > parserMoveIt->getNFingers() ) {
        std::cout << "[ERROR FINDACTIONS::" << __func__ << "]  I can not find an action which moves " << nFinger << 
        " fingers if the hand has only " << parserMoveIt->getNFingers() << " fingers. Returning an empty map" << std::endl;
        return mapOfMoreTips;
    }
       
    std::string actionName = "moreTips_" + std::to_string(nFinger); //action name same for each action

    for (auto mapEl : parserMoveIt->getFingertipsOfJointMap() ) {
        
        if (mapEl.second.size() != nFinger ) {
            continue;
        }
        
        std::vector<double> furtherPos = parserMoveIt->getBiggerBoundFromZero(mapEl.first);
        std::vector<double> nearerPos = parserMoveIt->getSmallerBoundFromZero(mapEl.first);
        
        //create and initialize JointPos map
        JointPos jpFar;
        for (auto it : parserMoveIt->getActiveJointModels()){
            std::vector <double> jPos (it->getVariableCount(), DEFAULT_JOINT_POS);
            jpFar.insert ( std::make_pair ( it->getName(), jPos ));
        }
        JointPos jpNear = jpFar;
        
        jpFar.at ( mapEl.first ) = furtherPos;
        jpNear.at ( mapEl.first ) = nearerPos;
        
        ActionMoreTips action (actionName, mapEl.second, mapEl.first, jpFar, jpNear);
        //"convert" vector to set 
        std::set <std::string> setFingers;
        setFingers.insert (mapEl.second.begin(), mapEl.second.end() );
        
        mapOfMoreTips.insert (std::make_pair(mapEl.first, action));
    }
    
    //// EMITTING
    if (mapOfMoreTips.size() == 0 ) {
        std::cout << "[FINDACTIONS::" << __func__ << "]  no moreTips with " << nFinger << " found" << std::endl;
        return mapOfMoreTips;
    }
    
    std::map < std::set <std::string> , ActionPrimitive* > mapForWorker;

    for (auto& it : mapOfMoreTips) {  // auto& and not auto alone!

        ActionPrimitive* pointer = &(it.second);
        std::set<std::string> set;
        set.insert (it.first);
        mapForWorker.insert (std::make_pair ( set, pointer ) );
    }
    
    ROSEE::YamlWorker yamlWorker;
    yamlWorker.createYamlFile(mapForWorker, actionName, path2saveYaml);
    
    return mapOfMoreTips;
}


std::map<std::set<std::string>, ROSEE::ActionMultiplePinchStrong> ROSEE::FindActions::findMultiplePinch(unsigned int nFinger, std::string path2saveYaml,
                                                                                                        bool strict ) {
    
    std::map<std::set<std::string>, ROSEE::ActionMultiplePinchStrong> multiplePinchMap;
    if (nFinger < 3 ) {
        std::cerr << "[ERROR " << __func__ << "] for this find pass at least 3 as number " <<
        " of fingertips for the pinch" << std::endl;
        return multiplePinchMap;
    }
    
    multiplePinchMap = checkCollisionsForMultiplePinch(nFinger, strict);
        
    //// EMITTING YAML
    if (multiplePinchMap.size() == 0 ) {
        return multiplePinchMap;
    }
    std::map < std::set <std::string> , ActionPrimitive* > mapForWorker;

    for (auto& it : multiplePinchMap) {  // auto& and not auto alone!

        ActionPrimitive* pointer = &(it.second);
        mapForWorker.insert (std::make_pair ( it.first, pointer ) );
    }
    
    ROSEE::YamlWorker yamlWorker;
    yamlWorker.createYamlFile(mapForWorker, multiplePinchMap.begin()->second.getName(), path2saveYaml);
    
    return multiplePinchMap;
}



/*********************************** PRIVATE FUNCTIONS ***********************************************************************/
/**************************************** PINCHES ***********************************************************************/

std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong > ROSEE::FindActions::checkCollisions () {
        
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong > mapOfPinches;
    
    planning_scene::PlanningScene planning_scene ( parserMoveIt->getRobotModel() );
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;  //set to compute collisions
    collision_request.max_contacts = 1000;
    
    // Consider only collisions among fingertips 
    // If an object pair does not appear in the acm, it is assumed that collisions between those 
    // objects is no tolerated. So we must fill it with all the nonFingertips
    
    collision_detection::AllowedCollisionMatrix acm;
    acm.setEntry(parserMoveIt->getRobotModel()->getLinkModelNames(), 
                 parserMoveIt->getRobotModel()->getLinkModelNames(), true); //true==not considered collisions
    acm.setEntry(parserMoveIt->getFingertipNames(), 
                 parserMoveIt->getFingertipNames(), false); //false== considered collisions

    robot_state::RobotState kinematic_state(parserMoveIt->getRobotModel());
        
    for (int i = 0; i < N_EXP_COLLISION; i++){
        
        std::stringstream logCollision;
        collision_result.clear();
        kinematic_state.setToRandomPositions();
        planning_scene.checkSelfCollision(collision_request, collision_result, kinematic_state, acm);
        
        if (collision_result.collision) { 
            
            //for each collision with this joints state...
            for (auto cont : collision_result.contacts){
                
                //store joint states
                JointPos jointPos = getConvertedJointPos(&kinematic_state);

                //moveit contacts is a map between a pair (2 strings with link names) and a vector of Contact object ?? I don't know why the contact is a vector, I have always find only one element            
                logCollision << "[FINDACTIONS::" << __func__ << "] Collision between " << cont.first.first << " and " << 
                                                    cont.first.second << std::endl;                                

                for (auto contInfo : cont.second){ 
                    logCollision << "\tWith a depth of contact: " << contInfo.depth;
                }

                JointsInvolvedCount jointsInvolvedCount = setOnlyDependentJoints(cont.first, &jointPos);

                //create the actionPinch
                ActionPinchStrong pinch (cont.first, jointPos, cont.second.at(0) );
                pinch.setJointsInvolvedCount ( jointsInvolvedCount );
                auto itFind = mapOfPinches.find ( cont.first );
                if ( itFind == mapOfPinches.end() ) {
                    //if here, we have to create store the new created action
                    mapOfPinches.insert ( std::make_pair (cont.first, pinch) );
                    logCollision << ", NEW INSERTION";

                } else { //Check if it is the best depth among the found collision among that pair
                    if (itFind->second.insertActionState( jointPos, cont.second.at(0)) ) {
                         logCollision << ", NEW INSERTION";
                    }
                }
                logCollision << std::endl;
                logCollision << jointPos;
            }
           // std::cout << logCollision.str() << std::endl;
        }            
    }
    
    return mapOfPinches;
}


void ROSEE::FindActions::checkDistances (std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* mapOfWeakPinches) {
        
    robot_state::RobotState kinematic_state(parserMoveIt->getRobotModel());
        
    for (int i = 0; i < N_EXP_DISTANCES; i++){
        
        kinematic_state.setToRandomPositions();

        //for each pair remaining in notCollidingTips, check if a new min distance is found
        for (auto &mapEl : *mapOfWeakPinches) { 
            
                            // restore all joint pos
            JointPos jointPosWeak = getConvertedJointPos(&kinematic_state);
            
            JointsInvolvedCount jointsInvolvedCount = setOnlyDependentJoints(mapEl.first, &jointPosWeak);
            
            Eigen::Affine3d tip1Trasf = kinematic_state.getGlobalLinkTransform(mapEl.first.first);
            Eigen::Affine3d tip2Trasf = kinematic_state.getGlobalLinkTransform(mapEl.first.second);
            double distance = (tip1Trasf.translation() - tip2Trasf.translation() ) .norm() ;
                                
            mapEl.second.insertActionState( jointPosWeak, distance ) ;
            mapEl.second.setJointsInvolvedCount ( jointsInvolvedCount );
        }
    }
}


void ROSEE::FindActions::removeBoundsOfNotCollidingTips ( 
    const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* mapOfWeakPinches,
    robot_model::RobotModelPtr kinematic_model_noBound) {

    for (auto mapEl : *mapOfWeakPinches ) {
        
        //for each joint of first tip...
        /// C++ Question: WHY if I use directly parser....at() in the for the string joint is corrupted?
        auto joints = parserMoveIt->getJointsOfFingertipMap().at (mapEl.first.first);
        for ( std::string joint : joints ) { 
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
                std::cout << "[WARNING FINDACTIONS::" << __func__ << "] I am doubling the bounds for your prismatic joint " 
                    << "but I am not sure it is enough to make the tips colliding to find the weak pinches " <<
                    std::endl;
                auto bound = jointModel->getVariableBounds().at(0);
                bound.max_position_ *= 2;
                bound.min_position_ *= 2;
                //at(0) because we are sure to have 1 dof being prismatic
                jointModel->setVariableBounds ( jointModel->getVariableNames().at(0), bound );
                
            } else {
                    
                std::cout << "[FINDACTIONS::" << __func__ << "] Why are you using a type " 
                    << kinematic_model_noBound ->getJointModel(joint)->getType()
                    << " joint? Code not ready to temporarily delete the multiple dof bounds"
                    << " in the working done to find the weak pinches " << std::endl << std::endl;
                
                continue;
            }
        }
        
        //for each joint of second tip...
        auto joints2 = parserMoveIt->getJointsOfFingertipMap().at (mapEl.first.second);
        for ( auto joint : joints2 ) { 
            
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
                std::cout << "[WARNING FINDACTIONS::" << __func__ << "] I am doubling the bounds for your prismatic joint " 
                    << "but I am not sure it is enough to make the tips colliding to find the weak pinches " << std::endl;
                auto bound = jointModel->getVariableBounds().at(0);
                bound.max_position_ *= 2;
                bound.min_position_ *= 2;
                //at(0) because we are sure to have 1 dof being revolute
                jointModel->setVariableBounds ( jointModel->getVariableNames().at(0), bound );
                
            } else {
                    
                std::cout << "[FINDACTIONS::" << __func__ << "] Why are you using a type " 
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
    
    robot_model::RobotModelPtr kinematic_model_noBound = parserMoveIt->getCopyModel();
    
    removeBoundsOfNotCollidingTips (mapOfWeakPinches, kinematic_model_noBound );

    collision_detection::AllowedCollisionMatrix acm;
    acm.setEntry(kinematic_model_noBound->getLinkModelNames(), 
                 kinematic_model_noBound->getLinkModelNames(), true); //true == not considered collisions
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


std::map<std::set<std::string>, ROSEE::ActionMultiplePinchStrong> ROSEE::FindActions::checkCollisionsForMultiplePinch(unsigned int nFinger, bool strict) {
    
    std::map < std::set <std::string> , ROSEE::ActionMultiplePinchStrong > mapOfMultPinches;
    
    unsigned int nMinCollision =  strict ? 
            ROSEE::Utils::binomial_coefficent(nFinger, 2) : (nFinger-1);
    
    planning_scene::PlanningScene planning_scene ( parserMoveIt->getRobotModel() );
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;  //set to compute collisions
    collision_request.max_contacts = 1000;
    
    // Consider only collisions among fingertips 
    // If an object pair does not appear in the acm, it is assumed that collisions between those 
    // objects is no tolerated. So we must fill it with all the nonFingertips
    collision_detection::AllowedCollisionMatrix acm;
    acm.setEntry(parserMoveIt->getRobotModel()->getLinkModelNames(), 
                 parserMoveIt->getRobotModel()->getLinkModelNames(), true); //true==not considered collisions
    acm.setEntry(parserMoveIt->getFingertipNames(), 
                 parserMoveIt->getFingertipNames(), false); //false== considered collisions

    robot_state::RobotState kinematic_state(parserMoveIt->getRobotModel());
        
    for (int i = 0; i < N_EXP_COLLISION_MULTPINCH; i++){
        
        collision_result.clear();
        kinematic_state.setToRandomPositions();
        planning_scene.checkSelfCollision(collision_request, collision_result, kinematic_state, acm);
        
        if (collision_result.contacts.size() >= nMinCollision ) { 
        
            double depthSum = 0;
            std::set <std::string> fingerColliding;
            for (auto cont : collision_result.contacts){
                
                fingerColliding.insert(cont.first.first);
                fingerColliding.insert(cont.first.second);
                depthSum += std::abs(cont.second.at(0).depth);
            }
            
            //eg with 2 collision we can have 4 finger colliding because there are two
            //normal distinct pinch and not a 3-pinch... so we exlude these collisions
            if (fingerColliding.size() != nFinger) {
                continue;
            }
                
            //store joint states
            JointPos jointPos = getConvertedJointPos(&kinematic_state);
            JointsInvolvedCount jointsInvolvedCount = setOnlyDependentJoints(fingerColliding, &jointPos);

            ActionMultiplePinchStrong pinch (fingerColliding, jointPos, depthSum );
            pinch.setJointsInvolvedCount ( jointsInvolvedCount );
            auto itFind = mapOfMultPinches.find ( fingerColliding );
            if ( itFind == mapOfMultPinches.end() ) {
                //if here, we have to create store the new created action
                mapOfMultPinches.insert ( std::make_pair (fingerColliding, pinch) );

            } else { //Check if it is the best depth among the found collision among that pair
                if (itFind->second.insertActionState( jointPos, depthSum ) ) {
                    //print debug
                } else {
                    //pring debug
                }
            }
        }            
    }
    return mapOfMultPinches;
}




/**********************************************  TRIGS ***************************************************************/

std::map <std::string, ROSEE::ActionTrig> ROSEE::FindActions::trig() {

    std::map <std::string, ActionTrig> trigMap;

    for (auto mapEl : parserMoveIt->getFingertipsOfJointMap()) {
        
        if (mapEl.second.size() != 1) { //the joint must move ONLY a fingertip
            continue;
        }
        
        if ( parserMoveIt->checkIfContinuosJoint(mapEl.first) == true ) {
            continue; //we dont want to use a continuos joint for the trig
        }

        /// Go in the max range 
        double trigMax = parserMoveIt->getBiggerBoundFromZero(mapEl.first).at(0) ;

        ActionTrig action ("trig", ActionPrimitive::Type::Trig);
        action.setFingerInvolved (mapEl.second.at(0)) ;

        // mapEl.second.at(0) : sure to have only 1 element for the if before
        insertJointPosForTrigInMap(trigMap, action, mapEl.first, trigMax); 
    }
    
    return trigMap;
}


std::map <std::string, ROSEE::ActionTrig> ROSEE::FindActions::tipFlex() {
    
    std::map <std::string, ROSEE::ActionTrig> tipFlexMap;
    
    for (auto fingerName : parserMoveIt->getFingertipNames() ) {

        if (parserMoveIt->getNExclusiveJointsOfTip ( fingerName, false ) < 2 ) { 
        //if so, we have a simple trig (or none if 0) and not also a tip/finger flex
            continue;
        }

        std::string theInterestingJoint = parserMoveIt->getFirstActuatedParentJoint ( fingerName, false );
        double tipFlexMax = parserMoveIt->getBiggerBoundFromZero ( theInterestingJoint ).at(0) ;
        
        
        ActionTrig action ("tipFlex", ActionPrimitive::Type::TipFlex);
        action.setFingerInvolved (fingerName) ;

        if (! insertJointPosForTrigInMap(tipFlexMap, action, theInterestingJoint, tipFlexMax) ) {
            //if here, we have updated the joint position for a action that was already present in the map.
            //this is ok for normal trig because more joints are included in the action, but for the
            //tipflex and fingflex for definition only a joint is involved (the active one nearer to the tip)
            std::cout << "[FATAL ERROR FINDACTIONS::" << __func__ << "]: Inserting in tipFlexMap a tip already present??" << std::endl 
            << "I am returning a not completely filled map" << std::endl;
            return tipFlexMap;
        }
    }

    return tipFlexMap;
}


std::map <std::string, ROSEE::ActionTrig> ROSEE::FindActions::fingFlex() {
    
    std::map <std::string, ROSEE::ActionTrig> fingFlexMap;
    
    for (auto fingerName : parserMoveIt->getFingertipNames() ) {
        
        if (parserMoveIt->getNExclusiveJointsOfTip ( fingerName, false ) < 2 ) { 
        //if so, we have a simple trig (or none if 0) and not also a tip/finger flex
            continue;
        }

        std::string theInterestingJoint = parserMoveIt->getFirstActuatedJointInFinger ( fingerName );
        double fingFlexMax = parserMoveIt->getBiggerBoundFromZero ( theInterestingJoint ).at(0) ;

        ActionTrig action ("fingFlex", ActionPrimitive::Type::FingFlex);
        action.setFingerInvolved (fingerName) ;
        if (! insertJointPosForTrigInMap(fingFlexMap, action, theInterestingJoint, fingFlexMax) ) {
            //if here, we have updated the joint position for a action that was already present in the map.
            //this is ok for normal trig because more joints are included in the action, but for the
            //tipflex and fingflex for definition only a joint is involved (the active one farther from the tip
            //but still inside the finger)
            std::cout << "[FATAL ERROR FINDACTIONS::" << __func__ << "]: Inserting in fingFlexMap a tip already present??n" << std::endl 
            << "I am returning a not completely filled map" << std::endl;
            return fingFlexMap;
        }
    }
    return fingFlexMap;
}


bool ROSEE::FindActions::insertJointPosForTrigInMap ( std::map <std::string, ActionTrig>& trigMap, 
    ROSEE::ActionTrig action, std::string jointName, double trigValue) {
    
    auto itMap = trigMap.find ( action.getFingerInvolved() );
    if ( itMap == trigMap.end() ) {
        //still no action for this tip in the map

        JointPos jp;
        for (auto it : parserMoveIt->getActiveJointModels()){
            std::vector <double> jPos (it->getVariableCount(), DEFAULT_JOINT_POS);
            jp.insert ( std::make_pair ( it->getName(), jPos ));
        }
        
        //HACK at(0) because 1dof joint
        jp.at ( jointName ).at(0) = trigValue;
 
        action.setJointPos(jp);
        trigMap.insert ( std::make_pair ( action.getFingerInvolved(), action ) );

        return true;

    } else {
        //action already created, but we have to modify the position of a joint
        //itMap->second is an iterator to the already present element
        JointPos jp = itMap->second.getJointPos();
        //HACK at(0) because 1dof joint
        jp.at (jointName).at(0) = trigValue;
        itMap->second.setJointPos(jp);
        
        return false;
    }

}


/**********************************************  SUPPORT FUNCTIONS     ***************************************************************/


ROSEE::JointPos ROSEE::FindActions::getConvertedJointPos(const robot_state::RobotState* kinematic_state) {
    
    JointPos jp;
    for ( auto actJ : parserMoveIt->getActiveJointModels()) {
        //joint can have multiple pos, so double*, but we want to store in a vector 
        const double* pos = kinematic_state->getJointPositions(actJ); 
        unsigned posSize = sizeof(pos) / sizeof(double);
        std::vector <double> vecPos(pos, pos + posSize);
        jp.insert(std::make_pair(actJ->getName(), vecPos));
    }
    return jp;
}


void ROSEE::FindActions::fillNotCollidingTips ( 
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchWeak >* mapOfWeakPinches,
    const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchStrong >* mapOfPinches) {
    
    // first fill mapOfWeakPinches with all pairs ...
    for ( auto tip1 : parserMoveIt->getFingertipNames() )  {
        for ( auto tip2 : parserMoveIt->getFingertipNames() ) { 
            
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




ROSEE::JointsInvolvedCount ROSEE::FindActions::setOnlyDependentJoints(
    std::pair < std::string, std::string > tipsNames, JointPos *jPos) {
    
    JointsInvolvedCount jointsInvolvedCount;
    
    for (auto &jp : *jPos) { //for each among ALL joints
        
        jointsInvolvedCount.insert ( std::make_pair (jp.first, 1) );
        
        /** other way around, second is better?
        std::vector <std::string> jointOfTips1 = jointsOfFingertipMap.at(tipsNames.first);
        std::vector <std::string> jointOfTips2 = jointsOfFingertipMap.at(tipsNames.second);
        
        // if the joint is not linked with neither of the two colliding tips...
        if ( std::find( jointOfTips1.begin(), jointOfTips1.end(), jp.first) == jointOfTips1.end() &&
             std::find( jointOfTips2.begin(), jointOfTips2.end(), jp.first) == jointOfTips2.end() ) {
              
            std::fill ( jp.second.begin(), jp.second.end(), DEFAULT_JOINT_POS);   
        
            IF USE THIS JOINTINVOLVEDCOUNT REMEMBER
        }
        */
        
        //the tips of the joint
        std::vector < std::string> tips = parserMoveIt->getFingertipsOfJointMap().at(jp.first); 
        
        //check if the two tips that collide are among the ones that the joint moves
        if (std::find (tips.begin(), tips.end(), tipsNames.first) == tips.end() &&
            std::find (tips.begin(), tips.end(), tipsNames.second) == tips.end() ) {
            // not dependant, set to default the position
            std::fill ( jp.second.begin(), jp.second.end(), DEFAULT_JOINT_POS); 
            jointsInvolvedCount.at ( jp.first ) = 0;
        }
    } 
    
    return jointsInvolvedCount;    
}


ROSEE::JointsInvolvedCount ROSEE::FindActions::setOnlyDependentJoints(
    std::set< std::string > tipsNames, JointPos *jPos) {
    
    JointsInvolvedCount jointsInvolvedCount;
    
    for (auto &jp : *jPos) { //for each among ALL joints
        
        jointsInvolvedCount.insert ( std::make_pair (jp.first, 0) );
        
        //the tips of the joint
        std::vector < std::string> tips = parserMoveIt->getFingertipsOfJointMap().at(jp.first); 
        
        // if at least one tip of tipsNames is moved by jp.first joint, set the counter
        // and break the loop (because useless to continue
        // if no tip of tipsNames is moved by the joint, the count remain to zero and the 
        // for ends normally
        for ( auto fingInv : tipsNames ) {
            if (std::find (tips.begin(), tips.end(), fingInv) != tips.end()) {
                jointsInvolvedCount.at ( jp.first )  = 1 ;
                break;
            }
        }
        
        if (jointsInvolvedCount.at ( jp.first ) == 0 ) {
            std::fill ( jp.second.begin(), jp.second.end(), DEFAULT_JOINT_POS); 
            //not used joint, set to default state (all its dof)
        }

    } 
    return jointsInvolvedCount;    
}

