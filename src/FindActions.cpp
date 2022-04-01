#include <end_effector/FindActions.h>

ROSEE::FindActions::FindActions ( std::shared_ptr < ROSEE::ParserMoveIt > parserMoveIt ){

    this->parserMoveIt = parserMoveIt;
    
    this->mimicNLRelMap = parserMoveIt->getMimicNLFatherOfJointMap();
}


std::pair <  std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchTight >, 
             std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose > >
             ROSEE::FindActions::findPinch ( std::string path2saveYaml ){
    
    std::map < std::pair <std::string, std::string> , ActionPinchTight > mapOfPinches = checkCollisions();
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose > mapOfLoosePinches;
    fillNotCollidingTips(&mapOfLoosePinches, &mapOfPinches);

    checkWhichTipsCollideWithoutBounds ( &mapOfLoosePinches ) ;

    if (mapOfLoosePinches.size() != 0 ){
        checkDistances (&mapOfLoosePinches) ;
    }
    
    /// EMITTING PART ................
    if (mapOfPinches.size() == 0 ) {  //print if no collision at all
        std::cout << "[FINDACTIONS::" << __func__ << "]: I found no collisions between tips. Are you sure your hand"
            << " has some fingertips that can collide? If yes, check your urdf/srdf, or be sure to"
            << " have set the mesh or some collision geometry for the links, or" 
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

        yamlWorker.createYamlFile(mapForWorker, "pinchTight", path2saveYaml);
    }
    
    if (mapOfLoosePinches.size() == 0 ) { 
        std::cout << "[FINDACTIONS::" << __func__ << "]: I found no loose pinches. This mean that some error happened or that" <<
        " all the tips pairs collide with each other for at least one hand configuration." << std::endl;
        
    } else {
        
        ROSEE::YamlWorker yamlWorker;
        std::map < std::set <std::string> , ActionPrimitive* > mapForWorker;  
        
        for (auto& it : mapOfLoosePinches) {  // auto& and not auto alone!

            ActionPrimitive* pointer = &(it.second);
            std::set < std::string > keys ;
            keys.insert (it.first.first) ;
            keys.insert (it.first.second) ;
            mapForWorker.insert (std::make_pair ( keys, pointer ) );                
        }

        yamlWorker.createYamlFile(mapForWorker, "pinchLoose", path2saveYaml);
    }
    
    return std::make_pair(mapOfPinches, mapOfLoosePinches);
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


std::map <std::string, ROSEE::ActionSingleJointMultipleTips> ROSEE::FindActions::findSingleJointMultipleTips(unsigned int nFinger, std::string path2saveYaml) {
    
    std::map <std::string, ROSEE::ActionSingleJointMultipleTips> mapOfSingleJointMultipleTips;
    
    if (nFinger <= 0) {
        std::cout << "[ERROR FINDACTIONS::" << __func__ << "] please pass a positive number " << 
        " as number of fingers. You passed " << nFinger << " ! " << std::endl;
        return mapOfSingleJointMultipleTips; 
    }
    
    if (nFinger == 1) {
        std::cout << "[ERROR FINDACTIONS::" << __func__ << "]  with 1 finger, you are looking for a ActionTrig, "
            << "and not a ActionSingleJointMultipleTips. Returning an empty map" << std::endl;
        return mapOfSingleJointMultipleTips;
    }
    
    if (nFinger > parserMoveIt->getNFingers() ) {
        std::cout << "[ERROR FINDACTIONS::" << __func__ << "]  I can not find an action which moves " << nFinger << 
        " fingers if the hand has only " << parserMoveIt->getNFingers() << " fingers. Returning an empty map" << std::endl;
        return mapOfSingleJointMultipleTips;
    }
       
    std::string actionName = "singleJointMultipleTips_" + std::to_string(nFinger); //action name same for each action

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
        
        std::vector<std::string> fingersInvolved;
        for (auto tip : mapEl.second){
            fingersInvolved.push_back(parserMoveIt->getFingerOfFingertip (tip) );
        }
        
        ActionSingleJointMultipleTips action (actionName, fingersInvolved, mapEl.first, jpFar, jpNear);
        
        mapOfSingleJointMultipleTips.insert (std::make_pair(mapEl.first, action));
    }
    
    //// EMITTING
    if (mapOfSingleJointMultipleTips.size() == 0 ) {
        std::cout << "[FINDACTIONS::" << __func__ << "]  no singleJointMultipleTips with " << nFinger << " found" << std::endl;
        return mapOfSingleJointMultipleTips;
    }
    
    std::map < std::set <std::string> , ActionPrimitive* > mapForWorker;

    for (auto& it : mapOfSingleJointMultipleTips) {  // auto& and not auto alone!

        ActionPrimitive* pointer = &(it.second);
        std::set<std::string> set;
        set.insert (it.first);
        mapForWorker.insert (std::make_pair ( set, pointer ) );
    }
    
    ROSEE::YamlWorker yamlWorker;
    yamlWorker.createYamlFile(mapForWorker, actionName, path2saveYaml);
    
    return mapOfSingleJointMultipleTips;
}


std::map<std::set<std::string>, ROSEE::ActionMultiplePinchTight> ROSEE::FindActions::findMultiplePinch(unsigned int nFinger, std::string path2saveYaml,
                                                                                                        bool strict ) {
    
    std::map<std::set<std::string>, ROSEE::ActionMultiplePinchTight> multiplePinchMap;
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

std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchTight > ROSEE::FindActions::checkCollisions () {
    
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchTight > mapOfPinches;
    
    planning_scene::PlanningScene planning_scene ( parserMoveIt->getRobotModel() );
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;                       // std::cout << "before" << std::endl;
                       // kinematic_state.printStatePositions();
                        //std::vector<double> one_dof;
                       // one_dof.push_back (1);
                      // kinematic_state.setJointPositions("SFP1_1__SFP1_2", one_dof);
                       // std::cout << "after" << std::endl;
                       // kinematic_state.printStatePositions();
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
        
        setToRandomPositions(&kinematic_state);

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

                ///create the actionPinch
                
                // get the finger name
                auto fingerPair = getFingersPair(cont.first);

                ActionPinchTight pinch (fingerPair, jointPos, cont.second.at(0) );
                pinch.setJointsInvolvedCount ( jointsInvolvedCount );
                auto itFind = mapOfPinches.find ( fingerPair );
                if ( itFind == mapOfPinches.end() ) {
                    //if here, we have to create store the new created action
                    mapOfPinches.insert ( std::make_pair (fingerPair, pinch) );
                    logCollision << ", NEW INSERTION";

                } else { //Check if it is the best depth among the found collision among that pair
                    if (itFind->second.insertActionState( jointPos, cont.second.at(0)) ) {
                         logCollision << ", NEW INSERTION";
                    }
                }
                logCollision << std::endl;
                logCollision << jointPos;
            }
            //this print is for debugging purposes
            //std::cout << logCollision.str() << std::endl;
        }            
    }
    
    return mapOfPinches;
}


void ROSEE::FindActions::checkDistances (std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose >* mapOfLoosePinches) {
        
    robot_state::RobotState kinematic_state(parserMoveIt->getRobotModel());
        
    for (int i = 0; i < N_EXP_DISTANCES; i++){
        
        setToRandomPositions(&kinematic_state);

        //for each pair remaining in notCollidingTips, check if a new min distance is found
        for (auto &mapEl : *mapOfLoosePinches) { 
            
            // restore all joint pos
            JointPos jointPosLoose = getConvertedJointPos(&kinematic_state);
            
            auto tips = getFingertipsPair(mapEl.first);
            
            JointsInvolvedCount jointsInvolvedCount = setOnlyDependentJoints(tips, &jointPosLoose);
            
            Eigen::Affine3d tip1Trasf = kinematic_state.getGlobalLinkTransform(tips.first);
            Eigen::Affine3d tip2Trasf = kinematic_state.getGlobalLinkTransform(tips.second);
            double distance = (tip1Trasf.translation() - tip2Trasf.translation() ) .norm() ;
                                
            mapEl.second.insertActionState( jointPosLoose, distance ) ;
            mapEl.second.setJointsInvolvedCount ( jointsInvolvedCount );
        }
    }
}


void ROSEE::FindActions::removeBoundsOfNotCollidingTips ( 
    const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose >* mapOfLoosePinches,
    robot_model::RobotModelPtr kinematic_model_noBound) {

    for (auto mapEl : *mapOfLoosePinches ) {
        
        //for each joint of first tip...
        auto tips = getFingertipsPair(mapEl.first);
        /// C++ Question: WHY if I use directly parser....at() in the for the string joint is corrupted?
        auto joints = parserMoveIt->getJointsOfFingertipMap().at (tips.first);
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
                    << "but I am not sure it is enough to make the tips colliding to find the loose pinches " <<
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
                    << " in the job done to find the loose pinches " << std::endl << std::endl;
                
                continue;
            }
        }
        
        //for each joint of second tip...
        auto joints2 = parserMoveIt->getJointsOfFingertipMap().at (tips.second);
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
                    << "but I am not sure it is enough to make the tips colliding to find the loose pinches " << std::endl;
                auto bound = jointModel->getVariableBounds().at(0);
                bound.max_position_ *= 2;
                bound.min_position_ *= 2;
                //at(0) because we are sure to have 1 dof being revolute
                jointModel->setVariableBounds ( jointModel->getVariableNames().at(0), bound );
                
            } else {
                    
                std::cout << "[FINDACTIONS::" << __func__ << "] Why are you using a type " 
                    << kinematic_model_noBound ->getJointModel(joint)->getType()
                    << " joint? Code not ready to temporarily delete the multiple dof bounds"
                    << " in the working done to find the loose pinches " << std::endl << std::endl;
                
                continue;
            }
        }
    }
}


void ROSEE::FindActions::checkWhichTipsCollideWithoutBounds (
    std::map < std::pair <std::string, std::string>, ROSEE::ActionPinchLoose >* mapOfLoosePinches ) {
    
    robot_model::RobotModelPtr kinematic_model_noBound = parserMoveIt->getCopyModel();
    
    removeBoundsOfNotCollidingTips (mapOfLoosePinches, kinematic_model_noBound );

    collision_detection::AllowedCollisionMatrix acm;
    acm.setEntry(kinematic_model_noBound->getLinkModelNames(), 
                 kinematic_model_noBound->getLinkModelNames(), true); //true == not considered collisions
    
    for( auto  it : *mapOfLoosePinches) {
        //we want to look for collision only on the pair inside the map
        //take the tip from the keys (that now are fingers)
        std::string tip1 = parserMoveIt->getFingertipOfFinger(it.first.first);
        std::string tip2 = parserMoveIt->getFingertipOfFinger(it.first.second);
        acm.setEntry(tip1, tip2, false); //false == considered collisions   
    }

    planning_scene::PlanningScene planning_scene(kinematic_model_noBound);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;  //set to compute collisions
    collision_request.max_contacts = 1000;

    robot_state::RobotState kinematic_state(kinematic_model_noBound);

    // similar to checkcollisions here, but we dont want to store anything, only check if collision happen
    std::set < std::pair<std::string, std::string> > collidingFingers ;
    for (int i = 0; i < N_EXP_COLLISION; i++){
        collision_result.clear();
        setToRandomPositions(&kinematic_state);

        planning_scene.checkSelfCollision(collision_request, collision_result, kinematic_state, acm);
        
        for (auto cont : collision_result.contacts){
            //moveit contacts is a map between a pair (2 strings with link names) and a vector of Contact object , so cont.first contain the pair of fingerTIPS which collide.
            collidingFingers.insert ( getFingersPair (cont.first) );   
        }
    }

    //erase from loose map the not colliding tips (: not colliding even without bounds)
    for (auto mapEl = mapOfLoosePinches->cbegin(); mapEl != mapOfLoosePinches->cend() ; /*no increment*/ ) {

        if (collidingFingers.count(mapEl->first) == 0 ) {
            mapOfLoosePinches->erase(mapEl++);
        } else { 
            ++mapEl;
        }
    }
}


std::map<std::set<std::string>, ROSEE::ActionMultiplePinchTight> ROSEE::FindActions::checkCollisionsForMultiplePinch(unsigned int nFinger, bool strict) {
    
    std::map < std::set <std::string> , ROSEE::ActionMultiplePinchTight > mapOfMultPinches;
    
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
        setToRandomPositions(&kinematic_state);

        planning_scene.checkSelfCollision(collision_request, collision_result, kinematic_state, acm);
        
        if (collision_result.contacts.size() >= nMinCollision ) { 
        
            double depthSum = 0;
            std::set <std::string> tipsColliding;
            for (auto cont : collision_result.contacts){
                
                tipsColliding.insert(cont.first.first);
                tipsColliding.insert(cont.first.second);
                depthSum += std::abs(cont.second.at(0).depth);
            }
            
            //eg with 2 collision we can have 4 finger colliding because there are two
            //normal distinct pinch and not a 3-pinch... so we exlude these collisions
            if (tipsColliding.size() != nFinger) {
                continue;
            }
                
            //store joint states
            JointPos jointPos = getConvertedJointPos(&kinematic_state);
            JointsInvolvedCount jointsInvolvedCount = setOnlyDependentJoints(tipsColliding, &jointPos);
            
            auto fingerColliding = getFingersSet(tipsColliding);

            ActionMultiplePinchTight pinch (fingerColliding, jointPos, depthSum );
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
        action.setFingerInvolved ( parserMoveIt->getFingerOfFingertip( mapEl.second.at(0)) ) ;

        // mapEl.second.at(0) : sure to have only 1 element for the if before
        insertJointPosForTrigInMap(trigMap, action, mapEl.first, trigMax); 
    }
    
    return trigMap;
}


std::map <std::string, ROSEE::ActionTrig> ROSEE::FindActions::tipFlex() {
    
    std::map <std::string, ROSEE::ActionTrig> tipFlexMap;
    
    for (auto tipName : parserMoveIt->getFingertipNames() ) {

        if (parserMoveIt->getNExclusiveJointsOfTip ( tipName, false ) < 2 ) { 
        //if so, we have a simple trig (or none if 0) and not also a tip/finger flex
            continue;
        }

        std::string theInterestingJoint = parserMoveIt->getFirstActuatedParentJoint ( tipName, false );
        double tipFlexMax = parserMoveIt->getBiggerBoundFromZero ( theInterestingJoint ).at(0) ;
        
        
        ActionTrig action ("tipFlex", ActionPrimitive::Type::TipFlex);
        action.setFingerInvolved (parserMoveIt->getFingerOfFingertip(tipName)) ;

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
    
    for (auto tipName : parserMoveIt->getFingertipNames() ) {
        
        if (parserMoveIt->getNExclusiveJointsOfTip ( tipName, false ) < 2 ) { 
        //if so, we have a simple trig (or none if 0) and not also a tip/finger flex
            continue;
        }

        std::string theInterestingJoint = parserMoveIt->getFirstActuatedJointInFinger ( tipName );
        double fingFlexMax = parserMoveIt->getBiggerBoundFromZero ( theInterestingJoint ).at(0) ;

        ActionTrig action ("fingFlex", ActionPrimitive::Type::FingFlex);
        action.setFingerInvolved ( parserMoveIt->getFingerOfFingertip ( tipName) ) ;
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
        //still no action for this finger in the map

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
    std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchLoose >* mapOfLoosePinches,
    const std::map < std::pair <std::string, std::string> , ROSEE::ActionPinchTight >* mapOfPinches) {
    
    // first fill mapOfLoosePinches with all pairs ...
    for ( auto fingerEl1 : parserMoveIt->getFingertipOfFingerMap() )  {
        for ( auto fingerEl2 : parserMoveIt->getFingertipOfFingerMap() ) { 
            
            // important to put in order in the pair, then in the set thing are autoordered
            if (fingerEl1.first < fingerEl2.first) {
                mapOfLoosePinches->insert (std::make_pair (std::make_pair (fingerEl1.first, fingerEl2.first), ActionPinchLoose(fingerEl1.first, fingerEl2.first)));
                
            } else if (fingerEl1.first > fingerEl2.first) {
                mapOfLoosePinches->insert (std::make_pair (std::make_pair (fingerEl2.first, fingerEl1.first), ActionPinchLoose(fingerEl2.first, fingerEl1.first)));
            }    
        }
    }  
    
    // ... then remove all the colliding tips
    for (const auto mapEl : *mapOfPinches){
        mapOfLoosePinches->erase(mapEl.first);
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

void ROSEE::FindActions::setToDefaultPositionPassiveJoints(moveit::core::RobotState * kinematic_state) {
    
    const double pos = DEFAULT_JOINT_POS; //idk why but setJointPositions want a pointer as 2nd arg
    for (auto joint : parserMoveIt->getPassiveJointNames()){
        kinematic_state->setJointPositions(joint, &pos);
    }
    
}

void ROSEE::FindActions::setToRandomPositions(moveit::core::RobotState * kinematic_state) {

    //NOTE this function will consider the mimic LINEAR. if in mimic tag only nlFunPos is written, default LINEAR args are 
    //considered : multiplier 1 and offset 0. Then the joint which mimic someone will have same position of father, it is not
    // an error of randomic. Also, this is not a problem for us because below we overwrite the mimic sons, and we keep only 
    // the random pos of actuated joints.
    kinematic_state->setToRandomPositions();
    
    //when setting a joint pos (also a single one) moveit call also updateMimic using the standard linear params
    //we cant take the single functions inside the setJoint pos of moveit, because are private, so we
    //must always bear the updateMimic. So, here the joint pos of nonlinear mimic joint are ovewritten
    //with the correct non linear function
    
    if (mimicNLRelMap.size() > 0 ) { //necessary if we have the for immediately after? faster?
        
        for (auto el : mimicNLRelMap) {
            
            double mimPos = 0;
            
            try {
                //HACK single dof joint
                double fatherPos = kinematic_state->getJointPositions(el.second.first)[0];
                
                mu::Parser p;
                //we assume that there is always a x in the expression
                p.DefineVar("x", &fatherPos);
                p.SetExpr(el.second.second);
                mimPos = p.Eval();
            }

            catch (mu::Parser::exception_type &e)
            {
                std::cout << e.GetMsg() << std::endl;
                std::cout << "[FINDACTIONS " << __func__ << "] Parsing of non linear function for mimic joint "
                 << "'" << el.first << "'. Please be sure to put characther 'x' as (unique) variable for father position" 
                 << "in the expression. Have you used syntax valid for muparser?. Expression found: '" 
                 << el.second.second << "'" << std::endl;
                 
                 exit(-1); //TODO is it good to use exit?
                
            }
            
            //HACK single dof joint
            std::vector<double> one_dof ;
            one_dof.push_back(mimPos);
            
            //we enforce the bounds, in this way. calling at the end kinematic_state->enforceBounds() call internally updateMimicJoint
            //and we would have again problems.
            kinematic_state->getJointModel(el.first)->enforcePositionBounds(one_dof.data());
            kinematic_state->setJointPositions(el.first, one_dof);

                  
        }
        
        
    }
    
    setToDefaultPositionPassiveJoints(kinematic_state);

}

std::pair <std::string, std::string> ROSEE::FindActions::getFingersPair (std::pair <std::string, std::string> tipsPair) const {
    
    std::pair <std::string, std::string> fingersPair = std::make_pair (
        parserMoveIt->getFingerOfFingertip(tipsPair.first),
        parserMoveIt->getFingerOfFingertip(tipsPair.second)  );

    //So we have the key pair always in lexicographical order
    if ( fingersPair.first.compare (fingersPair.second) > 0 ) {
        auto temp = fingersPair.first;
        fingersPair.first = fingersPair.second;
        fingersPair.second = temp;
        
    } else if (fingersPair.first.compare (fingersPair.second) == 0 ) {
        std::cout << "[FINDACTIONS " << __func__ << "] STRANGE ERROR: '" << tipsPair.first <<
          "' and '" << tipsPair.second << "' are in the same finger '" << fingersPair.first <<
          "' so this pair can't perform a pinch" << std::endl;
        return std::pair<std::string, std::string>();
    }
    
    return fingersPair;
}

std::set <std::string> ROSEE::FindActions::getFingersSet (std::set <std::string> tipsSet) const {
    
    std::set <std::string> fingersSet;
    for (auto it : tipsSet) {
        
        fingersSet.insert ( parserMoveIt->getFingerOfFingertip ( it ) );
    }

    //If size is less, there is a finger that we have try to insert more than once.
    if ( fingersSet.size() < tipsSet.size() ) {
        std::cout << "[FINDACTIONS " << __func__ << "] STRANGE ERROR: " <<
            "the tipsSet passed has some fingertips that belong to the same finger."  
            << " I will return an empty set " << std::endl;
            
        return std::set <std::string>();
    }
    
    return fingersSet;
}

std::pair <std::string, std::string> ROSEE::FindActions::getFingertipsPair (std::pair <std::string, std::string> fingersPair) const {
    
    std::pair <std::string, std::string> tipsPair = std::make_pair (
        parserMoveIt->getFingertipOfFinger(fingersPair.first),
        parserMoveIt->getFingertipOfFinger(fingersPair.second)  );

    //So we have the key pair always in lexicographical order
    if ( tipsPair.first.compare (tipsPair.second) > 0 ) {
        auto temp = tipsPair.first;
        tipsPair.first = tipsPair.second;
        tipsPair.second = temp;
        
    } else if (tipsPair.first.compare (tipsPair.second) == 0 ) {
        std::cout << "[FINDACTIONS " << __func__ << "] STRANGE ERROR: '" << fingersPair.first <<
          "' and '" << fingersPair.second << "' have the same fingertip '" << tipsPair.first <<
          "' so this pair can't perform a pinch" << std::endl;
        return std::pair<std::string, std::string>();
    }
    
    return tipsPair;
}
