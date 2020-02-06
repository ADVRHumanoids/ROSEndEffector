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
        //Remove here after checking pinches further with method said
        std::cout << "WARNING: I found no collisions between tips. Are you sure your hand"
            << " has some fingertips that collide? If yes, check your urdf/srdf, or"
            << " set a bigger value in N_EXP_COLLISION. I am creating a empty file" << std::endl;
    } 
    
    ROSEE::YamlWorker yamlWorker(parserMoveIt->getHandName(), path2saveYaml);
    
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
        //errror
    }
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
    
    ROSEE::YamlWorker yamlWorker(parserMoveIt->getHandName(), path2saveYaml);
    yamlWorker.createYamlFile(mapForWorker, trigMap.begin()->second.getName());

    return trigMap;
}  

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
                logCollision << "Collision between " << cont.first.first << " and " << 
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







/// trig is the action of closing a SINGLE finger towards the palm
/// to know the joint direction, the position is set to the limit which is different from 0
// nikos easy solution: go in the direction of the max range. All hands have more range of motion
// in the flexion respect to extension (as human finger). NOT valid for other motion, like finger spread or
// thumb addition/abduction. PROBLEM IS: what is the default position????? WE have to assume 0?
/// if a joint is continuos, it is excluded from the trig action. (because I cant think about a continuos 
/// joint that is useful for a trig action, but can be present in theory)
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
    
    for (auto mapEl : parserMoveIt->getJointsOfFingertipMap() ) {
        
        if (parserMoveIt->getNExclusiveJointsOfTip ( mapEl.first, false ) < 2 ) { 
        //if so, we have a simple trig (or none if 0) and not also a tip/finger flex
            continue;
        }
        
        std::string theInterestingJoint = parserMoveIt->getFirstActuatedParentJoint ( mapEl.first, false );
        double tipFlexMax = parserMoveIt->getBiggerBoundFromZero ( theInterestingJoint ).at(0) ;
        
        ActionTrig action ("tipFlex", ActionPrimitive::Type::TipFlex);
        action.setFingerInvolved (mapEl.first) ;
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
    
    for (auto mapEl : parserMoveIt->getJointsOfFingertipMap()) {
        
        if (parserMoveIt->getNExclusiveJointsOfTip ( mapEl.first, false ) < 2 ) { 
        //if so, we have a simple trig (or none if 0) and not also a tip/finger flex
            continue;
        }

        std::string theInterestingJoint = parserMoveIt->getFirstActuatedJointInFinger ( mapEl.first );
        double fingFlexMax = parserMoveIt->getBiggerBoundFromZero ( theInterestingJoint ).at(0) ;

        ActionTrig action ("fingFlex", ActionPrimitive::Type::FingFlex);
        action.setFingerInvolved (mapEl.first) ;
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
    
    auto itMap = trigMap.find ( action.getFingerInvolved() );
    if ( itMap == trigMap.end() ) {
        //still no action for this tip in the map
        
        JointPos jp;
        for (auto it : parserMoveIt->getRobotModel()->getActiveJointModels()){
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




ROSEE::JointPos ROSEE::FindActions::getConvertedJointPos(const robot_state::RobotState* kinematic_state) {
    
    JointPos jp;
    for ( auto actJ : parserMoveIt->getRobotModel()->getActiveJointModels() ) {
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
                std::cout << "[WARNING FINDACTIONS WEAKPINCHES] Im doubling the bounds for your prismatic joint " 
                    << "but I am not sure it is enough to make the tips colliding to find the weak pinches " <<
                    std::endl;
                auto bound = jointModel->getVariableBounds().at(0);
                bound.max_position_ *= 2;
                bound.min_position_ *= 2;
                //at(0) because we are sure to have 1 dof being prismatic
                jointModel->setVariableBounds ( jointModel->getVariableNames().at(0), bound );
                
            } else {
                    
                std::cout << "[WARNING] Why are you using a type " 
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
