#include <ROSEndEffector/FindActions.h>

ROSEE::FindActions::FindActions ( std::string robot_description ){
    
    this->robot_description = robot_description;

    //it is a ros param in the launch, take care that also sdrf is read (param: robot_description_semantic)
    robot_model_loader::RobotModelLoader robot_model_loader(robot_description); 
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


void ROSEE::FindActions::printFingertipLinkNames(){

    std::cout << "Fingertips list:" << std::endl;
    for (const auto &it: fingertipNames) {
        std::cout << it << std::endl;
    }
    std::cout << std::endl;
    
}

void ROSEE::FindActions::printAllLinkNames(){

    std::cout << "All Links  list:" << std::endl;
    for (const auto &it: kinematic_model->getLinkModelNames()) {
        std::cout << it << std::endl;
    }
    std::cout << std::endl;    
}

void ROSEE::FindActions::printActuatedJoints(){

    for (const auto &it: kinematic_model->getActiveJointModels()){
        std::cout << "jointname: " << it->getName() << std::endl;
    }  
    std::cout << std::endl;

}

void ROSEE::FindActions::printJointsOfFingertips(){

    std::stringstream logInfo;

    logInfo << "Tip and joints that influence its position" << std::endl;
    for (const auto &it : jointsOfFingertipMap) {
        logInfo << it.first << " parent joints: \n";
        for (const auto jointName : it.second) {
            logInfo << "\t" << jointName << std::endl;
        }
    }
    std::cout << logInfo.str() << std::endl;
}

void ROSEE::FindActions::printFingertipsOfJoints(){

    std::stringstream logInfo;
    logInfo << "Joint and the tips that it moves" << std::endl;

    for (const auto &it : fingertipsOfJointMap) {
        logInfo << it.first << " child fingertips: \n";
        for (const auto linkName : it.second) {
            logInfo << "\t" << linkName << std::endl;
        }
    }
    std::cout << logInfo.str() << std::endl;
}


///TODO move i look for in Parser
void ROSEE::FindActions::lookForFingertips(){

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


void ROSEE::FindActions::lookJointsTipsCorrelation(){
    
    //initialize the map with all tips and with empty vectors of its joints
    for (const auto &it: fingertipNames) { 
        jointsOfFingertipMap.insert ( std::make_pair (it, std::vector<std::string>() ) );
        
    }
    
    //initialize the map with all the actuated joints and an empty vector for the tips that the joint move
    for (const auto &it: kinematic_model->getActiveJointModels()) { 
        fingertipsOfJointMap.insert ( std::make_pair (it->getName(), std::vector<std::string>() ) );
    }
    
    for ( const auto &joint: kinematic_model->getActiveJointModels()){ //for each actuated joint        
        for (const auto &link : joint->getDescendantLinkModels()) { //for each descendand link
            
            if (std::find(fingertipNames.begin(), fingertipNames.end(), link->getName()) != fingertipNames.end()){
                jointsOfFingertipMap.at ( link->getName() ).push_back( joint->getName() );
                fingertipsOfJointMap.at ( joint->getName() ).push_back( link->getName() );
            }
        }
    }      
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

            //store joint states
            JointStates jointStates = getConvertedJointStates(&kinematic_state);

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
                ActionPinchStrong pinch (cont.first, jointStates, cont.second.at(0) );
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
            }
            
            logCollision << jointStates;                  
            //std::cout << logCollision.str() << std::endl;
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
            
            setOnlyDependentJoints(mapEl.first, &jointStatesWeak);
            
            Eigen::Affine3d tip1Trasf = kinematic_state.getGlobalLinkTransform(mapEl.first.first);
            Eigen::Affine3d tip2Trasf = kinematic_state.getGlobalLinkTransform(mapEl.first.second);
            double distance = (tip1Trasf.translation() - tip2Trasf.translation() ) .norm() ;
                                
            mapEl.second.insertActionState( jointStatesWeak, distance ) ;
        }
            
    }

}


void ROSEE::FindActions::setOnlyDependentJoints(
    std::pair < std::string, std::string > tipsNames, JointStates *jStates) {
    
    for (auto &js : *jStates) { //for each among ALL joints
        
        /** other way around, second is better?
        std::vector <std::string> jointOfTips1 = jointsOfFingertipMap.at(tipsNames.first);
        std::vector <std::string> jointOfTips2 = jointsOfFingertipMap.at(tipsNames.second);
        
        // if the joint is not linked with neither of the two colliding tips...
        if ( std::find( jointOfTips1.begin(), jointOfTips1.end(), js.first) == jointOfTips1.end() &&
             std::find( jointOfTips2.begin(), jointOfTips2.end(), js.first) == jointOfTips2.end() ) {
              
            std::fill ( js.second.begin(), js.second.end(), DEFAULT_JOINT_POS);                     
        }
        */
        
        std::vector < std::string> tips = fingertipsOfJointMap.at(js.first); //the tips of the joint
        
        //check if the two tips that collide are among the ones that the joint moves
        if (std::find (tips.begin(), tips.end(), tipsNames.first) == tips.end() &&
            std::find (tips.begin(), tips.end(), tipsNames.second) == tips.end() ) {
            // not dependant, set to zero the position
            std::fill ( js.second.begin(), js.second.end(), DEFAULT_JOINT_POS); 
        }
    } 
    
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
                std::vector <double> jPos (it->getVariableCount());
                std::fill (jPos.begin(), jPos.end(), 0.0);
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


bool ROSEE::FindActions::checkIfContinuosJoint ( std::string jointName) {
    return (ROSEE::FindActions::checkIfContinuosJoint(kinematic_model->getJointModel(jointName)));     
}

//HACK consider only 2 bounds now, because 1dof joint
// Except planar and full 6-dof, urdf (in 2020) does not permit to have more dofs joints, so this Hack is ok
bool ROSEE::FindActions::checkIfContinuosJoint ( const moveit::core::JointModel* joint ) {
    
    if (joint->getType() != moveit::core::JointModel::REVOLUTE ) {
        return false;
    }
    
    moveit::core::JointModel::Bounds limits = joint->getVariableBounds();
                
    if ( limits.at(0).max_position_ - limits.at(0).min_position_ >= (2*EIGEN_PI) ) {
        //std::cout << limits.at(0).max_position_ - limits.at(0).min_position_ << std::endl;
        return true;
    }
    return false;
}

double ROSEE::FindActions::getBiggestBound(std::string jointName ) {
    return ( ROSEE::FindActions::getBiggestBound (kinematic_model->getJointModel(jointName) ) );
}

// go in the position of the max range, 0 must be included between the bounds
double ROSEE::FindActions::getBiggestBound ( const moveit::core::JointModel* joint ) {
    
    double trigMax;
    moveit::core::JointModel::Bounds limits = joint->getVariableBounds();

    if ( std::abs(limits.at(0).max_position_) > std::abs(limits.at(0).min_position_)) {
        trigMax = limits.at(0).max_position_ ;
    } else {
        trigMax = limits.at(0).min_position_ ;
    }
    return trigMax;
}

//WARNING: continuos joint are not counted because... TODO explain
unsigned int ROSEE::FindActions::getNExclusiveJointsOfTip (std::string tipName) {
    
    unsigned int nExclusiveJoints = 0;

    for (auto jointOfTip : jointsOfFingertipMap.find(tipName)->second ){ 
        
        if ( checkIfContinuosJoint(jointOfTip) == true ) {
            continue; //we dont want to use a continuos joint for the trig
        }
        
        //check if the joints of the tip move only that tip
        if ( fingertipsOfJointMap.find(jointOfTip)->second.size() == 1 &&
             (fingertipsOfJointMap.find(jointOfTip)->second.at(0).compare (tipName) == 0) ) {
            // second condition should be always true if jointsOfFingertipMap and fingertipsOfJointMap 
            // are built well
            

            nExclusiveJoints++;
        }
    }
    
    return nExclusiveJoints;    
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
