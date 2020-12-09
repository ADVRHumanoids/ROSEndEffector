/*
 * Copyright 2020 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros_end_effector/ParserMoveIt.h>

ROSEE::ParserMoveIt::ParserMoveIt() {

}

ROSEE::ParserMoveIt::~ParserMoveIt() {

}

bool ROSEE::ParserMoveIt::init ( std::string robot_description, bool verbose ) {
    
    if (robot_model != nullptr ) {
        std::cerr << "[PARSER::"  << __func__ << "]: init() already called by someone " << std::endl;;
        return false;
    }
    // it is a ros param in the launch, take care that also sdrf is read by robot_mo
    // (param for srd is robot_description_semantic)
    this->robot_description = robot_description;
    
    //false: we dont need kinematic solvers now
    robot_model_loader::RobotModelLoader robot_model_loader(robot_description, false) ; 
    robot_model = robot_model_loader.getModel() ;
    if (robot_model == nullptr) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: Fail To load robot model " << robot_description <<
            " are you sure to have put both urdf and srdf files in the parameter server " <<
            "with names robot_description and robot_description_semantic, respectively?" << std::endl; 
        return false;
    }
    std::cout << "[PARSER::" << __func__ << "]: Parsed Model: " << robot_model->getName() << std::endl; ;
    
    handName = robot_model->getName();
    
    lookForFingertips(verbose);
    lookForActiveJoints();
    lookForPassiveJoints();
    lookForDescendants();
    lookJointsTipsCorrelation();
    
    return true;
    
}

std::string ROSEE::ParserMoveIt::getHandName() const {
    return handName;
}

std::vector<std::string> ROSEE::ParserMoveIt::getFingertipNames() const {
    return fingertipNames;
}

std::vector<std::string> ROSEE::ParserMoveIt::getActiveJointNames() const {
    return activeJointNames;
}

std::vector<const moveit::core::JointModel*> ROSEE::ParserMoveIt::getActiveJointModels() const {
    return activeJointModels;
}

std::vector<std::string> ROSEE::ParserMoveIt::getPassiveJointNames() const {
    return passiveJointNames;
}

std::map <std::string, std::vector < const moveit::core::LinkModel* > >  ROSEE::ParserMoveIt::getDescendantLinksOfJoint() const {
    return descendantLinksOfJoint;
}

std::map <std::string, std::vector < const moveit::core::JointModel* > >  ROSEE::ParserMoveIt::getDescendantJointsOfJoint() const {
    return descendantJointsOfJoint;
}

unsigned int ROSEE::ParserMoveIt::getNFingers () const {
    return nFingers;
}

const robot_model::RobotModelPtr ROSEE::ParserMoveIt::getRobotModel () const {
    return robot_model;
}

std::map<std::string, std::vector<std::string> > ROSEE::ParserMoveIt::getFingertipsOfJointMap() const {
    return fingertipsOfJointMap;
}

std::map<std::string, std::vector<std::string> > ROSEE::ParserMoveIt::getJointsOfFingertipMap() const {
    return jointsOfFingertipMap;
}

std::map < std::string, std::string> ROSEE::ParserMoveIt::getFingerOfFingertipMap() const {
    return fingerOfFingertipMap;
}

std::string ROSEE::ParserMoveIt::getFingerOfFingertip (std::string tipName) const {
    
    auto it = fingerOfFingertipMap.find(tipName);
    
    if (it != fingerOfFingertipMap.end() ) {
        return (it->second);
        
    } else {
        return "";
    }
}

std::map < std::string, std::string> ROSEE::ParserMoveIt::getFingertipOfFingerMap() const {
    return fingertipOfFingerMap;
}

std::string ROSEE::ParserMoveIt::getFingertipOfFinger (std::string fingerName) const {
    
    auto it = fingertipOfFingerMap.find(fingerName);
    
    if (it != fingertipOfFingerMap.end() ) {
        return (it->second);
        
    } else {
        return "";
    }
}

std::pair<std::string, std::string> ROSEE::ParserMoveIt::getMimicNLFatherOfJoint(std::string mimicNLJointName) const {
    
    std::pair<std::string, std::string> retPair;
    
    auto it = mimicNLFatherOfJointMap.find(mimicNLJointName);
    
    if (it != mimicNLFatherOfJointMap.end()) {
        retPair = it->second;
    }
    return retPair;
}

std::map<std::string, std::pair<std::string, std::string>> ROSEE::ParserMoveIt::getMimicNLFatherOfJointMap() const {
    
    return mimicNLFatherOfJointMap;
    
}

std::string ROSEE::ParserMoveIt::getMimicNLJointOfFather(std::string mimicNLFatherName, std::string mimicNLJointName) const {
    
    auto map = getMimicNLJointsOfFather(mimicNLFatherName);
    
    auto it = map.find(mimicNLJointName);
    
    if (it != map.end()) {
        return it->second;
    }
    
    return "";
}

std::map<std::string, std::string> ROSEE::ParserMoveIt::getMimicNLJointsOfFather(std::string mimicNLFatherName) const {
    
    std::map<std::string, std::string> map;
    
    auto it = mimicNLJointsOfFatherMap.find(mimicNLFatherName);
    
    if (it != mimicNLJointsOfFatherMap.end()) {
        map = it->second;
    }
    return map;
}

std::map<std::string, std::map<std::string, std::string>> ROSEE::ParserMoveIt::getMimicNLJointsOfFatherMap() const {
    
    return mimicNLJointsOfFatherMap;
    
}


robot_model::RobotModelPtr ROSEE::ParserMoveIt::getCopyModel() const {
    robot_model_loader::RobotModelLoader robot_model_loader(robot_description); 
    return robot_model_loader.getModel();
}

std::vector < std::string > ROSEE::ParserMoveIt::getGroupOfLink ( std::string linkName ) { 
    
    std::vector < std::string > groupsReturn;

    if (robot_model == nullptr) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: robot_model is null. Have you called init() before?"  << std::endl;
        return groupsReturn;
    }
    
    for (auto group : robot_model->getJointModelGroups() ) {
        
        if ( group->hasLinkModel(linkName) ) {
                
            groupsReturn.push_back ( group->getName() ) ;
        }
    }
    return groupsReturn;
}

bool ROSEE::ParserMoveIt::groupIsChain(const std::string groupName) const {
    
    if (robot_model == nullptr) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: robot_model is null. Have you called init() before?"  << std::endl;
        return false;
    }
    
    if (! robot_model->hasJointModelGroup(groupName) ) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: " << groupName << " is not a group "  << std::endl;
        return false;
    }
    return groupIsChain(robot_model->getJointModelGroup(groupName));
}

bool ROSEE::ParserMoveIt::groupIsChain(const moveit::core::JointModelGroup* group) const {
    
    std::stringstream log;
    log << "Checking if " << group->getName() << " is a chain ..." << std::endl;
    for (auto link : group->getLinkModels() ){
        if (link->getChildJointModels().size() > 1 ) {
            log << "... no because " << link->getName() << " has " << link->getChildJointModels().size() << " children " << std::endl;
           // std::cout << log.str() << std::endl;
            return false;
        }
    }
    return true;
}



bool ROSEE::ParserMoveIt::checkIfContinuosJoint ( std::string jointName) const {
    if (robot_model == nullptr) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: robot_model is null. Have you called init() before?"  << std::endl;
        return false;
    }
    return (ROSEE::ParserMoveIt::checkIfContinuosJoint(robot_model->getJointModel(jointName)));     
}

bool ROSEE::ParserMoveIt::checkIfContinuosJoint ( const moveit::core::JointModel* joint ) const {
    if (robot_model == nullptr) {
        std::cerr <<  " [PARSER::" << __func__ << 
            "]: robot_model is null. Have you called init() before?" << std::endl;
        return false;
    }
    
    // for moveit, a continuos joint is a revolute joint
    if (joint->getType() != moveit::core::JointModel::REVOLUTE ) {
        return false;
    }
    
    //if revolute, only one limit (at.(0))
    moveit::core::JointModel::Bounds limits = joint->getVariableBounds();
    if ( limits.at(0).max_position_ - limits.at(0).min_position_ >= (2*EIGEN_PI) ) {
        return true;
    }
    
    return false;
}

std::vector <double> ROSEE::ParserMoveIt::getBiggerBoundFromZero ( std::string jointName ) const {
    if (robot_model == nullptr) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: robot_model is null. Have you called init() before?" << std::endl;
        return std::vector<double>();
    }
    return ( ROSEE::ParserMoveIt::getBiggerBoundFromZero (robot_model->getJointModel(jointName) ) );

}

std::vector <double> ROSEE::ParserMoveIt::getBiggerBoundFromZero ( const moveit::core::JointModel* joint ) const {
    if (robot_model == nullptr) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: robot_model is null. Have you called init() before?" << std::endl;
        return std::vector<double>();
    }
    
    moveit::core::JointModel::Bounds limits = joint->getVariableBounds();

    std::vector <double> maxPos;
    for ( auto limit : limits ) {
        if ( std::abs(limit.max_position_) > std::abs(limit.min_position_)) {
            maxPos.push_back ( limit.max_position_ ) ;
        } else {
            maxPos.push_back ( limit.min_position_ ) ;
        }
    }
    return maxPos;
}

std::vector <double> ROSEE::ParserMoveIt::getSmallerBoundFromZero ( std::string jointName ) const {
    if (robot_model == nullptr) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: robot_model is null. Have you called init() before?" << std::endl;
        return std::vector<double>();
    }
    return ( ROSEE::ParserMoveIt::getSmallerBoundFromZero (robot_model->getJointModel(jointName) ) );

}

std::vector <double> ROSEE::ParserMoveIt::getSmallerBoundFromZero ( const moveit::core::JointModel* joint ) const {
    if (robot_model == nullptr) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: robot_model is null. Have you called init() before?" << std::endl;
        return std::vector<double>();
    }
    
    moveit::core::JointModel::Bounds limits = joint->getVariableBounds();

    std::vector <double> minPos;
    for ( auto limit : limits ) {
        if ( std::abs(limit.max_position_) < std::abs(limit.min_position_)) {
            minPos.push_back ( limit.max_position_ ) ;
        } else {
            minPos.push_back ( limit.min_position_ ) ;
        }
    }
    return minPos;
}

unsigned int ROSEE::ParserMoveIt::getNExclusiveJointsOfTip ( std::string tipName, bool continuosIncluded ) const {
    
    if (jointsOfFingertipMap.size() == 0) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: jointsOfFingertipMap empty. Have you called init() before? Also check your URDF and SRDF files" 
            << std::endl;
        return 0;
    }
    
    if (fingertipsOfJointMap.size() == 0) {
        std::cerr << " [PARSER::" << __func__ << 
            "]: jointsOfFingertipMap empty. Have you called init() before? Also check your URDF and SRDF files" 
            << std::endl;
        return 0;
    }
    
    unsigned int nExclusiveJoints = 0;

    for (auto jointOfTip : jointsOfFingertipMap.find(tipName)->second ) { 

        if ( !continuosIncluded && checkIfContinuosJoint(jointOfTip) == true ) {
            continue; //we dont want to count a continuos joint
        }
    
        //check if the joints of the tip move only that tip
        if ( fingertipsOfJointMap.find(jointOfTip)->second.size() == 1 ) {

            if (fingertipsOfJointMap.find(jointOfTip)->second.at(0).compare (tipName) != 0) {
                //  this condition is false if jointsOfFingertipMap and fingertipsOfJointMap are built well
                std::cerr << " [PARSER::" << __func__ << 
                "]: Strange error in fingertipsOfJointMap and jointsOfFingertipMap: they are not consistent: " 
                << "The unique tip present in the map for the key " << jointOfTip 
                << " is " << fingertipsOfJointMap.find(jointOfTip)->second.at(0) 
                << " and not " << tipName << " as it should be" 
                << std::endl;
                return 0;  
            }
            
            nExclusiveJoints++;
        }
    }
    return nExclusiveJoints;  
}


std::string ROSEE::ParserMoveIt::getFirstActuatedParentJoint ( std::string linkName, bool includeContinuos ) const {

    const moveit::core::LinkModel* linkModel = robot_model->getLinkModel ( linkName ) ;

    while ( linkModel->getParentJointModel()->getMimic() != NULL || 
            linkModel->parentJointIsFixed() ||
            linkModel->getParentJointModel()->isPassive() || 
            (!includeContinuos && checkIfContinuosJoint(linkModel->getParentJointModel())) ) {
        
        //an active joint is not any of these condition.
        //passive is an attribute of the joint in the srdf, so it may be not setted 
        // (default is not passive), so we need also the getMimic == NULL 
        // (ie: an actuated joint dont mimic anything)
        //WARNING these 4 conditions should be enough I think
        
        linkModel = linkModel->getParentLinkModel();
        
        if (linkModel == NULL ) {
        
            std::cerr << " [PARSER::" << __func__ << 
                "]: Strange error: fingertipsOfJointMap, jointsOfFingertipMap, and/or other structures " <<
                "may have been built badly"  << std::endl ;
            return "";
        }
    }

    return (linkModel->getParentJointModel()->getName());
}


std::string ROSEE::ParserMoveIt::getFirstActuatedJointInFinger (std::string linkName) const {
    const moveit::core::LinkModel* linkModel = robot_model->getLinkModel(linkName);
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
    
    return joint->getName();
}



/*********************************** PRIVATE FUNCTIONS **********************************************************/
void ROSEE::ParserMoveIt::lookForFingertips(bool verbose) {
     for (auto it: robot_model->getJointModelGroups()) {
        
        std::string logGroupInfo;
        logGroupInfo = "[PARSER::" + std::string(__func__) + "] Found Group '" + it->getName() + "', " ;
        
        if (it->getSubgroupNames().size() != 0 ) {
            logGroupInfo.append("but it has some subgroups \n");
            
        } else if (! groupIsChain ( it ) ) { 
            logGroupInfo.append("but it is not a chain \n");
        
        } else if (it->getLinkModels().size() == 0) { 
            logGroupInfo.append("but it has 0 links \n");

        } else {

            logGroupInfo.append("with links: \n");
            
            std::string theTip = ""; //the last link with a visual geometry
            for (auto itt : it->getLinkModels()) {
                
                logGroupInfo.append("\t'" + itt->getName() + "' ");
                
                if (itt->getChildJointModels().size() != 0) {
                    logGroupInfo.append("(not a leaf link) ");
                } else {
                    logGroupInfo.append("(a leaf link) ");
                }
                
                if (itt->getShapes().size() == 0 ) {
                    logGroupInfo.append("(no visual geometry) ");
                    
                } else {
                    theTip = itt->getName();
                }
                logGroupInfo.append("\n");

            }
            
            if (theTip.compare("") == 0) {
                logGroupInfo.append("Warning: No link has a mesh in this group\n");
                
            } else {
                fingertipNames.push_back(theTip);
                fingerOfFingertipMap.insert( std::make_pair(theTip, it->getName()));
                fingertipOfFingerMap.insert( std::make_pair(it->getName(), theTip));
            }
        }
        
        if (verbose) {
            std::cout << logGroupInfo << std::endl;
        }
    }
    nFingers = fingertipNames.size();
}

void ROSEE::ParserMoveIt::lookForActiveJoints() { 
    
    for (auto joint : robot_model->getActiveJointModels() ) { 
        // robot_model->getActiveJointModels() returns not fixed not mimic but CAN return PASSIVE joints
        if (! joint->isPassive() ) {
            activeJointNames.push_back(joint->getName());
            activeJointModels.push_back(joint);
        }
    }
}

void ROSEE::ParserMoveIt::lookForPassiveJoints() { 
    
    for (auto joint : robot_model->getJointModels() ) { 
        if ( joint->isPassive() ) {
            passiveJointNames.push_back(joint->getName());
        }
    }
}


void ROSEE::ParserMoveIt::lookJointsTipsCorrelation() {
    
    //initialize the map with all tips and with empty vectors of its joints
    for (const auto &it: fingertipNames) { 
        jointsOfFingertipMap.insert ( std::make_pair (it, std::vector<std::string>() ) );
    }
    
    //initialize the map with all the actuated joints and an empty vector for the tips that the joint move
    for (const auto &it: activeJointNames) { 
        fingertipsOfJointMap.insert ( std::make_pair (it, std::vector<std::string>() ) );
    }
    
    for ( const auto &jointLink: descendantLinksOfJoint){ //for each actuated joint   

        for (const auto &link : jointLink.second) { //for each descendant link

            //if link is a tip...
            if (std::find(fingertipNames.begin(), fingertipNames.end(), link->getName()) != fingertipNames.end()){
                jointsOfFingertipMap.at ( link->getName() ) .push_back( jointLink.first);
                fingertipsOfJointMap.at ( jointLink.first ) .push_back( link->getName() );
            }
        }
    }

}

void ROSEE::ParserMoveIt::lookForDescendants () {
    
    for (auto actJoint : activeJointModels) {
        
        std::vector < const moveit::core::LinkModel* >  linksVector;      
        std::vector < const moveit::core::JointModel* >  jointsVector;   
        
        getRealDescendantLinkModelsRecursive ( actJoint->getChildLinkModel(), linksVector, actJoint, jointsVector );

        //now we have to look among the mimic joints, but the mimic of only joint passed as argument, not also the mimic of children joints
        for (auto mimicJ : actJoint->getMimicRequests()) {
        // but we do not look on mimic joints that are children of the this joint in the tree,
        //  because we have already explored them with recursion. Here we look only for mimic that are "cousins" of this joint
            if (std::find (jointsVector.begin(), jointsVector.end(), mimicJ) == jointsVector.end() ) {
                getRealDescendantLinkModelsRecursive (mimicJ->getChildLinkModel(), linksVector, mimicJ, jointsVector );  
            }
        }
        
        descendantLinksOfJoint.insert (std::make_pair ( actJoint->getName(), linksVector ) );
        descendantJointsOfJoint.insert (std::make_pair ( actJoint->getName(), jointsVector ) );
    }
}

void ROSEE::ParserMoveIt::getRealDescendantLinkModelsRecursive ( 
    const moveit::core::LinkModel* link,  std::vector< const moveit::core::LinkModel* > & linksVect,
    const moveit::core::JointModel* joint,  std::vector< const moveit::core::JointModel* > & jointsVect ) const {
    
    linksVect.push_back (link) ;
    jointsVect.push_back (joint);
    auto childJoints = link->getChildJointModels();
    if ( childJoints.size() == 0 ) { 
        return; //recursive base
    }
    
    for (auto cj : childJoints) {
        //recursion
        getRealDescendantLinkModelsRecursive( cj->getChildLinkModel(), linksVect, cj, jointsVect );
    }
    
}


void ROSEE::ParserMoveIt::parseNonLinearMimicRelations (std::string xml) {
        
        
    //we do not make urdf verification here, it is already done before by robot model loader of moveit,
    //and also by Parser with parseUrdf
    TiXmlDocument tiDoc;
    tiDoc.Parse(xml.c_str());
    //std::cout << robot_description << std::endl;
    TiXmlElement* jointEl = tiDoc.FirstChildElement("robot")->FirstChildElement("joint") ;
            
    while (jointEl) {
        
        std::string jointName = jointEl->Attribute("name");
        auto mimicEl = jointEl->FirstChildElement("mimic");
        if (mimicEl) {
            auto nlAttr = mimicEl->Attribute("nlFunPos");
            if (nlAttr) {
                //std::cout << jointName << std::endl;
                //std::cout << nlAttr << std::endl;
                //std::cout << mimicEl->Attribute("joint") << std::endl;
                std::string fatherName = mimicEl->Attribute("joint");
                mimicNLFatherOfJointMap.insert ( std::make_pair( jointName,
                                                        std::make_pair(fatherName, nlAttr)) );
                
                mimicNLJointsOfFatherMap[fatherName].insert(std::make_pair(jointName, nlAttr)) ;
            }
        }
        
        jointEl = jointEl->NextSiblingElement("joint");
    }                
    
}
