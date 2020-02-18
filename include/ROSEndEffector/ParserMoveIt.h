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

#ifndef __ROSEE_PARSER_MOVEIT_H
#define __ROSEE_PARSER_MOVEIT_H

#include <string>
#include <vector>
#include <map>
#include <iostream>

#include <moveit/robot_model_loader/robot_model_loader.h>

namespace ROSEE {
/**
 * @brief class to parse urdf and srdf with moveit classes and to give information about the model parsed
 * @todo merge this with Parser class?
 */
class ParserMoveIt {
    
public:
    
    typedef std::shared_ptr<ParserMoveIt> Ptr;
    typedef std::shared_ptr<const ParserMoveIt> ConstPtr;

    ParserMoveIt();
    ~ParserMoveIt();
    
    /**
     * @brief Init the parser, fill the structures
     * @param robot_description 
     * 
     */
    bool init (std::string robot_description) ;
    std::string getHandName () const;
    unsigned int getNumberOfTips () const ;
    std::vector <std::string> getFingertipNames () const; 
    std::vector <std::string> getActiveJointNames () const; 
    std::vector <const moveit::core::JointModel*> getRealActiveJointModels () const; 
    /** 
     * @brief the robot model can't be modified, if you want it to modify, use @ref getCopyModel 
     * to get a copy.
     * @return const robot_model::RobotModelPtr a shared pointer to the robot model
     */
    const robot_model::RobotModelPtr getRobotModel () const ;
    std::map < std::string, std::vector<std::string> > getFingertipsOfJointMap () const;
    std::map < std::string, std::vector<std::string> > getJointsOfFingertipMap () const;
    
    /** 
     * @brief This function reload another model, same as the one loaded in \ref init but this one can be
     * modified externally, because it will not affect the internal structures of this class
     * @return robot_model::RobotModelPtr a pointer to a new robot model created
     */
    robot_model::RobotModelPtr getCopyModel ( ) const;
    
    /**
     * @brief This function explores all groups of srdf and says to which ones the linkName
     * belongs to. Returns a vector because a link can be part of more group.
     * @param linkName the name on the link for which look the group
     * @return std::vector < std::string > a vector containing the name of all the groups which contain the linkName
     */
    std::vector < std::string > getGroupOfLink ( std::string linkName );
    
    /**
     * @brief check if a group (defined in srdf file) is a chain. See \ref groupIsChain ( \ref moveit::core::JointModelGroup* group )
     * @param groupName the name of the group
     * @return bool : true is group is chain, false othwerwise (also false if some errors happened)
     */
    bool groupIsChain ( const std::string groupName ) const;
    
    /**
     * @brief check if a group (defined in srdf file) is a chain. This is done simply exploring all the links of the group:
     * if no links have more than 1 children, the group is a chain.
     * Moveit has a function isChain() but I don't understand how it works,
     * in fact also if there is a link with more children joint sometimes the group is considered a chain anyway. I can't find where 
     * the moveit _is_chain member is set.
     * @param group pointer to moveit::core::JointModelGroup object
     * @return bool : true is group is chain, false othwerwise (also false if some errors happened)
     */
    bool groupIsChain ( const moveit::core::JointModelGroup* group ) const;
    
    /**
     * @brief check if the passed joint is continuos (i.e. a revolute one with sum of bounds greater than 2*PI)
     * @param jointName the name of the joint
     * @return bool true if joint is continuos
     */
    bool checkIfContinuosJoint ( const std::string jointName) const;
    /**
     * @brief check if the passed joint is continuos (i.e. a revolute one with sum of bounds greater than 2*PI)
     * @param joint pointer to the joint model
     * @return bool true if joint is continuos
     */
    bool checkIfContinuosJoint ( const moveit::core::JointModel* joint ) const;
    
    /**
     * @brief For each DOF of a joint, find the limit which is farther from 0 position
     * @param jointName the name of the joint
     * @return std::vector<double> a vector (because joint can have more dofs) containing the values of the limits that are farther from 0
     */
    std::vector<double> getBiggerBoundFromZero ( std::string jointName ) const;
    /**
     * @brief For each DOF of a joint, find the limit which is farther from 0 position
     * @param joint pointer to the joint model
     * @return std::vector<double> a vector (because joint can have more dofs) containing the values of the limits that are farther from 0
     */
    std::vector<double> getBiggerBoundFromZero ( const moveit::core::JointModel* joint ) const;
    
    /**
     * @brief For each DOF of a joint, find the limit which is nearer from 0 position
     * @param jointName the name of the joint
     * @return std::vector<double> a vector (because joint can have more dofs) containing the values of the limits that are nearer from 0
     */
    std::vector<double> getSmallerBoundFromZero ( std::string jointName ) const;
    /**
     * @brief For each DOF of a joint, find the limit which is nearer from 0 position
     * @param joint pointer to the joint model
     * @return std::vector<double> a vector (because joint can have more dofs) containing the values of the limits that are nearer from 0
     */
    std::vector<double> getSmallerBoundFromZero ( const moveit::core::JointModel* joint ) const;
    
    /**
     * @brief Given a fingertip link, this function return the number of the joint that affect
     * only the position of this fingertip and not any other fingertips (obviously the joints can affect
     * different other links that are not fingertips (e.g. middles phalanges) )
     * @param tipName the name of the fingertip link
     * @param continuosIncluded a bool set to true if we want to count also the possible present 
     *      continuos joints
     * @return unsigned int the number of the exclusive joints
     */
    unsigned int getNExclusiveJointsOfTip (std::string tipName, bool continuosIncluded) const;
    
    /**
     * @brief starting from the given link, we explore the parents joint, until we found 
     * the first actuated. This ones will be the interesting joint 
     * @param linkName the name of the link for which look for the parent joint
     * @param includeContinuos a bool set to true if we want to include continuos joint in the search
     * @return std::string the name of the first actuated parent joint
     */
    std::string getFirstActuatedParentJoint ( std::string linkName, bool includeContinuos ) const;
    
    /**
     * @brief Given the linkName, this function returns the actuated joint that is a parent of this link 
     * and it is the first joint of the chain which the link belongs to. E.G. given a fingertip, it returns
     * the first actuated joint of the finger (e.g. the joint that moves the firsts phalanges, if present)
     * From the linkName given, this function explores the parent links until we found a link with
     * more than 1 child joint. This means that we have found the "base" of the kinematic chain, so we can go 
     * forward to find the first actuated joint
     * @param linkName the name of the link which is part of the chain where we have to look in
     * @return std::string the name of the wanted joint
     */
    std::string getFirstActuatedJointInFinger (std::string linkName) const ;
    
private:
    
    std::string handName;
    robot_model::RobotModelPtr robot_model;
    std::vector<std::string> fingertipNames;
    std::vector<std::string> activeJointNames;
    std::vector<const moveit::core::JointModel*> activeJointModels;
    std::string robot_description;
    
    /** The map with as key the name of the fingertip and as value all the joints (actuated) that can modify its pose*/
    std::map<std::string, std::vector<std::string>> jointsOfFingertipMap;
    
    /** The map with as key the name of the actuated joint and as value all the fingertips which pose can be modified by the joint */
    std::map<std::string, std::vector<std::string>> fingertipsOfJointMap;
    
    /**
     * @brief This function explore the robot_model (which was built from urdf and srdf files), 
     *  and fills the fingerTipNames vector.
     *  In particular, the function explores only the groups specified in the srdf, and prints infos
     *  about each link it finds (eg. not a fingertin, not a chain group, and so on).
     *  A fingertip is a link with the following conditions:
     *  - It is part of a group (defined in the srdf)
     *  - The group which the tip is part of is a chain (not a tree)
     *  - It is a "leaf" link, ie it has not children joints/links
     * @todo Check also if it is unique in the group?
     * @warning Only link belonging to a group are explored (and printed), so other links (if present) 
     *  are not considered 
     */
    void lookForFingertips();
    
    
    /**
     * @brief This function look for all active joints in the model (i.e. not mimic, not fixed, not passive) 
     * There exist a moveit function \ref getActiveJointModels() which return all not mimic and not fixed, but it can return
     * also passive joints (a info that is stored in srdf file). So this function also check if the joint is not passive
     * It stores both the joint names and pointer to joint models in two private vector (\ref activeJointNames and \ref activeJointModels)
     */
    void lookForActiveJoints();
    

    /** 
     * @brief Here, we find for each tip, which are all the joints (active) that can modifies its position
     * It is easier to start from each joint and see which tips has as its descendands, because there is the
     * getDescendantLinkModels() function in moveit that gives ALL the child links.
     * There is not a function like getNonFixedParentJointModels from the tip, there is only the one to take the 
     * FIRST parent joint (getParentJointModel())
     * Meanwhile, we find also, for each joint, all the tips that are influenced by the joint movement:
     * \ref fingertipsOfJointMap
     */
    void lookJointsTipsCorrelation();
        


};

} //namespace

#endif // __ROSEE_PARSER_MOVEIT_H
