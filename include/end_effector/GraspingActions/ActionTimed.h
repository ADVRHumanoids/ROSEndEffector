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

#ifndef ROSEE_ACTIONTIMED_H
#define ROSEE_ACTIONTIMED_H

#include <vector>
#include <string>
#include <map>
#include <end_effector/GraspingActions/Action.h>
#include <end_effector/Utils.h>
#include <yaml-cpp/yaml.h>

namespace ROSEE {

/**
 * @brief An action composed by other ones that must be executed one after other with some wait time (also 0) in between. 
 * E.G. 0.000000second -----> Grasp -----> 0.1 + 0.1 second -----> OpenLid -----> 0.5second ---- and so on
 * This class contains all the joint position of each action (\ref actionsJointPosMap ) and some time margins which indicates 
 * wait time before and after the action (\ref actionsTimeMarginsMap). 
 * Each action inside is identified by its name, so no two action with same name can exist (see \ref insertAction )
 * After create the ActionTimed object, we can add actions with \ref insertAction().
 * As all other Action classes, it implements also functions to emit and parse in a yaml file
 * @todo identify inner actions with integer instead of given name?
 */
class ActionTimed : public Action
{
public:
    typedef std::shared_ptr<ActionTimed> Ptr;
    typedef std::shared_ptr<const ActionTimed> ConstPtr;
    /**
     * @brief Default constructor, used when parsing action from yaml file
     */
    ActionTimed();
    
    /**
     * @brief Costructor
     * @param actionName name of the action that will be created
     */
    ActionTimed(std::string actionName);

    /**
     * @brief Destructor
     */
    ~ActionTimed() {};
    
    /**
     * @brief Override this function is necessary because it is pure virtual in father class \ref Action. 
     * @return JointPos the joints positions of the last inserted action (the last one in the time line)
     */
    JointPos getJointPos () const override;
    
    /**
     * @brief Override function from father \ref Action. For this class, it returns the jointPos of all the 
     * inner actions that are inside this ActionTimed.
     * @return vector of JointPos of all the inner actions, in order of execution
     */
    std::vector < ROSEE::JointPos > getAllJointPos () const override;

    /**
     * @brief Get the JointsInvolvedCount maps of all the inner actions, in order
     * @return vector of JointsInvolvedCount map of inner actions
     */ 
    std::vector < ROSEE::JointsInvolvedCount > getAllJointCountAction() const;
    
    /**
     * @brief get all the time margins of all inner action
     * @return vector of pair : the first element of pair is the time to wait before
     *        executing the action, the second element the time to wait after.
     */
    std::vector < std::pair <double, double> > getAllActionMargins() const;
    
    /**
     * @brief get for joint positions
     * @param actionName the name of the inner action of which we want to know the JointPos
     * @return JointPos position of joints of \p actionName
     *         Return empty JointPos if \p actionName is not present in this ActionTimed
     */
    ROSEE::JointPos getJointPosAction ( std::string actionName ) const ;
    
    /**
     * @brief get for time margins
     * @param actionName the name of the inner action of which we want to know the time margins
     * @return a pair of positive double. The first is the time needed BEFORE the action, the secont the time AFTER
     *         return -1 -1 if the \p actionName was not present in this ActionTimed.
     */
    std::pair <double, double> getActionMargins ( std::string actionName ) const ;
    
    /**
     * @brief get for JointsInvolvedCount of the inner actions
     * @param actionName the name of the inner action of which we want to know the JointCount
     * @return JointsInvolvedCount of joints of \p actionName
     *         Return empty JointsInvolvedCount if \p actionName is not present in this
     *         ActionTimed
     */
    ROSEE::JointsInvolvedCount getJointCountAction ( std::string actionName ) const;

    /**
     * @brief getter for action that composed this one
     * @return vector of string of inner actions, ordered by time execution
     */
    std::vector <std::string> getInnerActionsNames() const ;
    /**
     * @brief Print info about this action
     */
    void print () const override;
    
    /**
     * @brief Emit info in a file with yaml format
     * @param out a YAML::Emitter& object to emit the infos
     */    
    void emitYaml ( YAML::Emitter& out) const override;
    
    /**
     * @brief Fill the internal data with infos taken from yaml file. 
     * @param yamlIt a yamlt iterator to a node which has loaded the file
     * @return false if some error happened
     */
    bool fillFromYaml(  YAML::const_iterator yamlIt ) override;
    
    /** 
     * @brief Insert an action as last one in the time line 
     * @param action pointer to the action to be inserted
     * @param marginBefore the time margin to wait before executing the \p action 
     * @param marginAfter the time margin to wait after executing the \p action 
     * @param jointPosIndex (default == 0) the wanted jointPos of \p action to insert. Error the index is greater than the number
     *      of joint pos in the \p action. First element has index 0. 
     * @param percentJointPos (default == 1) OPTIONAL argument to scale all the joint position of the \p action that is being inserted
     * @param newActionName (default == "") OPTIONAL argument if we want to store the \p action with a different name
     * @return False if some error happened
     * @warning We can't have inned actions with same name. So, if \p action name (or \p newActionName) is already present, 
     * the action is not inserted and the function returns false. Being \ref Action names not changeable, to solve this 
     * we can pass the \p newActionName argument to this function. If it will be inserted, it will be referenced with this new name
     * @note We take only necessary infos from \p action and store them in the members of ActionTimed. There is not way to go
     * back to the original inserted action from an ActionTimed
     */
    bool insertAction ( ROSEE::Action::Ptr action, double marginBefore = 0.0, double marginAfter = 0.0, 
                        unsigned int jointPosIndex = 0, double percentJointPos = 1, std::string newActionName = "");
    
private:
    std::map <std::string, std::pair<double, double> > actionsTimeMarginsMap;
    std::map <std::string, ROSEE::JointPos> actionsJointPosMap;
    std::map <std::string, ROSEE::JointsInvolvedCount> actionsJointCountMap;
    
    /**
     * Here is contained the wanted final position of the timed action.
     * So, it is the sum of all the wanted joint position of all the inner actions
     */
    ROSEE::JointPos jointPosFinal;
    
    /**
     * This vector is used to take count of the order of the actions inserted
     */
    std::vector < std::string > actionsNamesOrdered;

};

}

#endif // ROSEE_ACTIONTIMED_H
