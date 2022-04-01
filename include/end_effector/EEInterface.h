/*
 * Copyright (C) 2019 IIT-HHCM
 * Author: Luca Muratore
 * email:  luca.muratore@iit.it
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef __ROSEE_EE_INTERFACE__
#define __ROSEE_EE_INTERFACE__

#include <map>
#include <vector>
#include <memory>

#include <end_effector/Parser.h>

#include <Eigen/Dense>


namespace ROSEE {
    
    /**
     * @brief Class representing and End-Effector
     * 
     */
    class EEInterface {

    public:
        
        typedef std::shared_ptr<EEInterface> Ptr;
        typedef std::shared_ptr<const EEInterface> ConstPtr;
        
        EEInterface( const ROSEE::Parser& p );
        //EEInterface ( const EEInterface& other );
        //EEInterface& operator= ( const EEInterface& p );
        virtual ~EEInterface();
        
        /**
         * @brief getter for the fingers name in the current End-Effector
         * 
         * @return std::vector<std::string>& existing finger names in the End-Effector
         */
        const std::vector<std::string>& getFingers();
        
        /**
         * @brief check is the requested finger exists
         * 
         * @param finger_name name of the finger to check
         * @return true if the finger exists, false otherwise
         */
        bool isFinger( std::string finger_name );
        
        /**
         * @brief getter for the actuated joints of the end-effector
         * 
         * @return const std::vector< std::string >& the actuated joint in the End-Effector as a vector of String
         */
        const std::vector<std::string>& getActuatedJoints();
        
        /**
         * @brief getter for the actuated joints of a finger
         * 
         * @param finger_name the name of the finger where to retrieve actuated joints
         * @param actuated_joints [out] will be filled with the actuated joints of the finger
         * @return bool true if the requested finger exists, false otherwise
         */
        bool getActuatedJointsInFinger( std::string finger_name, std::vector<std::string>& actuated_joints );
        
        /**
         * @brief getter for the number of actuated joints in the end-effector
         * 
         * @return int the number of actuated joints in the end effector
         */
        int getActuatedJointsNum();
        
        /**
         * @brief getter for the number of actuated joints in the requested finger
         * 
         * @param finger_name the name of the finger where to retrieve the number of actuated joints
         * @return int the number of actuated joints in finger_name, or -1 if the finger does not exists
         */
        int getActuatedJointsNumInFinger(  std::string finger_name );
        
        /**
         * @brief getter for the upper position limits of EE joints as specified in the URDF
         * 
         * @return Eigen::VectorXd the upper position limits of EE joints as specified in the URDF
         */
        Eigen::VectorXd getUpperPositionLimits();
        
        /**
         * @brief getter for the lower position limits of EE joints as specified in the URDF
         * 
         * @return Eigen::VectorXd the lower position limits of EE joints as specified in the URDF
         */
        Eigen::VectorXd getLowerPositionLimits();
        
        /**
         * @brief getter for the internal ids (position in the EEInterface vectors) of joints in a certain finger
         * 
         * @param finger_name the name of the requested finger
         * @param internal_ids the internal ids of the joints in the requested finger
         * @return bool true if the finger exists, false otherwise
         */
        bool getInternalIdsForFinger( std::string finger_name, std::vector< int >& internal_ids );
        
        /**
         * @brief getter for the internal id of a certain joint
         * 
         * @param joint_name the name of the requested joint
         * @param internal_id the internal id of the requested joint
         * @return bool true if the joint exists, false otherwise
         */
        bool getInternalIdForJoint ( std::string joint_name, int& internal_id );
        
        /**
         * @brief getter for the number of fingers in the End-Effector
         * 
         * @return int the number of fingers in the End-Effector
         */
        int getFingersNumber();
        
        /**
         * @brief getter for the EE name
         * 
         * @return std::string the EE name parsed from the given config files of the EE
         */
        std::string getName();
        
    private:
        
        std::map<std::string, std::vector<std::string>> _ee_description;
        std::map<std::string, urdf::JointConstSharedPtr> _urdf_joint_map;
        
        std::map<std::string, std::vector<int>> _finger_joints_internal_id_map;
        
        std::map<std::string, int> _joints_internal_id_map;
        
        std::vector<std::string> _fingers_names, _actuated_joints;
        
        Eigen::VectorXd _upper_limits, _lower_limits;
        
        std::string _ee_name;
        
        int _joints_num = 0;
        
        
        
    };
    
}

#endif // __ROSEE_EE_INTERFACE__
