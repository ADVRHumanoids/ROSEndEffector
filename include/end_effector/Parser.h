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

#ifndef __ROSEE_PARSER__
#define __ROSEE_PARSER__


#include <iostream>
#include <fstream>
#include <memory>

#include <ros/ros.h>
#include <ros/console.h>

#include <end_effector/Utils.h>

#include <yaml-cpp/yaml.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <srdfdom/model.h>

//to find relative path for the config files
#include <boost/filesystem/path.hpp>

namespace ROSEE {
    
    /**
     * @brief Class responsible for parsing the YAML file providing
     * the URDF, SRDF, the EE-HAL library implementation and the EEInterface
     * 
     */
    class Parser {
            
    public:
        
        typedef std::shared_ptr<Parser> Ptr;
        typedef std::shared_ptr<const Parser> ConstPtr;

        Parser( const ros::NodeHandle& nh );
        //Parser ( const Parser& other );
        //Parser& operator= ( const Parser& p );
        virtual ~Parser();

        /**
         * @brief Initialization function using the ROS param ROSEEConfigPath
         * 
         * @return bool true for success, false otherwise
         */
        bool init ();
        
        /**
         * @brief Initialization function using the path_to_cfg parameter for the path
         * 
         * @param urdf_path path to the urdf file
         * @param srdf_path path to the srdf file
         * @param action_path path to the action folder
         * @return bool true for success, false otherwise
         */
        bool init (const std::string& urdf_path, const std::string& srdf_path, const std::string& action_path );

        
        /**
         * @brief getter for the URDF information of the joints of the End-Effector
         * 
         * @return std::map<std::string, urdf::JointConstSharedPtr> map between the joint name and the URDF info of the joint
         */
        std::map<std::string, urdf::JointConstSharedPtr> getUrdfJointMap() const;
        
        /**
         * @brief getter for a description of the End-Effector as a map of finger name, finger joint names. There exists another version to get the map as reference: 
         * \ref getFingerJointMapAsReference
         * 
         * @return std::map<std::string, std::vector<std::string>> a map respresenting an End-Effector with a key representing the finger name 
         * and the values representing a vector of joint names.
         */
        std::map<std::string, std::vector<std::string>> getFingerJointMap() const;
        
        /**
         * @brief getter for the map between the finger kinematic chains and the related  actuated joints, reference version of \ref getFingerJointMap
         * 
         * @param finger_joint_map a map between the finger kinematic chains and the related actuated joints
         * @return void
         */
        void getFingerJointMap(std::map<std::string, std::vector<std::string>>& finger_joint_map) const;
        
        /**
         * @brief getter for a description of the End-Effector as a map of joint name, 
         * finger name
         * 
         * @return std::map<std::string, std::string> a map respresenting an End-Effector with a key representing the joint name
         * and the value representing the finger which the joint belongs to
         */
        std::map<std::string, std::string> getJointFingerMap() const;
        
        /**
         * @brief getter for a description of the End-Effector as a map of joint name, 
         * finger name
         * 
         * @param joint_finger_map a map between the joint and the related finger kinematic chain
         * 
         * @return void
         */
        void getJointFingerMap(std::map<std::string, std::string>& joint_finger_map) const;
        
        /**
         * @brief getter for the configure End-Effector name
         * 
         * @return std::string the End-Effector name as reported in the URDF robot name
         */
        std::string getEndEffectorName() const;
        
        /**
         * @brief getter for the total number of actuated joints in the configuration files
         * 
         * @return int the number of actuated joints in the End-Effector
         */
        int getActuatedJointsNumber() const;
        
        /**
         * @brief Utility to print the mapping between the End Effector finger chains and the related actuated joints
         * 
         * @return void
         */
        void printEndEffectorFingerJointsMap() const;
        
        /**
         * @brief get the whole urdf file parsed as a string
         *
         * @return a string containg the urdf file parsed
         */     
        std::string getUrdfString() const;
        
        /**
         * @brief get the whole srdf file parsed as a string
         *
         * @return a string containg the srdf file parsed
         */    
        std::string getSrdfString() const;
        
        /**
         * @brief get the path where emit and parse grasping actions
         *
         * @return the path as a string
         */    
        std::string getActionPath() const;
        
        /**
         * @brief get the filename (with path) of the yaml config file. 
         *   Useful get to print infos about file parsed outside this class
         * 
         * @return a string that is the filepath of yaml config file
         */
        std::string getRoseeConfigPath() const;
        
    private:
        
        ros::NodeHandle _nh;
        std::string _urdf_path, _srdf_path, _urdf_string, _srdf_string;
        std::string _action_path;
        bool _is_initialized = false;
        
        urdf::ModelInterfaceSharedPtr _urdf_model;
        KDL::Tree _robot_tree;
        
        srdf::Model _srdfdom;
        
        int _fingers_num = 0;
        std::vector<std::string> _fingers_names;
        std::vector<int> _fingers_group_id;
        
        int _joints_num = 0;
        
        std::map<std::string, std::vector<std::string>> _finger_joint_map;
        std::map<std::string, std::string> _joint_finger_map;
        std::map<std::string, urdf::JointConstSharedPtr> _urdf_joint_map;
        
        
        /**
         * @brief configure the ROSEE parser based on the configration files requested
         * 
         * @return bool true on success, false otherwise
         */
        bool configure();
        
        /**
         * @brief Function responsible to get the file needed to fill the internal data structure of the Parser
         * with data coming from the ros_ee configuration file requested by the user
         * 
         * @return bool true on success, false if the config file contains 
         * errors (either in the main YAML, or in URDF, or in SRDF or in EE-HAL library path.
         * or in the Grasping Primitive Definition).
         */
        //bool getROSEndEffectorConfig();
        
        /**
         * @brief Function responsible to parse the URDF data
         * 
         * @return bool true if the URDF requested exists, false otherwise
         */
        bool parseURDF();
        
        /**
         * @brief Function responsible to parse the SRDF data
         * 
         * @return bool true if the SRDF requested exists, false otherwise
         */
        bool parseSRDF();
        
        /**
         * 
         */
        void addNotInFingerJoints();
        
        /**
         * @brief Function to remove the passive joints from the filled maps
         * 
         * @return bool is everything works correctly
         */
        bool removePassiveJoints();
        
        /**
         * @brief fill a data structure related with the revolute/prismatic
         * joints included in between base_link and tip_link in the requested URDF
         * 
         * @param base_link base link of the finger chain
         * @param tip_link tip link of the finger chain
         * @return bool true if the chain is found in the URDF, false otherwise
         */
        bool getJointsInFinger(std::string base_link, std::string tip_link, std::string finger_name);

        
    };
}

#endif // __ROSEE_PARSER__
