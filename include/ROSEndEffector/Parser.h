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


#include <memory>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace ROSEE {
    
    
    /**
     * @brief Class responsible for parsing the YAML file providing
     * the URDF, SRDF, the EE-HAL library implementation and the 
     * Grasping Primitive Definition (GPD).
     * 
     */
    class Parser {
    
        typedef std::shared_ptr<Parser> Ptr;
        typedef std::shared_ptr<const Parser> ConstPtr;
        
    public:
        
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
         * @param path_to_cfg path to the config file
         * @return bool true for success, false otherwise
         */
        bool init (const std::string& path_to_cfg);
        
    private:
        
        ros::NodeHandle _nh;
        std::string _ros_ee_config_path, _urdf_path, _srdf_path;
        
        /**
         * @brief Function responsble to fill the internal data structure of the Parser
         * with data coming from the ros_ee configuration file requested by the user
         * 
         * @return bool true on success, false if the config file contains 
         * errors (either in the main YAML, or in URDF, or in SRDF or in EE-HAL library path.
         * or in the Grasping Primitive Definition).
         */
        bool getROSEndEffectorConfig();
        
    
    };
}

#endif // __ROSEE_PARSER__