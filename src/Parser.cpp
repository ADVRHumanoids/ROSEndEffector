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

#include <ROSEndEffector/Parser.h>

#include <ros/console.h>

#include <iostream>
#include <fstream> 

ROSEE::Parser::Parser ( const ros::NodeHandle& nh ) :
  _nh ( nh )
{

}


ROSEE::Parser::~Parser()
{

}

bool ROSEE::Parser::getROSEndEffectorConfig()
{
    bool success = true;
    
    // check if the ROSEE config path exists
    std::ifstream fin(_ros_ee_config_path);
    if (fin.fail()) {
        
        ROS_ERROR_STREAM("in " << __func__ << "! Can NOT open " << _ros_ee_config_path << "!");
        success = false;
    }
    else {
        // load the node for the _ros_ee_config_path
        YAML::Node cfg = YAML::LoadFile(_ros_ee_config_path);
        
        // find the internal node ROSEndEffector
        YAML::Node ros_ee_node;
        if(cfg["ROSEndEffector"]) {
            ros_ee_node = cfg["ROSEndEffector"];
        }
        else {
            ROS_ERROR_STREAM("in " << __func__ << " : YAML file  " << _ros_ee_config_path << " does not contain ROSEndEffector mandatory node!!");
            success = false;
        }
        
        // check the urdf_filename
        if(ros_ee_node["urdf_path"]) {
            // TBD relative path
            _urdf_path = ros_ee_node["urdf_path"].as<std::string>();
        }
        else {
            ROS_ERROR_STREAM("in " << __func__ << " : ROSEndEffector node of  " << _ros_ee_config_path << " does not contain urdf_path mandatory node!!");
            success = false;
        }

        // check the srdf_filename
        if(ros_ee_node["srdf_path"]) {
            // TBD relative path
            _srdf_path = ros_ee_node["srdf_path"].as<std::string>();
        }
        else {
            ROS_ERROR_STREAM("in " << __func__ << " : ROSEndEffector node of  " << _ros_ee_config_path << " does not contain srdf_path mandatory node!!");
            success = false;
        }
    }

    return success;
}


bool ROSEE::Parser::init()
{
    // try to retrive the path to config from the ROS param server
    if ( _nh.getParam("ros_ee_config_path", _ros_ee_config_path) ) {
        
        return getROSEndEffectorConfig();
    }
    
    return false;
    
}

bool ROSEE::Parser::init ( const std::string& path_to_cfg )
{
    _ros_ee_config_path = path_to_cfg;
    
    return getROSEndEffectorConfig();
}
