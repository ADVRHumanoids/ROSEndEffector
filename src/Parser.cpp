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

#define CHAIN_PER_GROUP 1

ROSEE::Parser::Parser ( const ros::NodeHandle& nh ) :
    _nh ( nh ) {

}


ROSEE::Parser::~Parser() {

}

bool ROSEE::Parser::getJointsInFinger ( std::string base_link,
                                        std::string tip_link,
                                        std::string finger_name
                                      ) {

    KDL::Chain actual_chain;
    if ( _robot_tree.getChain ( base_link, tip_link, actual_chain ) ) {

        int segments_num = actual_chain.getNrOfSegments();
        for ( int i = 0; i < segments_num; i++ ) {

            KDL::Segment actual_segment = actual_chain.getSegment ( i );
            KDL::Joint actual_joint = actual_segment.getJoint();

            bool is_valid_joint = false;

            // Check if joint is revolute or prismatic
            is_valid_joint = actual_joint.getTypeName() == "RotAxis";   
            
//                              || actual_joint.getTypeName() == "TransAxis";

            // if the joint is revolute or prismatic
            if ( is_valid_joint ) {

                _finger_joint_map[finger_name].push_back ( actual_joint.getName() );
                _urdf_joint_map[actual_joint.getName()] = _urdf_model->getJoint ( actual_joint.getName() );

                _joints_num++;

                ROS_INFO_STREAM ( actual_joint.getName() );
            }
        }

        return true;
    }

    ROS_ERROR_STREAM ( "chain from base_link " << base_link << " to tip_link " << tip_link << " not found in the URDF" );

    return false;
}



bool ROSEE::Parser::parseSRDF() {

    // initialize srdfdom
    _srdfdom.initFile ( *_urdf_model, _srdf_path );

    // get the end-effectors in the SRDF file
    std::vector<srdf::Model::EndEffector> srdf_end_effectors = _srdfdom.getEndEffectors();

    // NOTE only one end-effectors supported
    // TBD support multiple end-effectors
    if ( srdf_end_effectors.size() != 1 ) {

        return false;
    }

    ROS_INFO_STREAM ( "ROSEndEffector Parser found end_effector: " << srdf_end_effectors.at ( 0 ).name_ );

    // get all the groups in the SRDF
    std::vector<srdf::Model::Group> srdf_groups = _srdfdom.getGroups();

    // TBD will be a vector
    srdf::Model::Group fingers_group;
    // find end effector fingers chains group
    // TBD will be a vector
    int group_num = srdf_groups.size();
    // TBD will be a vector
    std::string end_effector_group_name = srdf_end_effectors.at ( 0 ).component_group_;
    for ( int i = 0; i < group_num; i++ ) {
        if ( srdf_groups[i].name_ == end_effector_group_name ) {
            fingers_group = srdf_groups[i];
            ROS_INFO_STREAM ( "ROSEndEffector Parser found group: " << end_effector_group_name << " in the SRDF with the following fingers: " );
        }
    }

    _fingers_num = fingers_group.subgroups_.size();
    _fingers_names.resize ( _fingers_num );
    _fingers_group_id.resize ( _fingers_num );

    // fill the chain names vector
    for ( int i = 0; i < _fingers_num; i++ ) {

        _fingers_names[i] = fingers_group.subgroups_[i];

        // find the id of the current finger group
        for ( int j = 0; j < group_num; j++ ) {
            if ( srdf_groups[j].name_ == _fingers_names[i] ) {
                _fingers_group_id[i] = j;
            }
        }
        ROS_INFO_STREAM ( _fingers_names[i] );
    }

    // iterate over the fingers and find revolute joints
    for ( int i = 0; i < _fingers_num; i++ ) {
        srdf::Model::Group current_finger_group = srdf_groups[ _fingers_group_id[i] ];

        ROS_INFO_STREAM ( "Actuated joints in finger: " << current_finger_group.name_ );

        // NOTE only one chain per group
        if ( current_finger_group.chains_.size() != CHAIN_PER_GROUP )  {

            ROS_ERROR_STREAM ( "for the finger chain groups you can specify only one chain per group in your SRDF check " << current_finger_group.name_.c_str() << "group" );
            return false;
        }

        // fill the enabled/disabled joints in chain map
        if ( !getJointsInFinger ( current_finger_group.chains_[0].first,
                                  current_finger_group.chains_[0].second,
                                  current_finger_group.name_
                                ) ) {
            return false;
        }
    }

    return true;

}


bool ROSEE::Parser::parseURDF() {

    std::string xml_string;
    std::fstream xml_file ( _urdf_path.c_str(), std::fstream::in );
    if ( xml_file.is_open() ) {

        while ( xml_file.good() ) {

            std::string line;
            std::getline ( xml_file, line );
            xml_string += ( line + "\n" );
        }

        xml_file.close();
        _urdf_model = urdf::parseURDF ( xml_string );

        // create the robot KDL tree from the URDF model
        if ( !kdl_parser::treeFromUrdfModel ( *_urdf_model, _robot_tree ) ) {

            ROS_ERROR_STREAM ( "in " << __func__ << " Failed to construct kdl tree" );
            return false;
        }

        // save urdf and string (can be useful to have, thanks @alaurenzi!)
        std::ifstream t_urdf ( _urdf_path );
        std::stringstream buffer_urdf;
        buffer_urdf << t_urdf.rdbuf();
        _urdf_string = buffer_urdf.str();

        return true;

    } else {

        ROS_ERROR_STREAM ( "in " << __func__ << " : Can NOT open " << _urdf_path << " !" );
        return false;
    }
}


bool ROSEE::Parser::getROSEndEffectorConfig() {

    bool success = true;

    // check if the ROSEE config path exists
    std::ifstream fin ( _ros_ee_config_path );
    if ( fin.fail() ) {

        ROS_ERROR_STREAM ( "in " << __func__ << " : Can NOT open " << _ros_ee_config_path << "!" );
        success = false;
    } else {

        // load the node for the _ros_ee_config_path
        YAML::Node cfg = YAML::LoadFile ( _ros_ee_config_path );

        // find the internal node ROSEndEffector
        YAML::Node ros_ee_node;
        if ( cfg["ROSEndEffector"] ) {

            ros_ee_node = cfg["ROSEndEffector"];
        } else {

            ROS_ERROR_STREAM ( "in " << __func__ << " : YAML file  " << _ros_ee_config_path << " does not contain ROSEndEffector mandatory node!!" );
            success = false;
        }

        // check the urdf_filename
        if ( ros_ee_node["urdf_path"] ) {

            // TBD relative path
            _urdf_path = ros_ee_node["urdf_path"].as<std::string>();
            ROS_INFO_STREAM ( "ROSEndEffector Parser found URDF path: " << _urdf_path );
        } else {

            ROS_ERROR_STREAM ( "in " << __func__ << " : ROSEndEffector node of  " << _ros_ee_config_path << " does not contain urdf_path mandatory node!!" );
            success = false;
        }

        // check the srdf_filename
        if ( ros_ee_node["srdf_path"] ) {

            // TBD relative path
            _srdf_path = ros_ee_node["srdf_path"].as<std::string>();
            ROS_INFO_STREAM ( "ROSEndEffector Parser found SRDF path: " << _srdf_path );
        } else {

            ROS_ERROR_STREAM ( "in " << __func__ << " : ROSEndEffector node of  " << _ros_ee_config_path << " does not contain srdf_path mandatory node!!" );
            success = false;
        }
    }

    return success;
}

bool ROSEE::Parser::configure() {

    bool ret = true;
    if ( getROSEndEffectorConfig() ) {

        if ( parseURDF() && parseSRDF() ) {

            // build the EEInterface
//             _ee_interface = std::make_shared<ROSEE::EEInterface>( this );
            ROS_INFO_STREAM ( "ROSEndEffector Parser successfully configured using config file:  " << _ros_ee_config_path );
        } else {

            ret = false;
        }

    } else {

        ret = false;
    }


    return ret;
}



bool ROSEE::Parser::init() {

    // try to retrive the path to config from the ROS param server
    if ( _nh.getParam ( "ros_ee_config_path", _ros_ee_config_path ) ) {

        _is_initialized =  configure();
        return _is_initialized;
    }

    // error
    ROS_ERROR_STREAM ( "in " << __func__ << " : ros_ee_config_path not found on ROS parameter server" );
    return false;

}

bool ROSEE::Parser::init ( const std::string& path_to_cfg ) {

    _ros_ee_config_path = path_to_cfg;

    _is_initialized =  configure();
    return _is_initialized;
}

void ROSEE::Parser::printEndEffectorFingerJointsMap() const {

    if ( _is_initialized ) {

        ROS_INFO_STREAM ( "ROS End Effector: Finger Joints Map" );
        ROS_INFO_STREAM ( "-------------------------" );
        for ( auto& chain_joints: _finger_joint_map ) {
            ROS_INFO_STREAM ( chain_joints.first );

            for ( int i = 0; i <  chain_joints.second.size(); i++ ) {
                ROS_INFO_STREAM ( chain_joints.second.at ( i ) );
            }

            ROS_INFO_STREAM ( "-------------------------" );
        }
    } else {

        ROS_ERROR_STREAM ( "in " << __func__ << " :  ROSEE::Parser needs to be initialized. Call init() frist." );
    }
}

int ROSEE::Parser::getActuatedJointsNumber() const {
    
    return _joints_num;
}



std::map< std::string, std::vector< std::string > > ROSEE::Parser::getFingerJointMap() const {

    return _finger_joint_map;
}

std::map< std::string, urdf::JointConstSharedPtr > ROSEE::Parser::getUrdfJointMap() const {

    return _urdf_joint_map;
}

void ROSEE::Parser::getActuatedJointsMap ( std::map< std::string, std::vector< std::string > >& finger_joint_map ) {

    finger_joint_map = _finger_joint_map;
}



