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

#ifndef __ROSEE_EE_HAL__
#define __ROSEE_EE_HAL__


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <fstream>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

//#include <ros_end_effector/Utils.h>
#include <ros_end_effector/UtilsYAML.h>

//Macro to be used in each concrete HAL that will define the create_object functions
#define HAL_CREATE_OBJECT(className) \
    extern "C" ::ROSEE::EEHal* create_object_##className ( ros::NodeHandle* nh) { \
        return new className(nh); \
    } \

namespace ROSEE {
    
    
    /**
     * @brief Class representing an end-effector
     * 
     */
    class EEHal  {

    public:
        
        typedef std::shared_ptr<EEHal> Ptr;
        typedef std::shared_ptr<const EEHal> ConstPtr;
        
        EEHal ( ros::NodeHandle* nh );
        virtual ~EEHal() {};
                
        virtual bool sense() = 0;
        
        virtual bool move() = 0;
        
        virtual bool publish_joint_state();
        
        //virtual bool setMotorPositionReference(std::string motor, double pos) = 0;
        //virtual bool getJointPosition(std::string joint, std::vector<double> &pos ) = 0;
                        
        //Matrices needed by Grasp Planner ROSEE
        virtual bool getFingersNames(std::vector<std::string> &fingers_names);
        virtual bool getMotorsNames(std::vector<std::string> &motors_names);
        virtual bool getTipsJacobians(std::unordered_map<std::string, Eigen::MatrixXd>& tips_jacobian);
        virtual bool getTransmissionMatrix(Eigen::MatrixXd &transmission_matrix) ;
        virtual bool getMotorStiffnessDiagonal(Eigen::VectorXd &motors_stiffness_diagonal);
        virtual bool getTipsFrictions(Eigen::VectorXd &tips_friction);
        virtual bool getTipsForceLimits(Eigen::VectorXd &tips_force_limits);
        virtual bool getMotorTorqueLimits(Eigen::VectorXd &motors_torque_limits);
        
        virtual bool parseHandInfo();
        

    protected:
        
        ros::NodeHandle* _nh; 
        
        /**
         * The references that must be read in the move() to send to the real/simulated robot
         * TODO put private and create a get ? (no set)
         */
        sensor_msgs::JointState _mr_msg;
        ros::Subscriber _motor_reference_sub;
        
        /**
         * The states that must be filled in the sense(), reading info from real/simulated robot
         * TODO put private and create a set (no get) ?
         */
        sensor_msgs::JointState _js_msg;
        ros::Publisher _joint_state_pub;
        
        /**** Hand info matrices***/
        std::vector <std::string> fingers_names, motors_names;
        std::unordered_map<std::string, Eigen::MatrixXd> tips_jacobians;
        Eigen::MatrixXd transmission_matrix, motors_stiffness_diagonal;
        Eigen::VectorXd tips_frictions, tips_force_limits, motors_torque_limits;
        
    private:
        
        void motor_reference_clbk(const sensor_msgs::JointState::ConstPtr& msg);
    };
}

#endif // __ROSEE_EE_HAL__
