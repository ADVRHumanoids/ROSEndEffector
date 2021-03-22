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
#include <ros_end_effector/Utils.h>
#include <ros_end_effector/UtilsEigen.h>

#include <ros_end_effector/UtilsYAML.h>

#include <rosee_msg/HandInfo.h>
#include <rosee_msg/MotorPhalangePressure.h>

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
        
        virtual bool checkCommandPub();
        
        //virtual bool setMotorPositionReference(std::string motor, double pos) = 0;
        //virtual bool getJointPosition(std::string joint, std::vector<double> &pos ) = 0;
                        
        //Matrices needed by Grasp Planner ROSEE
        virtual bool getFingersNames(std::vector<std::string> &fingers_names);
        virtual bool getMotorsNames(std::vector<std::string> &motors_names);
        
        virtual bool getMotorStiffness(Eigen::VectorXd &motors_stiffness);
        virtual bool getTipsFrictions(Eigen::VectorXd &tips_friction);
        virtual bool getMotorTorqueLimits(Eigen::VectorXd &motors_torque_limits);
        virtual bool getTipJointToTipFrameX(Eigen::VectorXd &tip_joint_to_tip_frame_x);
        virtual bool getTipJointToTipFrameY(Eigen::VectorXd &tip_joint_to_tip_frame_y);
        
        // These are dependent on hand configuration, hence can not be simply parsed by conf file so
        // each derived HAL must implement them with some logic if they want to use the planne
        virtual bool getTipsJacobiansNormal(std::unordered_map<std::string, Eigen::MatrixXd>& tips_jacobian_normal) {return false;}
        virtual bool getTipsJacobiansFriction(std::unordered_map<std::string, Eigen::MatrixXd>& tips_jacobian_friction) {return false;}
        virtual bool getTransmissionMatrix(Eigen::MatrixXd &transmission_matrix) {return false;}
        virtual bool getTransmissionSquareMatrices(std::unordered_map<std::string, Eigen::MatrixXd>& transmission_square_matrices) {return false;}
        virtual bool getTransmissionAugmentedMatrices(std::unordered_map<std::string, Eigen::MatrixXd>& transmission_augmented_matrices) {return false;}
        virtual bool getTipsForcesNormal(Eigen::VectorXd& tips_forces_normal) {return false;}
        virtual bool getTipsForcesFriction(Eigen::VectorXd& tips_forces_friction) {return false;}

        virtual bool parseHandInfo();
        
        virtual bool isHandInfoPresent();
        
        /**
         * @brief init the service message with info parsed from yaml file, ie info that will not change according to hand configuration
         * A derived HAL can override this if necessary, since this is called by EEHalExecutor
         */
        virtual bool init_hand_info_response() ;
        
        /**
         * @brief Here service is advertise and callback set: if derived class wants to use different callback,
         *   they must override this and do :
         *         _hand_info_server = _nh->advertiseService(_hand_info_service_name, <custom_callback>);
         */
        virtual bool setHandInfoCallback();
        
        bool _pressure_active;
        bool publish_pressure();

        Eigen::VectorXd getMotorReference() const;
        Eigen::VectorXd getJointPosition() const;
        Eigen::VectorXd getJointEffort() const;
        Eigen::MatrixXd getPressure() const;


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
        
        //Heri specific message
        bool initPressureSensing();
        rosee_msg::MotorPhalangePressure _pressure_msg;
        ros::Publisher _pressure_pub;

        
        /**** Hand info matrices***/
        std::vector <std::string> fingers_names, motors_names;
        Eigen::VectorXd tips_frictions, motors_torque_limits, motors_stiffness,
                        tip_joint_to_tip_frame_x, tip_joint_to_tip_frame_y;
        
        ros::ServiceServer _hand_info_server;
        rosee_msg::HandInfo::Response _hand_info_response;
        std::string _hand_info_service_name;

        
    private:
        
        bool _hand_info_present;
        
        bool handInfoEEHalCallback(
            rosee_msg::HandInfo::Request& request,
            rosee_msg::HandInfo::Response& response);

        
        void motor_reference_clbk(const sensor_msgs::JointState::ConstPtr& msg);
    };
}

#endif // __ROSEE_EE_HAL__
