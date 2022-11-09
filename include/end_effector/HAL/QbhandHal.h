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

#ifndef __ROSEE_QBHAND_HAL__
#define __ROSEE_QBHAND_HAL__

#include <ros/ros.h>

#include <end_effector/HAL/EEHal.h>

#include <serial/serial.h>
#include <qbrobotics_research_api/qbsofthand2_research_api.h>

#include <string>
#include <memory>

namespace ROSEE {
    
    /**
     * @brief Class representing an end-effector
     * 
     */
    class QbhandHal : public EEHal {

    public:
        
        typedef std::shared_ptr<QbhandHal> Ptr;
        typedef std::shared_ptr<const QbhandHal> ConstPtr;
        
        QbhandHal( ros::NodeHandle* nh);
        virtual ~QbhandHal();
        
        virtual bool init() override;
        virtual bool sense() override;
        virtual bool move() override;
      
    private:
        
        std::map<std::string, std::unique_ptr<std::mutex>> serial_protectors_;  // only callbacks must lock the serial resources
        std::map<int, std::string> connected_devices_;
        
        double max_joint_pos_, min_joint_pos_;
        double close_hand_value_, open_hand_value_;
        
//         handlers to manage the communication with qbdevices
        std::shared_ptr<qbrobotics_research_api::Communication> communication_handler_;
        std::shared_ptr<qbrobotics_research_api::Communication> communication_handler_legacy_;
//         Communication ports
        std::vector<serial::PortInfo> serial_ports_;
        std::map<int, std::shared_ptr<qbrobotics_research_api::Device>> devices_;
//         IDs of connected devices 
        std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> device_ids_;
        
        int getPositions(const int &id, const int &max_repeats, std::vector<short int> &positions);
        int setCommandsAndWait(const int &id, const int &max_repeats, std::vector<short int> &commands);
        int setCommandsAsync(const int &id, std::vector<short int> &commands);

        int getSerialPortsAndDevices(const int &max_repeats);
        int close(const std::string &serial_port);
        int activate(const int &id, const bool &command, const int &max_repeats);
        int activate(const int &id, const int &max_repeats);
        int deactivate(const int &id, const int &max_repeats);
        
        int setControlMode(const int &id, const int &max_repeats, uint8_t &control_id);
        int isActive(const int &id, const int &max_repeats, bool &status);
    };
    
HAL_CREATE_OBJECT(QbhandHal)    
}

#endif // __ROSEE_QBHAND_HAL__
