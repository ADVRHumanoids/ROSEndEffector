/*
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

#include <end_effector/HAL/QbhandHal.h>

ROSEE::QbhandHal::QbhandHal ( ros::NodeHandle *nh) : EEHal ( nh ) {
    
    communication_handler_ = std::make_shared<qbrobotics_research_api::Communication>();
    communication_handler_legacy_ = std::make_shared<qbrobotics_research_api::CommunicationLegacy>(*communication_handler_);
    
    max_joint_pos_ = 1;
    min_joint_pos_ = 0;
    close_hand_value_ = 19000;
    open_hand_value_ = 0;
    
}

ROSEE::QbhandHal::~QbhandHal() { 
    
    //deactivate motors
    for (const auto& dev_pair : connected_devices_) {
        if (deactivate(dev_pair.first, 3) >= 3) {
            
            std::cout << "Error in deactivating device " << dev_pair.first << std::endl;
            
        } else {
            std::cout << "Device " << dev_pair.first << " deactived!" << std::endl;
        }
    }

}

bool ROSEE::QbhandHal::init () {
    
    while (!getSerialPortsAndDevices(3)) {
        ROS_WARN_STREAM_NAMED("communication_handler", "[CommunicationHandler] is waiting for devices...");
        ros::Duration(1.0).sleep();
    }
    
    //activate motors
    for (const auto& dev_pair : connected_devices_) {
        if (activate(dev_pair.first, 3) >= 3) {
            std::cout << "Error in activating device " << dev_pair.first << std::endl;
            return false;
        } 
        std::cout << "Device " << dev_pair.first << " active!" << std::endl;
    }
    
    //control mode 
    uint8_t control_id = 0; //position mode
    for (const auto& dev_pair : connected_devices_) {
        if (setControlMode(dev_pair.first, 3, control_id) >= 3) {
            std::cout << "Error in activating device " << dev_pair.first << std::endl;
            return false;
        } 
        std::cout << "Device " << dev_pair.first << " active!" << std::endl;
    }
    
    return true;
}


bool ROSEE::QbhandHal::sense() {

    //TODO maybe
    //failures = isActive(devices.first, max_repeats, status);
    
    std::vector<short int> positions(1);
    
    for (const auto& dev_pair : connected_devices_) {
        getPositions(dev_pair.first, 3, positions);
    }
    
    double old_range = (close_hand_value_ - open_hand_value_);  
    double new_range = (max_joint_pos_ - min_joint_pos_);  
    
    _js_msg.position[0] = (((_mr_msg.position[0] - open_hand_value_) * new_range) / old_range) + min_joint_pos_;
    _js_msg.header.stamp = ros::Time::now();
    _js_msg.header.seq =_js_msg.header.seq+1;
    
    return true;
}

bool ROSEE::QbhandHal::move() {
    
    std::vector<short int> commands(1);
    
    double old_range = (max_joint_pos_ - min_joint_pos_);  
    double new_range = (close_hand_value_ - open_hand_value_);  
    double command = (((_mr_msg.position[0] - min_joint_pos_) * new_range) / old_range) + open_hand_value_;
    
    commands.at(0) = std::round(command);
    
    for (const auto& dev_pair : connected_devices_) {
        setCommandsAndWait(dev_pair.first, 3, commands);
        //setCommandsAsync(dev_pair.first, 3, commands);
    }

    return true;
}



int ROSEE::QbhandHal::getSerialPortsAndDevices(const int &max_repeats) {
  // clear devices and serial vectors/maps
  serial_ports_.clear();
  device_ids_.clear();
  devices_.clear();
  if (communication_handler_->listSerialPorts(serial_ports_) <= 0) {
      ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] No serial ports found!");
      return -1;
  }
  int devices_retrieved;
  ros::Duration(1).sleep();
  for(auto &serial_port:serial_ports_){ // scan and open all the serial port
    try {
      devices_retrieved = communication_handler_->listConnectedDevices(serial_port.serial_port, device_ids_);
    } catch(serial::SerialIOException &/*exc_name*/) {
      ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] No qbrobotics device(s) connected!");
      return -1;
    }
    serial_protectors_.insert(std::make_pair(serial_port.serial_port, std::make_unique<std::mutex>()));  // never override
    if (devices_retrieved > 0) { // retrieved at least a qbrobotics device
      ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] Found "<< devices_retrieved << " qbrobotics devices on [" << serial_port.serial_port << "]");
      for(auto &device_id:device_ids_) {
        if (device_id.id == 120 || device_id.id == 0) {
          continue;  // ID 120 is reserved for dummy board which should not be considered as a connected device (ID 0 is for sure an error)
        }
        int failures = 0;
        while (failures <= max_repeats) {
          try { // TODO: differentiate the devices
            if (device_id.type == "001" || device_id.type == "006") {  // device S/N can be retireved only from new device firmware. They can use communication_handler_
              devices_.insert(std::make_pair(static_cast<int>(device_id.id), std::make_shared<qbrobotics_research_api::Device>(communication_handler_, "dev", serial_port.serial_port, device_id.id)));
              connected_devices_.insert(std::make_pair(static_cast<int>(device_id.id), serial_port.serial_port));
              ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] Connected to device with id: "<< (int)device_id.id);
              break;
            } else {
              communication_handler_legacy_ = std::make_shared<qbrobotics_research_api::CommunicationLegacy>(*communication_handler_);
              devices_.insert(std::make_pair(static_cast<int>(device_id.id), std::make_shared<qbrobotics_research_api::Device>(communication_handler_legacy_, "dev", serial_port.serial_port, device_id.id)));
              connected_devices_.insert(std::make_pair(static_cast<int>(device_id.id), serial_port.serial_port));
              ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] The device with id " << (int)device_id.id << " is connected.");
              break;
            }
          } catch(...) {
            ros::Duration(0.1).sleep();
            failures++;
          }
        }
        if(failures > max_repeats){
          ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] cannot connect to qbrobotics device(s) after "<< max_repeats << " attempt(s)");
          close(serial_port.serial_port);
          return -1;
        }
      }
    } else {
      ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] no qbrobotics devices found on [" << serial_port.serial_port << "]");
      close(serial_port.serial_port);
      return -1;
    }
  }
  return devices_retrieved;
}

int ROSEE::QbhandHal::close(const std::string &serial_port) {
  for (auto const &device : connected_devices_) {
    if (device.second == serial_port) {
      deactivate(device.first, 3);
    }
  }
  connected_devices_.clear();
  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] # of connected devices " << connected_devices_.size());
  communication_handler_->closeSerialPort(serial_port);
  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] does not handle [" << serial_port << "] anymore.");
  return 0;
}

int ROSEE::QbhandHal::activate(const int &id, const bool &command, const int &max_repeats) {
  std::string command_prefix = command ? "" : "de";
  bool status = false;
  int failures = 0;

  failures = isActive(id, max_repeats, status);
  if (status != command) {
    devices_.at(id)->setMotorStates(command);
    ros::Duration(0.5).sleep(); // wait for motors to be active
    failures = std::max(failures, isActive(id, max_repeats, status));
    if (status != command) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"communication_handler", "[CommunicationHandler] device [" << id << "] fails on " << command_prefix << "activation.");
      return -1;
    }
    ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] device [" << id << "] motors have been " << command_prefix << "activated!");
    return failures;
  }
  ROS_DEBUG_STREAM_NAMED("communication_handler", "[CommunicationHandler] device [" << id << "] motors were already " << command_prefix << "activated!");
  return failures;
}

int ROSEE::QbhandHal::activate(const int &id, const int &max_repeats) {
  return activate(id, true, max_repeats);
}

int ROSEE::QbhandHal::deactivate(const int &id, const int &max_repeats) {
  return activate(id, false, max_repeats);
}

int ROSEE::QbhandHal::isActive(const int &id, const int &max_repeats, bool &status) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  status = false;
  while (failures <= max_repeats) {
    if (devices_.at(id)->getMotorStates(status) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int ROSEE::QbhandHal::setControlMode(const int &id, const int &max_repeats, uint8_t &control_id) {
  int failures = 0;
  while (failures <= max_repeats) {
    if (devices_.at(id)->setParamControlMode(control_id) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int ROSEE::QbhandHal::getPositions(const int &id, const int &max_repeats, std::vector<short int> &positions) {
  int failures = 0;
  positions.resize(3);  // required by 'getPositions()'
  while (failures <= max_repeats) {
    if (devices_.at(id)->getPositions(positions) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int ROSEE::QbhandHal::setCommandsAndWait(const int &id, const int &max_repeats, std::vector<short int> &commands) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  const clock_t begin_time = clock();
  commands.resize(2);  // required by 'setCommandsAndWait()'
  while (failures <= max_repeats) {
    if (devices_.at(id)->setControlReferencesAndWait(commands) < 0) {
      failures++;
      continue;
    }
    break;
  }
  //std::cout << "time setCommandsAndWait of" << id << ": " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
  return failures;
}

int ROSEE::QbhandHal::setCommandsAsync(const int &id, std::vector<short int> &commands) {
  // qbhand sets only inputs.at(0), but setCommandsAsync expects two-element vector (ok for both qbhand and qbmove)
  commands.resize(2);  // required by 'setCommandsAsync()'
  const clock_t begin_time = clock();
  devices_.at(id)->setControlReferences(commands);
  ros::Duration(0.0001).sleep();
  //std::cout << "time setCommandsAsync of" << id << ": " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
  return 0;  // note that this is a non reliable method
}



