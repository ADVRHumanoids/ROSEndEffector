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

namespace ROSEE {
    
    
    /**
     * @brief Class representing and End-Effector
     * 
     */
    class EEInterface {

    public:
        
        typedef std::shared_ptr<EEInterface> Ptr;
        typedef std::shared_ptr<const EEInterface> ConstPtr;
        
        EEInterface( const std::map<std::string, std::vector<std::string>>& ee_description );
        //EEInterface ( const EEInterface& other );
        //EEInterface& operator= ( const EEInterface& p );
        virtual ~EEInterface();
        
        /**
         * @brief getter for the fingers name in the current End-Effector
         * 
         * @return bool existing finger names in the End-Effector
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
         * @return void
         */
        void getActuatedJoints( std::vector<std::string>& actuated_joints );
        
        /**
         * @brief getter for the actuated joints of a finger
         * 
         * @param finger_name the name of the finger where to retrieve actuated joints
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
        
    private:
        
        std::map<std::string, std::vector<std::string>> _ee_description;
        
        std::vector<std::string> _fingers_names, _actuated_joints;
        
        int _joints_num = 0;
        
        
        
    };
    
}

#endif // __ROSEE_EE_INTERFACE__
