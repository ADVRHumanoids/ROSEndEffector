/*
   Copyright (C) Italian Institute of Technology

   Developer:
       Luca Muratore (2020-, luca.muratore@iit.it)


*/

#include <heri_2_ee_hal.h>
#include <string>

ROSEE::Heri2EEHal::Heri2EEHal(const char* config_yaml,
                              ROSEE::EEInterface::Ptr ee_interface) :
    Ec_Boards_ctrl(config_yaml),
    ROSEE::EEHal(ee_interface)
{

    XBot::Logger::info(XBot::Logger::Severity::HIGH) << "Low Level Config file in use is: " << config_yaml << XBot::Logger::endl();
    
    YAML::Node node = YAML::LoadFile(config_yaml);

    for (auto joint_it : node["Joint_Actuation_Info"]) {
        
        std::string jointName = joint_it.first.as<std::string>();
        auto values = joint_it.second;
        JointActuationInfo jai;
        jai.board_id = values["board_id"];
        jai.finger_in_board_id = values["finger_in_board_id"];
        jai.motor_lower_limit = values ["motor_lower_limit"];
        jai.motor_upper_limit = values ["motor_upper_limit"];
        jai.joint_to_moto_slope = 
          (jai.motor_upper_limit - jai.motor_lower_limit) / 
            (ee_interface.getUpperPositionLimit(jointName) - ee_interface.getLowerPositionLimit(jointName));
        
        jai.moto_to_joint_slope = 1/jai.joint_to_moto_slope;
        
        
        _joint_actuation_info[joint_it.first] = jai;
        
    }
    
    
    for (auto joint_it : node["Pressure_Sensor_Info"]) {
    
       sensorName_to_motorId[it.first.as<std::string>()] =  
            std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(
                it.second["board_id"],
                it.second["finger_in_board_id"],
                it.second["sensor_in_finger_id"]); 
    }
    
    
    //TODO OBSOLETE DELETE THESEE do not fill this hardcoded
    jointName_to_motorId["LFB1__LFP1_1"] = 
        std::make_pair<unsigned short int, unsigned short int>(112, 1);
    jointName_to_motorId["LFB2__LFP2_1"] = 
        std::make_pair<unsigned short int, unsigned short int>(112, 2);
    jointName_to_motorId["LFB3__LFP3_1"] = 
        std::make_pair<unsigned short int, unsigned short int>(113, 1);
    jointName_to_motorId["SFB1__SFP1_1"] = 
        std::make_pair<unsigned short int, unsigned short int>(113, 2);
        
    sensorName_to_motorId["Finger1_Sensor1"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(112, 1, 1);
    sensorName_to_motorId["Finger1_Sensor2"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(112, 1, 2);
    sensorName_to_motorId["Finger1_Sensor3"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(112, 1, 3);
    
    sensorName_to_motorId["Finger2_Sensor1"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(112, 2, 1);
    sensorName_to_motorId["Finger2_Sensor2"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(112, 2, 2);
    sensorName_to_motorId["Finger2_Sensor3"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(112, 2, 3);
    
    sensorName_to_motorId["Finger3_Sensor1"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(113, 1, 1);
    sensorName_to_motorId["Finger3_Sensor2"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(113, 1, 2);
    sensorName_to_motorId["Finger3_Sensor3"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(113, 1, 3);
    
    sensorName_to_motorId["Thumb_Sensor1"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(113, 2, 1);
    sensorName_to_motorId["Thumb_Sensor2"] =  std::make_tuple<unsigned short int, unsigned short int, unsigned short int>(113, 2, 2);
}

ROSEE::Heri2EEHal::~Heri2EEHal()
{

    std::cout << "~" << typeid(this).name() << std::endl;
    iit::ecat::print_stat(s_loop);

}

void ROSEE::Heri2EEHal::base_init()
{
    const YAML::Node config = get_config_YAML_Node();

    // init Ec_Boards_ctrl
    if (Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK) {
        XBot::Logger::error() << "Ec_Boards_ctrl::init() failed!" << XBot::Logger::endl();
        throw "ECAT protocol issue";
    }

    // I know the wkc here
    _wkc = slaves.size();

    // >>> actual ECAT state is PREOP ...
    get_esc_map_byclass(fingers);

    int slave_pos, slave_id;
    float motor_pos, motor_pos_2, aux;
    iit::ecat::advr::HeriHandESC* moto;

    for (auto const & item : fingers) {
        slave_pos = item.first;
        slave_id = pos2Rid(slave_pos);
        moto = item.second;
        moto->readSDO_byname("motor_pos", motor_pos);
        moto->readSDO_byname("motor_pos_2", motor_pos_2);
        
        moto->readSDO_byname("Min_pos", aux);
        min_pos[slave_id] = aux;
        moto->readSDO_byname("Max_pos", aux);
        max_pos[slave_id] = aux;
        moto->readSDO_byname("Min_pos_2", aux);
        min_pos_2[slave_id] = aux;
        moto->readSDO_byname("Max_pos_2", aux);
        max_pos_2[slave_id] = aux;


        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        if (!(moto->start(iit::ecat::advr::CTRL_SET_POS_MOTOR_MODE) == iit::ecat::advr::EC_BOARD_OK)) {
            XBot::Logger::error(">> Joint_id %d Error while starting the HERI hand motor. \n",
                                pos2Rid(slave_pos));
        }
    }
}

int ROSEE::Heri2EEHal::base_start()
{

    int wkc =  Ec_Boards_ctrl::set_operative();
    if (wkc <= 0) {
        XBot::Logger::error() << "Ec_Boards_ctrl::set_operative() failed!" << XBot::Logger::endl();
        std::runtime_error("something wrong in Ec_Boards_ctrl::set_operative()");;
    }

    return wkc;
}

int ROSEE::Heri2EEHal::base_stop()
{

    set_pre_op();

    return 0;
}

void ROSEE::Heri2EEHal::post_init()
{

    int operative_wkc = base_start();

    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;

}

bool ROSEE::Heri2EEHal::init()
{

    base_init(); // can't fail, in case of error it throws
    
    sleep(1);   // NOTE ignoranza
    
    base_start();
    return true;
}

bool ROSEE::Heri2EEHal::sense()
{

    tNow = iit::ecat::get_time_ns();
    s_loop(tNow - tPre);
    tPre = tNow;

    try {

        int ret = recv_from_slaves(timing);

        if (ret != iit::ecat::advr::EC_BOARD_OK) {
            // TODO implement a strategy
            XBot::Logger::error() << "recv_from_slaves FAIL !" << XBot::Logger::endl();
        }
    } catch (iit::ecat::EscWrpError& e) {
        XBot::Logger::error() << e.what() << XBot::Logger::endl();
        return false;
    }
    return true;
}

bool ROSEE::Heri2EEHal::move()
{

    try {
        send_to_slaves();

    } catch (iit::ecat::EscWrpError& e) {
        XBot::Logger::error() << e.what() << XBot::Logger::endl();
        return false;
    }

    return true;
}

bool ROSEE::Heri2EEHal::getMotorPosition(std::string joint_name, double& motor_position)
{
    iit::ecat::advr::HeriHandEscPdoTypes::pdo_rx hand_pdo_rx;
    iit::ecat::advr::HeriHandESC * finger;
        
    // HACK do proper mapping. Is it ok now?
    int finger_id = -1;
    int motor_in_finger_id = -1;
    
    auto it = _joint_actuation_info.find(joint_name);
    if (it == jointName_to_motorId.end()) {
        XBot::Logger::error ( ">> Joint_name %s does not exists \n",
                                joint_name);
        return false;
    }
    
    finger_id = it->second.board_id;
    motor_in_finger_id = it->second.finger_in_board_id;
    
    /////////////  TODO deprecaaated below
    auto it = jointName_to_motorId.find(joint_name);
    
    if (it == jointName_to_motorId.end()) {
        XBot::Logger::error ( ">> Joint_name %s does not exists \n",
                                joint_name);
        return false;
    }
    
    finger_id = it->second.first;
    motor_in_finger_id = it->second.second;

    /////////////////////////////////////
    
    finger = fingers.at(rid2Pos(finger_id));
    hand_pdo_rx = finger->getRxPDO();
    
    if( motor_in_finger_id == 1 ) {
        motor_position = hand_pdo_rx.motor_pos;
    }
    else if ( motor_in_finger_id == 2 ) {
        motor_position = hand_pdo_rx.motor_pos_2;
    }
    else {
        XBot::Logger::error ( ">> Joint_id %d Error while calling get_finger_motor_position() on HERI hand ESC: the motor_in_finger_id must be 1 or 2. \n",
                                motor_in_finger_id);
    }
    
    //ROS_WARN_STREAM("CURRENT 1 " << hand_pdo_rx.m1_curr);
    //ROS_WARN_STREAM("CURRENT 2 " << hand_pdo_rx.m2_curr);
    //std::cout << std::endl;

    
    return true;
}

bool ROSEE::Heri2EEHal::getJointPosition(std::string joint_name, double& joint_position)
{
    double moto_pos = 0.0;
    this->getMotorPosition ( joint_name, moto_pos);
    actuatorToJointPosition ( joint_name, moto_pos, joint_position);
    
    return true;
}

bool ROSEE::Heri2EEHal::getMotorEffort(std::string joint_name, double& motor_effort)
{
    return false;
}

bool ROSEE::Heri2EEHal::getMotorVelocity(std::string joint_name, double& motor_velocity)
{
    return false;
}
/*
bool ROSEE::Heri2EEHal::getMotorCurrent(std::string joint_name, double& motor_current)
{
    
    iit::ecat::advr::HeriHandEscPdoTypes::pdo_rx hand_pdo_rx;
    iit::ecat::advr::HeriHandESC * finger;
        
    // HACK do proper mapping. Is it ok now?
    int finger_id = -1;
    int motor_in_finger_id = -1;
    
    auto it = jointName_to_motorId.find(joint_name);
    
    if (it == jointName_to_motorId.end()) {
        XBot::Logger::error ( ">> Joint_name %s does not exists \n",
                                joint_name);
        return false;
    }
    
    finger_id = it->second.first;
    motor_in_finger_id = it->second.second;

    /////////////////////////////////////
    
    finger = fingers.at(rid2Pos(finger_id));
    hand_pdo_rx = finger->getRxPDO();
    
    if( motor_in_finger_id == 1 ) {
        motor_current = hand_pdo_rx.m1_curr;
    }
    else if ( motor_in_finger_id == 2 ) {
        motor_current = hand_pdo_rx.m2_curr;
    }
    else {
        XBot::Logger::error ( ">> Joint_id %d Error while calling getMotorCurrent() on HERI hand ESC: the motor_in_finger_id must be 1 or 2. \n",
                                motor_in_finger_id);
        return false;
    }
    
    return true;
}
**/

bool ROSEE::Heri2EEHal::getPressure(std::string sensor_name, double& sensor_value) {
    
    iit::ecat::advr::HeriHandEscPdoTypes::pdo_rx hand_pdo_rx;
    iit::ecat::advr::HeriHandESC * finger;
    
    int finger_id = -1;
    int motor_in_finger_id = -1;
    int sensor_in_motor_id = -1;
    
    auto it = sensorName_to_motorId.find(sensor_name);
    
    if (it == sensorName_to_motorId.end()) {
        XBot::Logger::error ( ">> sensor Name %s does not exists \n",
                                sensor_name);
        return false;
    }
    
    auto value = it->second;
    
    finger_id = std::get<0>(value);
    motor_in_finger_id = std::get<1>(value);
    sensor_in_motor_id = std::get<2>(value);
    
    finger = fingers.at(rid2Pos(finger_id));
    hand_pdo_rx = finger->getRxPDO();
    
    if( motor_in_finger_id == 1 ) {
        if (sensor_in_motor_id == 1 ) {
            
            sensor_value = hand_pdo_rx.m1_an_1;
            
        } else if (sensor_in_motor_id == 2 ) {
            
            sensor_value = hand_pdo_rx.m1_an_2;
            
        } else if (sensor_in_motor_id == 3 ) {
            
            sensor_value = hand_pdo_rx.m1_an_3;
            
        } else {
            XBot::Logger::error ( ">> sensor_in_motor_id %d Error while calling getPressure on HERI hand ESC: the sensor_in_motor_id must be 1 or 2 or 3. \n",
                                sensor_in_motor_id);
        }

    }
    else if ( motor_in_finger_id == 2 ) {
        if (sensor_in_motor_id == 1 ) {
            
            sensor_value = hand_pdo_rx.m2_an_1;
            
        } else if (sensor_in_motor_id == 2 ) {
            
            sensor_value = hand_pdo_rx.m2_an_2;
            
        } else if (sensor_in_motor_id == 3 ) {
            
            sensor_value = hand_pdo_rx.m2_an_3;
            
        } else {
            XBot::Logger::error ( ">> sensor_in_motor_id %d Error while calling getPressure on HERI hand ESC: the sensor_in_motor_id must be 1 or 2 or 3. \n",
                                sensor_in_motor_id);
        }
    }
    else {
        XBot::Logger::error ( ">> Joint_id %d Error while calling getPressure on HERI hand ESC: the motor_in_finger_id must be 1 or 2. \n",
                                motor_in_finger_id);\
                               
    }
        
    return true;
    
    
}

bool ROSEE::Heri2EEHal::setPositionReference(std::string joint_name, 
                                             double joint_position_reference)
{

    // HACK do proper mapping. Is it ok like this?
    int finger_id = -1;
    int motor_in_finger_id = -1;
    
    
        auto it = _joint_actuation_info.find(joint_name);
    if (it == jointName_to_motorId.end()) {
        XBot::Logger::error ( ">> Joint_name %s does not exists \n",
                                joint_name);
        return false;
    }
    
    finger_id = it->second.board_id;
    motor_in_finger_id = it->second.finger_in_board_id;
    
    ////////////////////////TODO deprecaaated
    auto it = jointName_to_motorId.find(joint_name);
    
    if (it == jointName_to_motorId.end()) {
        XBot::Logger::error ( ">> Joint_name %s does not exists \n",
                                joint_name);
        return false;
    }
    
    finger_id = it->second.first;
    motor_in_finger_id = it->second.second;
    //////////////////////////
    
    double moto_position_reference = 0.0;
    jointToActuatorPosition(joint_name, joint_position_reference, moto_position_reference);
    
    //TODO DELETE DEBUG PRINT
    //std::cout << "finger_id    " << finger_id << std::endl;
    //std::cout << "motor_in_finger_id    " << motor_in_finger_id << std::endl;
    //std::cout << "JOINTTTTTTTTTTT POSSSSSSSSSSSSSSSSSS    " << joint_position_reference << std::endl;
    //std::cout << "MOTOOOOOOOOOOOOOOOOOOOOO POSSSSSSSSSSSSSSSSSS     " << moto_position_reference << std::endl << std::endl;
    

    
    move_finger(finger_id, motor_in_finger_id, moto_position_reference);
    
    /////////////////////////////////////////////////////

    return true;

}

// NOTE motor_in_finger_id is 1 or 2
bool ROSEE::Heri2EEHal::move_finger(int finger_id,
                                    int motor_in_finger_id,
                                    double pos_ref)
{
    iit::ecat::advr::HeriHandESC* finger;
    finger = fingers.at(rid2Pos(finger_id));

    if (motor_in_finger_id == 1) {
        // TBD check pos_ref in range  max_pos.at(finger_id) and min_pos.at(finger_id)
        // TBD Lesser than max pos stored in the low code or use the value of yaml config??s
//         XBot::Logger::info("HERI: Pos ref: [%d - %d] %f\n", finger_id, motor_in_finger_id, pos_ref);
        finger->set_posRef(pos_ref);
    } else if (motor_in_finger_id == 2) {
        // TBD check pos_ref in range  max_pos_2.at(finger_id) and min_pos_2.at(finger_id)
//         XBot::Logger::info("HERI: Pos ref: [%d - %d] %f\n", finger_id, motor_in_finger_id, pos_ref);
        finger->set_posRef_2(pos_ref);
    } else {
        XBot::Logger::error(">> Joint_id %d Error while calling move_finger() on HERI hand ESC: the motor_in_finger_id must be 1 or 2. \n",
                            finger_id);
    }

    return true;
}


//TODO check this... Should be the implementation of an abstract method of ROSEE::EEHal?
bool ROSEE::Heri2EEHal::jointToActuatorPosition(std::string joint_name, 
                                               double joint_pos, 
                                               double& actuator_pos) {
    
    //NOTE for now, this relation is linear, even if in truth it is not like that.
    
    auto it = _joint_actuation_info.find(joint_name);
    if (it == jointName_to_motorId.end()) {
        XBot::Logger::error ( ">> Joint_name %s does not exists \n",
                                joint_name);
        return false;
    }
    
    actuator_pos = it.second.motor_lower_limit + it.second.joint_to_moto_slope * 
        (joint_pos - _ee_inteface->getLowerPositionLimit(joint_name));
        
        
        
        
    
    // TODO DEPRECAAATED BELOW
    //NOTE tori safe limits are considered for motors
    if(joint_name == "LFB1__LFP1_1" ) {
        //moto [0;1.95]  urdf joint [0; 0.9]
        actuator_pos = (1.95/0.9) * joint_pos;
    }
    else if(joint_name == "LFB2__LFP2_1" ) {
        //moto [0;1.45]  urdf joint [0; 0.73]
        actuator_pos = (1.45/0.73) * joint_pos;
    }
    else if(joint_name == "LFB3__LFP3_1" ) {
        //moto [0;1.35]  urdf joint [0; 0.9]
        actuator_pos = (1.35/0.9) * joint_pos;
    }
    //NOTE for the thumb an offset here is necessary: the cable initially only move the tip,
    //the first phalange starts moving visibily only when the tip is at is limit
    else if(joint_name == "SFB1__SFP1_1" ) {
        //moto [0;1.7]  urdf joint [0; 0.2]
        
        actuator_pos = (1.65/0.2) * (joint_pos);

        // with OFFSET...
        //if (joint_pos <= 0.001 ) {
        //    actuator_pos = 0;
            
        //} else {
        //    actuator_pos = (1.65/0.2) * (joint_pos + 0.15);
            //put to safe limit if it exceeds
        //    if (actuator_pos > 1.65) {
        //        actuator_pos = 1.65;
        //    } 
        //}
        
       // ROS_WARN_STREAM ("SENDING ACT COMMAND:: Act pos:" << actuator_pos << "   joint post " << joint_pos);
           
    }
    
    return true;
    
}

bool ROSEE::Heri2EEHal::actuatorToJointPosition(std::string joint_name, 
                                               double actuator_pos, 
                                               double& joint_pos) {
    
    auto it = _joint_actuation_info.find(joint_name);
    if (it == jointName_to_motorId.end()) {
        XBot::Logger::error ( ">> Joint_name %s does not exists \n",
                                joint_name);
        return false;
    }s
    
    joint_pos = _ee_inteface->getLowerPositionLimit(joint_name) + it.second.moto_to_joint_slope *
        (actuator_pos - it.second.motor_lower_limit);
        
        
        
// TODO deprecaaaaaaaaaaated below
    
    double jointLimit = _ee_inteface->getUpperPositionLimit(joint_name);
     
    //NOTE tori safe limits are considered for motors
    if(joint_name == "LFB1__LFP1_1" ) {
        //moto [0;1.95]  urdf joint [0; 0.9]
        joint_pos = (jointLimit/1.95) * actuator_pos;
    }
    else if(joint_name == "LFB2__LFP2_1" ) {
        //moto [0;1.45]  urdf joint [0; 0.73]
        joint_pos = (jointLimit/1.45) * actuator_pos;
    }
    else if(joint_name == "LFB3__LFP3_1" ) {
        //moto [0;1.35]  urdf joint [0; 0.9]
        joint_pos = (jointLimit/1.35) * actuator_pos;
    }
    else if(joint_name == "SFB1__SFP1_1" ) {
        //moto [0;1.7]  urdf joint [0; 0.2]
        
        joint_pos = (jointLimit/1.65) * actuator_pos;

        // OFFSET...
       // joint_pos = ((0.2/1.65) * actuator_pos)- 0.15;
        //HACK with offset, joint pos will never be at 0.2, but we say so to fill the feedback of the action...
        //if ( actuator_pos >= 1.63 ) {
        //    joint_pos = 0.2;
        //}
        
       // ROS_WARN_STREAM ("RECEIVING ACT FEEDBACK:: Act pos:" << actuator_pos << "   joint post " << joint_pos);

        
     
    }
    
    return true;

    
}

