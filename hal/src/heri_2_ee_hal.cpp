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
        
    // HACK do proper mapping
    int finger_id = -1;
    int motor_in_finger_id = -1;

    if(joint_name == "LFB1__LFP1_1" ) {
        finger_id = 112;
        motor_in_finger_id = 1;
    }
    else if(joint_name == "LFB2__LFP2_1" ) {
        finger_id = 112;
        motor_in_finger_id = 2;
    }
    else if(joint_name == "LFB3__LFP3_1" ) {
        finger_id = 113;
        motor_in_finger_id = 1;
    }
    else if(joint_name == "SFB1__SFP1_1" ) {
        finger_id = 113;
        motor_in_finger_id = 2;
    }
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
        XBot::Logger::error ( ">> Joint_id %d Error while calling get_finger_motor_position() on HERI hand ESC: the finger_id must be 1 or 2. \n",
                                motor_in_finger_id);
    }
    
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

bool ROSEE::Heri2EEHal::setPositionReference(std::string joint_name, 
                                             double position_reference)
{

    // HACK do proper mapping
    int finger_id = -1;
    
    if(joint_name == "LFB1__LFP1_1" ) {
        finger_id = 112;
        move_finger(finger_id, 1, position_reference);
    }
    else if(joint_name == "LFB2__LFP2_1" ) {
        finger_id = 112;
        move_finger(finger_id, 2, position_reference);
    }
    else if(joint_name == "LFB3__LFP3_1" ) {
        finger_id = 113;
        move_finger(finger_id, 1, position_reference);
    }
    else if(joint_name == "SFB1__SFP1_1" ) {
        finger_id = 113;
        move_finger(finger_id, 2, position_reference);
    }
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


