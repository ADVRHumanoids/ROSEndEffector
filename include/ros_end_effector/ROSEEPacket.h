#ifndef ROSEE_PACKET_H
#define ROSEE_PACKET_H

namespace ROSEE { 

struct ROSEEPacket
{
    struct Rx
    {
        
        std::map <std::string, double> motors_position_refs;
    };

    struct Tx
    {
        //std::map <std::string, double> motors_position_refs;

        std::string motor_name;
        double motor_position_ref;
    };

};


}


#endif // ROSEE_PACKET_H

