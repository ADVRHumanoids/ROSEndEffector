#ifndef ROSEE_PACKET_H
#define ROSEE_PACKET_H

#include <map>
#include <string>

namespace ROSEE { 
    
struct ROSEEPacket
{
    struct Rx
    {
        
        unsigned int* motor_id;
        double* motor_position_actual;
             
    };

    struct Tx
    {
        //std::map <std::string, double> motors_position_refs;
        unsigned int* motor_id;
        double* motor_position_reference;

    };

};


}


#endif // ROSEE_PACKET_H

