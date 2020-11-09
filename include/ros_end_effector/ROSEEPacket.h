#ifndef ROSEE_PACKET_H
#define ROSEE_PACKET_H

#include <map>
#include <string>

namespace ROSEE { 
    
struct ROSEEPacket
{
    struct Rx
    {
        
        //std::string motor_name;
        double motor_position;
             
    };

    struct Tx
    {
        //std::map <std::string, double> motors_position_refs;
       // std::string motor_name;
        double motor_position_ref;

    };

};


}


#endif // ROSEE_PACKET_H

