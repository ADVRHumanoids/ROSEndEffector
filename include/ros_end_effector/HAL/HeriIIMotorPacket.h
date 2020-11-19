#ifndef HERI_II_MOTOR_PACKET_H
#define HERI_II_MOTOR_PACKET_H

namespace XBot { namespace Hal {
    
struct HeriIIMotorPacket
{
    struct Rx
    {
        unsigned int motor_id; //TODO necessary? 
        double motor_position_actual;
        double motor_current_actual;
             
    };

    struct Tx
    {
        unsigned int motor_id; //TODO necessary? 
        double motor_position_reference;
        double motor_current_reference;

    };

};


}}


#endif // HERI_II_MOTOR_PACKET_H

