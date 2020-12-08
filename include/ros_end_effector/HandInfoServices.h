#ifndef __ROSEE_HAND_INFO_SERVICES__
#define __ROSEE_HAND_INFO_SERVICES__

#include <ros/ros.h>
#include <rosee_msg/HandInfo.h>

namespace ROSEE {
    
class HandInfoServices {
    
public:
    
    HandInfoServices(ros::NodeHandle *nh) : _nh (nh) {}
    bool init_ros_service();
    
    rosee_msg::HandInfo::Response response;
    

private: 
    ros::ServiceServer _hand_info_server;
    ros::NodeHandle* _nh;

    bool handInfoCallback(
        rosee_msg::HandInfo::Request& request,
        rosee_msg::HandInfo::Response& response);

};

} //namespace


#endif //__ROSEE_FIND_ACTIONS_
