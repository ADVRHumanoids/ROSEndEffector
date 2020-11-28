#include <ros_end_effector/HandInfoServices.h>

bool ROSEE::HandInfoServices::init_ros_service() {
    
    std::string service_name = "hand_info";
    
    _hand_info_server = _nh->advertiseService(service_name, 
        &HandInfoServices::handInfoCallback, this);
  
}

bool ROSEE::HandInfoServices::handInfoCallback (    
    rosee_msg::HandInfo::Request& request,
    rosee_msg::HandInfo::Response& response) {
   
    //empty request for now
    
    response = this->response;
}
