# ROS End-Effector 
It provides a ROS-based set of standard interfaces to command robotics end-effectors in an agnostic fashion.

The ROS End-Effector main documentation is here: https://advrhumanoids.github.io/ROSEndEffectorDocs/

Doxygen-generated documentation about the API of ROS End-Effector can be found here: https://advrhumanoids.github.io/ROSEndEffector/index.html

CI powered by Travis CI (.org) [![Build Status](https://travis-ci.org/ADVRHumanoids/ROSEndEffector.svg?branch=master)](https://travis-ci.org/ADVRHumanoids/ROSEndEffector)

https://travis-ci.org/ADVRHumanoids/ROSEndEffector

***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 



## NOTES about XBOT2 usage

- For a new hand, each non fixed joint (even the virtual, passive and mimic) must belong to a chain (convention that  I used is to add a
  virtualXXX named chain, if it is necessary
- Also, all the chains must belong to the supergroup "chains" (name is fixed)
- Check robotiq2f srdf for correctess

- In urdf, add the xbot gazebo plugin. Each joint must have a PID, even the mimic and passive. simply put 0 gains for them. And keep the mimic joint plugin ( maybe in future will be integrated in xbot), it is this one that move the mimic (since for xbot they have 0 gains).
- Again check the robotiq2f urdf as example

- rosee.world has the clock plugin added. This is already done, the world is used by each hand

- Running:
  - 1 terminal roscore (maybe not necessary)
  - 2 terminal: source xbot, source rosee, set_xbot2_config (necessary?), and launch rosee_gazebo_plugin (or rosee_startup) file with hand name. RUN gazebo
  - 3 terminal: set_xbot2_config, export xbot_root, and finally xbot2-core --verbose
  - 4 terminal: rosservice call /xbotcore/ros_ctrl/switch 1  otherwise command through ros are not read, then you can call the gui to move the hand
  
  - other stuff:
    - To publish via topic, the field ctrl_mode must be set (to 1 for position?)
    - If command is too brute, xbot stops the joint device by default, to restore: rosservice call /xbotcore/joint_master/set_control_mask 1
    
*TODO*
- Check all other hands
- Test the whole ROSEE + XBOT2 + Gazebo, automatize launch files if necessary
- A method to choose between xbot joint server and rosee plugin in the urdf


