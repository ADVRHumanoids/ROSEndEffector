# ROS End-Effector 
ROS End-Effector package: provides a ROS-based set of standard interfaces to command robotics end-effectors in an agnostic fashion.

Wiki is here: https://github.com/ADVRHumanoids/ROSEndEffector/wiki

Documentation about the API of ROS End-Effector can be found here https://advrhumanoids.github.io/ROSEndEffector/index.html

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




Welcome to the ROS End-Effector Action finder 

# How to run for the Schunk Hand
```bash
#usual source
roslaunch ros_end_effector findActionsSchunk.launch
```
Note that urdf and srdf files for schunk are in the urdf and srdf folders of this branch.    
2 yaml files will be created/ovewritten in /configs/actions/<hand_name>
### To play with the hand and manually set the poses
##### Install schunk things: 
(following [here](http://wiki.ros.org/schunk_svh_driver), section "2.4 From Source")
```bash
mkdir ~/schunk_ws
mkdir ~/schunk_ws/src
cd ~/schunk_ws/src
git clone https://github.com/fzi-forschungszentrum-informatik/fzi_icl_core.git #schunk library, not sure if needed for only simulation
git clone https://github.com/fzi-forschungszentrum-informatik/fzi_icl_comm.git #schunk library, not sure if needed for only simulation
git clone https://github.com/fzi-forschungszentrum-informatik/schunk_svh_driver.git #the main schunk repo
cd ..
catkin_make_isolated
source devel_isolated/setup.bash
```
#### Finally launch the hand simulation
```bash
roslaunch schunk_svh_driver svh_controller.launch standalone:=true gui:=true simulation:=true
```
Be sure to put in rviz as fixed frame __base_link__

# How to run for the TestEE Example
```bash
#usual source
roslaunch ros_end_effector findActionsTestEE.launch
```
#### To play with the hand and manually set the poses
```bash
roslaunch ros_end_effector two_finger_ee_startup.launch inSlider:=true
```

# How to run for the 2 Finger example
```bash
#usual source
roslaunch ros_end_effector findActionsTwoFinger.launch
```
#### To play with the hand and manually set the poses
```bash
roslaunch ros_end_effector test_ee_startup.launch inSlider:=true
```

