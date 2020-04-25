.. _install:

.. 
  for line break without identation ( | symbol put a identation)
.. role:: raw-html(raw)
    :format: html

Installation
============

ROS-EndEffector is a ROS package tested on ROS kinetic with Ubuntu 16.04

**Note** We are going to use the abreviation ROSEE for the ROS End-Effector from now on


Install External System Dependencies
######################################

.. code-block:: console 

  sudo apt-get install ros-kinetic-moveit #moveit

Install ROS End-Effector package from sources
##############################################

**Note** A .rosinstall file will be available in future for a single-command way to install all

**IMPORTANT** : ROSEE is a project under development. When cloning, be sure to download the branch you really want. Currently the newest features can be found in the *devel* branch (so, in the commands below substitute <branch_you_want> with devel. Note that also the other ROSEEE related packages should be used with the right branch.

The next steps will guide you to the creation of a new catkin_workspace, for clone all the necessary repository and compile them all

- Create a catkin workspace

  .. code-block:: console
  
    mkdir ~/ROSEE
    cd ROSEE
    mkdir src
    cd src
    catkin_init_workspace


- Clone Necessary dependencies:

  .. code-block:: console
   
    git clone -b <branch_you_want> https://github.com/ADVRHumanoids/rosee_msg.git
         
- Clone Optional dependencies:

  - GUI
  
    .. code-block:: console 
    
      git clone -b <branch_you_want> https://github.com/ADVRHumanoids/rosee_gui.git
  
  - Gazebo Plugin (more info in the dedicated :ref:`section <roseeGazeboPlugin>`)
  
    .. code-block:: console
    
      git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git #necessary external plugin
      git clone -b <branch_you_want> https://github.com/ADVRHumanoids/rosee_gazebo_plugins.git
          
  **TODO** you can see details on each optional plugin in their relative page of this doc
  :raw-html:`<br />` 
  :raw-html:`<br />` 
  
- Clone Core Package

  .. code-block:: console
   
    git clone -b <branch_you_want> https://github.com/ADVRHumanoids/ROSEndEffector
  
- Compile them all!

  .. code-block:: console
  
    cd ~/ROSEE
    catkin_make    


Installation issues
#####################  

Not found (for now...)

    
 

