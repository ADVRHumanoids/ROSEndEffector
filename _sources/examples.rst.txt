.. _examples:

Ready to use examples with tested hands
=========================================

You can run experiments with ready-to-use hands. The simplest one are geometrical grippers created only for showing purposes:

- test_ee
- two_finger
- two_finger_mimic

Simply run *UniversalFindActions* node and *UniversalROSEndEffector* as described in :ref:`How to use ROS End-Effector with your End-Effector <usage>` section, reported again briefly below :

.. code-block:: console

  #substitute test_ee accordigly
  
  roslaunch ros_end_effector findActions.launch hand_name:=test_ee #offline phase
  roslaunch ros_end_effector rosee_startup.launch hand_name:=test_ee #online phase
  

There are also some examples done with models from real hands, at the moment there are:

- `HERI II`_
- `Schunk SVH 5-finger hand`_
- `qb SoftHand`_
- `Robotiq 3-Finger Gripper`_
- `Robotiq 2F-140 Gripper`_

See below for istruction on how to install packages for these hand (because their meshes are needed)


.. _`HERI II`: 
 
HERI II
**************

The files for this hand are already in the ROS End-Effector main package. So simply run:

.. code-block:: console

  roslaunch ros_end_effector findActions.launch hand_name:=heri_II #offline phase
  roslaunch ros_end_effector rosee_startup.launch hand_name:=heri_II #online phase

Remember to add :code:`gazebo:=true` to the second command to launch the dynamic simulation with gazebo

  **NOTE** Experiment with the real hardware are incoming soon!


.. _`Schunk SVH 5-finger hand`:

Schunk SVH 5-finger hand
***************************

This a complex humanoids hand with lot of actuted joints and DOFs. More details about it can be found 
in `Schunk website <https://schunk.com/it_en/gripping-systems/highlights/svh/>`_.

Necessary steps before run ROS End-Effector with this hand:

.. code-block:: console

  mkdir ~/schunk_ws
  mkdir ~/schunk_ws/src
  cd ~/schunk_ws/src
  git clone https://github.com/fzi-forschungszentrum-informatik/fzi_icl_core.git #schunk library, I am not sure if needed for only the simulation
  git clone https://github.com/fzi-forschungszentrum-informatik/fzi_icl_comm.git #schunk library, I am not sure if needed for only the simulation
  git clone https://github.com/fzi-forschungszentrum-informatik/schunk_svh_driver.git #the main schunk repo
  cd ..
  catkin_make_isolated
  source devel_isolated/setup.bash   

Then run codes with :code:`hand_name:=schunk` as argument

  **Note** the schunk URDF does not have dynamic params, so at the moment it can not be simulated with gazebo
  (so do not use :code:`gazebo:=true`)
  
  
  
.. _`qb SoftHand`: 
 
qb SoftHand
*****************

This hand has a humanoid structure but it has only a single actuated joint that close all the fingers togheter in a compliant way. More info at `qb robotics website <https://qbrobotics.com/products/qb-softhand-research/>`_.

Necessary steps before run ROS End-Effector with this hand:

.. code-block:: console

  cd ~/qbhand/src
  git clone --branch production-kinetic https://bitbucket.org/qbrobotics/qbdevice-ros.git
  git clone --branch production-kinetic https://bitbucket.org/qbrobotics/qbhand-ros.git

  cd ~/catkin_ws
  catkin_make
  
Then run codes with :code:`hand_name:=qbhand` as argument
 
  **Note** This hand will be tested for real soon (not simulation only)



.. _`Robotiq 3-Finger Gripper`:
  
Robotiq 3-Finger Gripper
**************************

This is a gripper with 3-Fingers. More information at at `Robotiq website <https://robotiq.com/products/3-finger-adaptive-robot-gripper/>`_.

Necessary steps before run ROS End-Effector with this hand:

.. code-block:: console

  mkdir ~/robotiq_ws
  cd robotiq_ws
  mkdir src
  cd src
  git clone https://github.com/ros-industrial/robotiq.git
  cd robotiq
  git checkout kinetic-devel
  cd ../..
  rosdep update
  rosdep install robotiq_modbus_tcp
  sudo apt-get install ros-kinetic-soem
  rosdep install --from-paths src/ --ignore-src --rosdistro kinetic
  catkin_make
  source devel/setup.bash
  
Then run codes with :code:`hand_name:=robotiq_3f` as argument  

  **Note** The original urdf from robotiq has been modified. In their file, all joints are actuated. In truth, watching video of how the hand moves, there should be a unique joint that close all the fingers and another one that spread the two fingers on on side of the palm. So mimic tag for phalanges were added. Friction and damping were inserted for the joints so the model can be used in gazebo. Other addition are contact coefficent (of tips) and colors. These parameters obviosly can be very different from the real hand.


.. _`Robotiq 2F-140 Gripper`:

Robotiq 2F-140 Gripper
**************************

This is an industrial parallel gripper with a single actuated joint.
More information at at `Robotiq website <https://robotiq.com/products/2f85-140-adaptive-robot-gripper/>`_.

Necessary files are in the same repository of :ref:`Robotiq 3-Finger Gripper` so follow these steps. 

Then launch the nodes as usual with :code:`hand_name:=robotiq_2f_140` as argument 



   
