.. _usage:

.. role:: raw-html(raw)
    :format: html

How to use ROS End-Effector with your End-Effector
==================================================

ROS End-Effector allows you to control your end-effector with minimal inital set-up

- If you want to use also gazebo and :ref:`ROS End-Effector Gazebo plugin <roseeGazeboPlugin>`, please prepare correctly your *.urdf* model following the steps :ref:`here <prepare4Gazebo>`

- Put *.urdf* and *.srdf* files in *ROSEndEffector/configs/urdf/* and *ROSEndEffector/configs/srdf/* respectively.
  See below for information on how to proper write your *.srdf* file
  Be sure that the files are named like the name of the robot tag of *.urdf* and *.srdf*
  
    **E.G.** *my_hand.urdf*, *my_hand.srdf*, with tags in the files like : :code:`<robot name="my_hand" [...] >`

  The meshes can be kept on your hand_description folder, only be sure to :code:`source` that *setup.bash* before run ROSEE nodes.

- Create a yaml config file to tell where the *.urdf* and *.srdf* files are, and put in *ROSEndEffector/configs/* folder, like this one:

  .. code-block:: yaml 

    ROSEndEffector:
      urdf_path: "urdf/my_hand.urdf"
      srdf_path: "srdf/my_hand.srdf"

  In future we will use this file in a better way so you will not need to put urdf and srdf file in ROSEE folder, and you will be able to specify custom path for these files


Creating SRDF files
#######################

Both *moveit* and this *ROSEE node* refers to SRDF file to explore your hand. So it is important to provide a compatible SRDF file. 
:raw-html:`<br />`
The convention used is that each finger is a *group* (from the base of the finger to the tip).
Even for very complicated hand like schunk hand, this file is easy to create (see the one for the schunk hand `here <https://github.com/ADVRHumanoids/ROSEndEffector/blob/devel/configs/srdf/schunk.srdf>`_).
:raw-html:`<br />`
You must simply add a chain for every group. Take care to include in the chain all the joint that move that group (i.e. that finger). The chain can also have a common root. You can refer to already present srdf to understand how to write this file

If you don't want to create this by hand, you can use the `moveit setup assistant <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html>`_ , which will help to create SRDF files (among the other things) through a GUI. Only be sure to create the group as *chain* (use *add_chain* and not *add_joint* not *add_link*)

In the *SRDF* file you can also indicate some joints as passive in this way:

.. code-block:: xml 

  <passive_joint name="NOME_JOINT" /> 

See for example the *robotiq_3f.srdf* file. Please **DO NOT** add the passive tag for joint that are mimic.  This can cause problem and it is useless, because mimic joints are already considered as non-actuated by ROSEE. 
Instead, the passive tag can be useful to indicate that a non-fixed joint is not actuated (for example, it can move only if some external forces is received, in a compliant way). 
It is not necessary to set the passive tag for fixed joints.


**TODO** link schunk srdf not on internet but on interal package file?


Find Grasping Actions (offline phase)
######################################
First thing is to let **UniversalFindActions** explore your model and extract the grasping primitives. 

.. code-block:: console

  # source the setup.bash of package where robot meshes are
  roslaunch ros_end_effector findActions.launch hand_name:=my_hand
  
The findActions node will generate yaml files in the *config/action/my_hand/* folder. 
The same yaml files are also parsed by the same node to test (and print) the correctness. 

**WARNING** old action *yaml* files will be ovewritten every time you run again the node.
Also, take care to generate correctly the SRDF (as described above).



Control your End-Effector with ROSEE (online phase)
####################################################

Now you can run the main controller.
This node will take grasping actions command (received as *ROS actions*) and will send joint positions to the simulation/real robot.

.. code-block:: console

  # source the setup.bash of package where robot meshes are
  roslaunch ros_end_effector rosee_startup.launch hand_name:=my_hand
  
This command will load ROSEE controller node, togheter with rviz for visualization purposes.

  **Note** You can add the argument :code:`inSlider:=true` if you want to not run ROSEE controller but load a joint publisher GUI to command directly each joint position.

In another terminal, you can run the GUI to easy send the action parsed before. Only be sure to have rosee_gui installed (one of the optional dependencies indicated in :ref:`Installation <install>` section).

.. code-block:: console

  roslaunch rosee_gui gui.launch #no hand name is needed

This will load the GUI dynamically, visualizing only the action specific for the end effector launched before.

You can also send commands directly throught *ROS actions* (that is what GUI does). 
You can simply :code:`pub` a *rosee_msg/ROSEECommandActionGoal* message on */ros_end_effector/action_command/goal* topic, and :code:`echo` on /*ros_end_effector/action_command/feedback* to receive the feedback.

For example, to publish:

.. code-block:: console
  :emphasize-lines: 13,14,15,16,17,18,19,20
  
  rostopic pub /ros_end_effector/action_command/goal rosee_msg/ROSEECommandActionGoal "header:
    seq: 0
      stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
  goal_id:
    stamp:
      secs: 0
      nsecs: 0
    id: ''
  goal:
    goal_action:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      action_name: 'pinchTight'
      action_type: 0
      actionPrimitive_type: 0
      selectable_items: ['left_hand_c', 'left_hand_s']
      percentage: 1.0" 

**NOTE** the important lines are the last ones.

To receive feedback:

.. code-block:: console
  
  rostopic echo /ros_end_effector/action_command/feedback 





Dynamic Simulation with Gazebo
********************************

Be sure to have installed the rosee_gazebo_plugin (one of the optional dependencies indicated in :ref:`Installation <install>` section) and your urdf model ready to be used with Gazebo (TODO see plugingazebo section). 

  **Note** Also remember that you have to run the *offline phase* if you have never run it for your hand.

Launch the main node with the :code:`gazebo:=true` argument, like this way:

.. code-block:: console

  # source the setup.bash of package where robot meshes are
  roslaunch ros_end_effector rosee_startup.launch hand_name:=my_hand gazebo:=true
  
As before, you can use the ROSEE GUI to send grasping action commands:

.. code-block:: console

  roslaunch rosee_gui gui.launch #no hand name is needed



