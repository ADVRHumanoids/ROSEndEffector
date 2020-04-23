.. _issues:

.. role:: raw-html(raw)
    :format: html

Possible Issues
========================

- From 28-01-2020 the *use_gui* param (in launch files) gives an error because it is deprecated. This causes the sliders of joint state publisher not shown. To solve : 

  .. code-block:: console
  
    sudo apt install ros-kinetic-joint-state-publisher-gui
    
  **NOTE** *use_gui* is not used anymore by us, but this answer can be useful anyway
  :raw-html:`<br />`
  :raw-html:`<br />`

- With the schunk hand, if we move only the middle finger (base phalange)
  toward the hand, a collision between index tip, middle tip and ring tip is detected. 
  Easy reproducible with the `moveit assistant <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html>`_, in the set pose section (when we move the middle finger it finds a collision when visually is not present)
  I dont know if it is a problem of schunk model, moveit, or both.
  :raw-html:`<br />`
  :raw-html:`<br />`
  

- Rviz does not visualize my model!

  - Be sure to have :code:`source devel/setup.bash` the package with robot meshes
    :raw-html:`<br />`
    :raw-html:`<br />`
  
  
  - Be sure to have set the right *Fixed Frame* in Global Options (usually it is called *base_something*, or *world*
    if you have set up your *urdf* file for Gazebo usage.
    :raw-html:`<br />`
    :raw-html:`<br />`
    
  - Be sure to have added the RobotModel (it should be present in the right menu)
  
