.. _googleTests:

.. role:: raw-html(raw)
    :format: html

How to run the google tests present in the package
===================================================

This is the developer zone for advanced users.

- Compile

  .. code-block:: console
  
    cd <ROSEE_pkg_path>/build
    make tests
    
- Run Test Run Test OPTION 1 (Colorful prints but all tests togheter)

  .. code-block:: console
  
    roslaunch ros_end_effector googleTest_run_all.launch 
    
  Check the **googleTest_run_all.launch** file to change the hand for the tests
  :raw-html:`<br />`
  :raw-html:`<br />`
  
- Run Test OPTION 2 (each test sequentially)

    With  :code:`make test ARGS="-V"` we first need to load the urdf and srdf file "manually" with :code:`rosparam set [..]`, for example:
    
    for test_ee hand:
    
    .. code-block:: console
    
      # for test_ee hand
      rosparam set -t $(rospack find ros_end_effector)/configs/urdf/test_ee.urdf robot_description
      rosparam set -t $(rospack find ros_end_effector)/configs/srdf/test_ee.srdf robot_description_semantic
      make test ARGS="-V"
    
    OR, for for two_finger hand:
    
    .. code-block:: console
    
      # for two_finger hand
      rosparam set -t $(rospack find ros_end_effector)/configs/urdf/two_finger.urdf robot_description
      rosparam set -t $(rospack find ros_end_effector)/configs/srdf/two_finger.srdf robot_description_semantic
      make test ARGS="-V"
      
  Similar for other hands.



