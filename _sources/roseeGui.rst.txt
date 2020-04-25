.. _roseeGui:

ROS End-Effector GUI
==================================

The GUI package is hosted on *github* `here <https://github.com/ADVRHumanoids/rosee_gui>`_. 

Prerequisite
#################
QT5 (I am not sure if this is given with ROS, if this is necessary only to compile, I will check)

How To Install
################

**NOTE** You probably already installed this if you have followed the steps in :ref:`Install <install>` section

.. code-block:: console

  git clone -b <branch_you_want> https://github.com/ADVRHumanoids/rosee_gui
  
  compile with catkin_make
  
Details
##########

How it works - code structure (improvable obviously)
***********************************************************

- **main.cpp** : It handles ROS (creating the nodehandle) and the Qapplication. It creates the Window object
- **Window.cpp** a *QWidget* derived class which refers to the gui Window. It has as member a *QGridLayout*, and it creates all the inner *QGridLayout* (one for each action)
- **ActionLayout.cpp** *QGridLayout* derived class, which contain labels buttons and other widgets. It also send message to ros topic, handling a ros publisher with a nodehandle passed to its costructor
- **ActionBoxesLayout.cpp** Derived class from above, it includes checkboxes to select finger/joint and send them also with topics. 
- **ActionTimedLayout.cpp**, **ActionTimedElement.cpp** Container and element for the timed action, which per definition is composed by other actions executed one after another with some time margins. Each element has three progress bar so in the future we can take feedback and display a progress of the action execution, included time margins
