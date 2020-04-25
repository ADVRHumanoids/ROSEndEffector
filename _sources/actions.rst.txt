.. _actions:

ROS End-Effector Grasping Actions
=====================================

**TODO** check explain better this page, put examples for create actions

In ROS End-Effector, an *Action* is a movement that the end effector can be commanded to do. In practice, an action is a container that include the position for all the involved joints. There are three main categories of Action:

- `Primitive Actions`_
- `Generic Actions`_
- `Timed Actions`_


.. _`Primitive Actions`:

Primitive Actions
###################

Referred as *ActionPrimitive* in the code, this kind of actions is the one that is automatically detected thanks to
the *FindAction* node (during the offline phase). Until now the following primitives have been defined:

.. list-table:: Primitive Table
   :widths: 20 20 20 20 20
   :header-rows: 1

   * - Primitive
     - Fingers
     - Joints
     - Description
     - Constraints
   * - **Trig**
     - 1
     - 1+
     - Move all finger joints toward their bound
     - At least one joint that moves only one finger
   * - **TipFlex**
     - 1
     - 1
     - Move the last joint of the finger toward its bound
     - At least 2 joints that moves only one finger
   * - **FingFlex**
     - 1
     - 1
     - Move the first joint of the finger toward its bound
     - At least 2 joints that moves only one finger
   * - **PinchTight**
     - 2
     - 1+
     - Collision between two fingertip links
     -
   * - **PinchWeak**
     - 2
     - 1+
     - Movement of two fingertips links towards each other but without collision
     -
   * - **MultiPinchTight_N**
     - N (:math:`\geq` 3)
     - 1+
     - Collision between N (:math:`\geq` 3) fingertip links
     -
   * - **SingleJointMultipleTips_N**
     - N (:math:`\geq` 2)
     - 1
     - Move a joint that move more than one finger toward its bound
     - A joint that move more than one finger (e.g. grasp for lot of hands

.. _`Generic Actions`:

Generic Actions
##################

*ActionGeneric* are the customizable type. With ROSEE, you can add a custom action and then command the end-effector to do that action many time you want during the execution. 
Generic Action can be built in 2 different ways:

- You can write by hand a *yaml* file (see files in *config/generic*
- You can fill c++ structures with your code, and then emit the yaml file (or use directly the action as command)

There is also a third possibility: creating a *ActionComposed* object. This is a class that permit you to define 
an action as sum of other actions. For example, a grasp can be see as a sum of *trig* of each finger.


.. _`Timed Actions`:

Timed Actions
##################

ActionTimed are a special type of actions that permit you to execute more inner actions *one after the other*.
As an example, this can be useful to perform a *wide grasp*. First we want the finger to spread, and then perform the clousure toward the palm. 
These type of action are creable in the code, using the class functions.


