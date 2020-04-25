.. _nodes:


FindActions & Executor node
=====================================

**TODO** complete check this page

FindActions
#################

- This node will explore your robot model (i.e. *urdf* and *srdf* files) in order to find some grasping primitive actions (e.g. __pinch__, __trig__ ) that your end-effector can do. 

- MoveIt library is used to parse the models and to find collisions (e.g. to find the **pinches**)

- Information about each grasping action are stored in some *yaml* files, that can be parsed afterwards. The main information in the *yaml* files is the necessary position of the joints to perform that grasping action (see some examples [here](configs/actionExamples)).

- Additional grasping primitive actions can be added, deriving the c++ class or creating directly by hand the *yaml* file.

Executor
################
