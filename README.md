# ergodic_sketching_ros

This repository contains the source code to run the drozBot portraitist robot over ROS1. It contains 3 different packages:

* [``ergodic_sketching``](./ergodic_sketching/), the core C++ library.
* [``ergodic_sketching_msgs``](./ergodic_sketching_msgs/), the definition of custom ROS messages, services, and actions.
* [``ergodic_sketching_ros``](./ergodic_sketching_ros/), the ROS interface of the ``ergodic_sketching_library``.
* [``ilqr_planner``](https://github.com/idiap/ilqr_planner), a library to optimize trajectory using iLQR. This library is added as a submodule.

## Installation

* Clone this repository inside your catkin workspace.
  * Initialize the submodules.
* Add in your catkin workspace the packages dependencies:
  * [``fp_description``](), the package containing description of the F&P Robotics robots. The package is provided by F&P directly.
* Build the workspace with ``catkin build``.
  
## Usage 

### Launch file

You can setup the pipeline for PRob robot with:

```bash
$ roslaunch ergodic_sketching_ros prob_sim.launch gui:=true 
```

The ``gui`` argument indicated wheter we want to use RVIZ or not.

This launch file starts two ROS nodes:

* ``/ergodic_sketching_ros`` a node responsible of the sketching part (transforming the image into a list of strokes). It advertises the following:
  * a ``/ergodic_sketching_ros/sketch (ergodic_sketching_msgs/sketch)`` service responsible of performing the sketching.
* ``/ilqr_planner_ros`` a node responsible of the planning part (transforming the task space strokes into joint states coordinates). It advertises the following:
  * a ``/ilqr_planner_ros/plan`` action responsible of the planning. Since the list of strokes is quite big, the action break the planning into smaller part and return part of the joint state trajectory as feedback.

A python script as been implemented to facilitate the use of the pipeline. See below.

### Python script

The script [``prob_draw.py``](./ergodic_sketching_ros/scripts/prob_draw.py) automatize the call to ``/ergodic_sketching_ros/sketch`` and ``/ilqr_planner_ros/plan`` and take the path to an image as input. It publishes the joint states to the ``prob_sim/joint_states`` topic:

```bash
$ python prob_draw.py -l <path_to_log_dir>  -i <path_to_image>
```

``<path_to_log_dir>`` is a path to save the log of the sketching. It saves the joint positions, velocities, and optimization cost for investigation.

Standard image formats are working with this script. The recommended image resolution is 950x650 (HxW). Other resolution might distort the drawing or make the computation slower.

## Parameters

* Sketching parameters can be found [here](ergodic_sketching/config/drozbot_config_prob.yaml), here are the most common parameters:
  * ``num_strokes``: the number of strokes per sketch.
  * ``num_agents``: the number of exploring agent for the ergodic exploration.
  * ``timesteps``: the path size explored by one agent.
  * ``drawing_orientation``: the end-effector orientation while drawing.
* Planner parameters can be found [here](ergodic_sketching_ros/config/ilqr_planner_config_prob.yaml):
  * ``q0``: initial joint configuration of the robot. The end-effector orientation should match ``drawing_orientation``.

## Arguments

Position of the drawing frame can be specified as argument with the roslaunch command:

```bash
$ roslaunch ergodic_sketching_ros prob_sim.launch gui:=true drawing_frame_xyz:=<position> drawing_frame_rpy:=<orientation>
```