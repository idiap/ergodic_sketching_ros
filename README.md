<!--
SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>

SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>

SPDX-License-Identifier: GPL-3.0-only
-->

<span style="color:RED">*ROS2 Version!*</span>.

# ergodic_sketching_ros

This repository contains the source code to run the drozBot portraitist robot over ROS1. It contains 3 different packages:

* [``ergodic_sketching``](./ergodic_sketching/), the core C++ library.
* [``ergodic_sketching_msgs``](./ergodic_sketching_msgs/), the definition of custom ROS messages, services, and actions.
* [``ergodic_sketching_ros``](./ergodic_sketching_ros/), the ROS interface of the ``ergodic_sketching_library``.

The package also provides the following submodules: 

  * [``ilqr_planner``](https://github.com/idiap/ilqr_planner), a library to optimize trajectory using iLQR.
  * [``iiwa_ros2``](https://github.com/ICube-Robotics/iiwa_ros2), the package containing description of the KUKA iiwa LWR robot. The package is available on github with a BSD license.
  
## Installation

* Clone this repository inside the `src` folder of your colcon workspace.
  * Initialize the submodules: `git submodule update --init --recursive`
* Build the workspace with ``colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release``.
  
## Usage 

### Launch file

You can setup the pipeline for the kuka robot with:

```bash
$ ros2 launch ergodic_sketching_ros iiwa_drawing.launch.py
```

The ``use_gui`` argument indicated wheter we want to use RVIZ or not.

This launch file starts two ROS nodes:

* ``/ergodic_sketching_ros`` a node responsible of the sketching part (transforming the image into a list of strokes). It advertises the following:
  * a ``/sketch (ergodic_sketching_msgs/srv/Sketch)`` service responsible of performing the sketching.
* ``/planner_action_server`` a node responsible of the planning part (transforming the task space strokes into joint states coordinates). It advertises the following:
  * a ``/plan`` action responsible of the planning. Since the list of strokes is quite big, the action break the planning into smaller part and return part of the joint state trajectory as feedback.

A python script as been implemented to facilitate the use of the pipeline. See below.

### Python script

The script [``robot_sketch.py``](./ergodic_sketching_ros/scripts/robot_sketch.py) automatize the call to ``/sketch`` and ``/plan`` and take the path to an image as input. It publishes the joint states to the ``rob_sim/joint_states`` topic:

```bash
$ python robot_sketch.py -l <path_to_log_dir>  -i <path_to_image>
```

``<path_to_log_dir>`` is a path to save the log of the sketching. It saves the joint positions, velocities, and optimization cost for investigation.

A test image is available in the `images` folder.

When this script is used with the launch file described previously, it will automatically display all the results on RVIZ.

Standard image formats are working with this script. The recommended image resolution is 950x650 (HxW). Other resolution might distort the drawing or make the computation slower.

## Parameters

* Sketching parameters can be found [here](ergodic_sketching_ros/config/drozbot_config_iiwa.yaml), here are the most common parameters:
  * ``num_strokes``: the number of strokes per sketch.
  * ``num_agents``: the number of exploring agent for the ergodic exploration.
  * ``timesteps``: the path size explored by one agent.
  * ``drawing_orientation``: the end-effector orientation while drawing.
* Planner parameters can be found [here](ergodic_sketching_ros/config/ilqr_planner_config_iiwa.yaml):
  * ``q0``: initial joint configuration of the robot. The end-effector orientation should match ``drawing_orientation``.

## Arguments

Position of the drawing frame can be specified as argument with the roslaunch command:

```bash
$ ros2 launch ergodic_sketching_ros iiwa_drawing.launch.py use_gui:=true drawing_frame_xyz:=<position> drawing_frame_rpy:=<orientation>
```