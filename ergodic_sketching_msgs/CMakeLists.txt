# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

cmake_minimum_required(VERSION 3.16)

project(ergodic_sketching_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_compile_options(
    "-Wall"
    "-Wextra"
    "-Werror=vla"
    "-Wno-unused-function"
    "-Wno-missing-braces"
    "-Wno-unknown-pragmas"
    "-Wno-parentheses"
    "-pedantic"
    "-Wconversion"
    "-Werror=pedantic"
    "-O3"
)

find_package(catkin REQUIRED COMPONENTS
	roscpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    actionlib
    actionlib_msgs
    trajectory_msgs
    message_generation
)

add_service_files(FILES sketch.srv)
add_action_files(
    DIRECTORY action
    FILES Planner.action
)
generate_messages(DEPENDENCIES actionlib_msgs trajectory_msgs sensor_msgs nav_msgs std_msgs geometry_msgs)

catkin_package(
	CATKIN_DEPENDS roscpp message_runtime sensor_msgs nav_msgs std_msgs geometry_msgs actionlib actionlib_msgs trajectory_msgs
)
