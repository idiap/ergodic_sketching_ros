# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

cmake_minimum_required(VERSION 3.16)

project(ergodic_sketching_ros)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ergodic_sketching REQUIRED)
find_package(ilqr_planner REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sackmesser REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ergodic_sketching_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ergodic_sketching_ros src/ergodic_sketching_ros.cpp)

target_include_directories(ergodic_sketching_ros
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies( ergodic_sketching_ros 
    rclcpp
    ergodic_sketching
    sackmesser
    OpenCV
    Eigen3
    cv_bridge
    ergodic_sketching_msgs
    nav_msgs
    geometry_msgs
)

add_library(ilqr_planner_ros SHARED src/ilqr_planner_ros.cpp src/Planner.cpp include/ergodic_sketching_ros/Planner.hpp)
target_include_directories(ilqr_planner_ros
	PRIVATE
  	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies( ilqr_planner_ros 
    rclcpp
    rclcpp_action
    rclcpp_components
    ilqr_planner
    Eigen3
    ergodic_sketching_msgs
    trajectory_msgs
)

rclcpp_components_register_node(ilqr_planner_ros PLUGIN "PlannerActionServer" EXECUTABLE planner_action_server)

install(TARGETS ergodic_sketching_ros
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ilqr_planner_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch config descriptions assets rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()