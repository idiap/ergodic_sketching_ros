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
    #"-O3"
)

find_package(ergodic_sketching REQUIRED)
find_package(ilqr_planner REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sackmesser REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
    rospy
    cv_bridge
    actionlib
    actionlib_msgs
    ergodic_sketching_msgs
)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
	CATKIN_DEPENDS roscpp rospy message_runtime cv_bridge actionlib actionlib_msgs ergodic_sketching_msgs
)

add_executable(ergodic_sketching_ros src/ergodic_sketching_ros.cpp)
add_executable(ilqr_planner_ros src/ilqr_planner_ros.cpp src/Planner.cpp include/ergodic_sketching_ros/Planner.hpp)

target_include_directories(ergodic_sketching_ros
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
	SYSTEM PRIVATE
		${catkin_INCLUDE_DIRS}
	SYSTEM INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)

target_link_libraries(ergodic_sketching_ros  PUBLIC
    ${catkin_LIBRARIES}
    ergodic_sketching::ergodic_sketching
    sackmesser::sackmesser
    sackmesser::sackmesser_runtime
    ${OpenCV_LIBRARIES}
	Eigen3::Eigen
)

target_include_directories(ilqr_planner_ros
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
	SYSTEM PRIVATE
		${catkin_INCLUDE_DIRS}
	SYSTEM INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)

target_link_libraries(ilqr_planner_ros  PUBLIC
    ${catkin_LIBRARIES}
	Eigen3::Eigen
    ilqr_planner::ilqr_planner
)

catkin_install_python(PROGRAMS scripts/ik_node.py scripts/ik_marker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
