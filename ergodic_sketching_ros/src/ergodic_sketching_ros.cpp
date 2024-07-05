// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#include <algorithm>
#include <string>
#include <vector>
#include <array>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser_runtime/Interface.hpp>

#include <ergodic_sketching/ErgodicControl.hpp>
#include <ergodic_sketching/ErgodicSketching.hpp>
#include <ergodic_sketching/RobotDrawing.hpp>
#include <ergodic_sketching/TrajectoryUtils.hpp>

#include <ergodic_sketching_msgs/sketch.h>

sackmesser::runtime::Interface::Ptr interface;
std::unique_ptr<sketching::ErgodicSketching> ergodic_sketcher;
std::unique_ptr<sketching::RobotDrawing> robot_drawing;
std::string base_frame = "world";

bool sketch(ergodic_sketching_msgs::sketch::Request& req, ergodic_sketching_msgs::sketch::Response& resp) {
    cv_bridge::CvImagePtr image_ptr;
    try {
        image_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge_exception: %s", e.what());
        return false;
    }

    std::vector<sketching::ErgodicControl::Agent::Path> paths = ergodic_sketcher->sketch(image_ptr->image);
    std::vector<Eigen::Matrix<double, 7, 1>> path_7d = robot_drawing->process(paths, robot_drawing->getDrawingZonesTransforms()[req.drawing_zone_idx.data]);
    
    for(auto const& stroke : paths){
        nav_msgs::Path stroke_ros;
        stroke_ros.header.stamp = ros::Time::now();

        for(auto const& point: stroke){
            geometry_msgs::PoseStamped point_ros;
            point_ros.header.stamp = ros::Time::now();
            point_ros.pose.position.x = point(0);
            point_ros.pose.position.y = point(1);
            stroke_ros.poses.push_back(point_ros);
        }

        resp.strokes.push_back(stroke_ros);
    }

    resp.path.header.stamp = ros::Time::now();
    resp.path.header.frame_id = base_frame;

    for (auto const& pose : path_7d) {
        geometry_msgs::PoseStamped pose_ros;
        pose_ros.header.stamp = ros::Time::now();
        pose_ros.header.frame_id = base_frame;
        pose_ros.pose.position.x = pose(0);
        pose_ros.pose.position.y = pose(1);
        pose_ros.pose.position.z = pose(2);

        pose_ros.pose.orientation.w = pose(3);
        pose_ros.pose.orientation.x = pose(4);
        pose_ros.pose.orientation.y = pose(5);
        pose_ros.pose.orientation.z = pose(6);
        resp.path.poses.push_back(pose_ros);
    }

    return true;
}

std::array<double,3> stringArrayToDoubleArray(std::string raw_array) {
    // Trim from end
    raw_array.erase(std::find_if(raw_array.rbegin(), raw_array.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), raw_array.end());

    const std::string delimiter = " ";
    std::array<double,3> array;

    size_t index = 0;
    std::string token;
    while (true) {
        size_t pos = raw_array.find(delimiter);

        if (index > 2) {
            throw std::runtime_error("[urdf_parsing] Expected a 3d array got more, check arrays format.");
        }

        if (pos > 0) {
            token = raw_array.substr(0, pos);

            array[index] = std::stod(token);
            index++;
        }

        if (pos == std::string::npos) {
            break;
        }

        raw_array.erase(0, pos + delimiter.length());
    }
    if (index != 3) {
        throw std::runtime_error("[urdf_parsing] Expected a 3d array got less, check arrays format.");
    }

    return array;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle node_handle("~");

    std::string path;
    std::string config_file;
    std::string drawing_frame_rpy_str;
    std::string drawing_frame_xyz_str;

    if (!node_handle.getParam("path", path)) {
        ROS_ERROR("No parameters %s/path", node_handle.getNamespace().c_str());
        return 1;
    }

    if (!node_handle.getParam("config_file", config_file)) {
        ROS_ERROR("No parameters %s/config_file", node_handle.getNamespace().c_str());
        return 1;
    }

    if(!node_handle.getParam("/drawing_frame_rpy",drawing_frame_rpy_str)){
        ROS_ERROR("No parameters /drawing_frame_rpy");
        return 1;
    }

    if(!node_handle.getParam("/drawing_frame_xyz",drawing_frame_xyz_str)){
        ROS_ERROR("No parameters /drawing_frame_xyz");
        return 1;
    }

    node_handle.getParam("base_frame", base_frame);

    std::array<double,3> drawing_frame_rpy = stringArrayToDoubleArray(drawing_frame_rpy_str);
    std::array<double,3> drawing_frame_xyz = stringArrayToDoubleArray(drawing_frame_xyz_str);

    ROS_INFO("Ergodic sketching: path=%s", path.c_str());
    ROS_INFO("Ergodic sketching: config_file=%s", config_file.c_str());

    interface = std::make_shared<sackmesser::runtime::Interface>(path, config_file);
    ergodic_sketcher = std::make_unique<sketching::ErgodicSketching>(interface);
    robot_drawing = std::make_unique<sketching::RobotDrawing>(interface, "ergodic_sketching/robot_drawing/",drawing_frame_xyz,drawing_frame_rpy);

    ros::ServiceServer sketch_service = node_handle.advertiseService("sketch", sketch);

    ros::spin();

    return 0;
}
