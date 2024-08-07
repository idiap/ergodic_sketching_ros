// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#include <algorithm>
#include <string>
#include <vector>
#include <array>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include "rclcpp/rclcpp.hpp"

#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser_runtime/Interface.hpp>

#include <ergodic_sketching/ErgodicControl.hpp>
#include <ergodic_sketching/ErgodicSketching.hpp>
#include <ergodic_sketching/RobotDrawing.hpp>
#include <ergodic_sketching/TrajectoryUtils.hpp>

#include <ergodic_sketching_msgs/srv/sketch.hpp>

sackmesser::runtime::Interface::Ptr interface;
std::unique_ptr<sketching::ErgodicSketching> ergodic_sketcher;
std::unique_ptr<sketching::RobotDrawing> robot_drawing;
std::string base_frame = "world";
std::shared_ptr<rclcpp::Node> node;

void sketch(const std::shared_ptr<ergodic_sketching_msgs::srv::Sketch::Request>& req, std::shared_ptr<ergodic_sketching_msgs::srv::Sketch::Response> resp) {
    RCLCPP_INFO(node->get_logger(),"Sketch request received");
    cv_bridge::CvImagePtr image_ptr;
    try {
        image_ptr = cv_bridge::toCvCopy(req->image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node->get_logger(),"cv_bridge_exception: %s", e.what());
        return;
    }

    std::vector<sketching::ErgodicControl::Agent::Path> paths = ergodic_sketcher->sketch(image_ptr->image);

    std::vector<Eigen::Matrix<double, 7, 1>> path_7d = robot_drawing->process(paths, robot_drawing->getDrawingZonesTransforms()[req->drawing_zone_idx.data]);
    
    for(auto const& stroke : paths){
        nav_msgs::msg::Path stroke_ros;
        stroke_ros.header.stamp = node->get_clock()->now();

        for(auto const& point: stroke){
            geometry_msgs::msg::PoseStamped point_ros;
            point_ros.header.stamp = node->get_clock()->now();
            point_ros.pose.position.x = point(0);
            point_ros.pose.position.y = point(1);
            stroke_ros.poses.push_back(point_ros);
        }

        resp->strokes.push_back(stroke_ros);
    }

    resp->path.header.stamp = node->get_clock()->now();
    resp->path.header.frame_id = base_frame;

    for (auto const& pose : path_7d) {
        geometry_msgs::msg::PoseStamped pose_ros;
        pose_ros.header.stamp = node->get_clock()->now();
        pose_ros.header.frame_id = base_frame;
        pose_ros.pose.position.x = pose(0);
        pose_ros.pose.position.y = pose(1);
        pose_ros.pose.position.z = pose(2);

        pose_ros.pose.orientation.w = pose(3);
        pose_ros.pose.orientation.x = pose(4);
        pose_ros.pose.orientation.y = pose(5);
        pose_ros.pose.orientation.z = pose(6);
        resp->path.poses.push_back(pose_ros);
    }

    return;
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
    rclcpp::init(argc,argv);
    node = std::make_shared<rclcpp::Node>("ergodic_sketching_ros");

    node->declare_parameter<std::string>("path");
    node->declare_parameter<std::string>("config_file");
    node->declare_parameter<std::string>("drawing_frame_rpy");
    node->declare_parameter<std::string>("drawing_frame_xyz");
    node->declare_parameter<std::string>("base_frame");

    std::string path;
    std::string config_file;
    std::string drawing_frame_rpy_str;
    std::string drawing_frame_xyz_str;

    if (!node->get_parameter("path",path)) {
        RCLCPP_ERROR(node->get_logger(),"No parameters ~/path");
        return 1;
    }

    if (!node->get_parameter("config_file",config_file)) {
        RCLCPP_ERROR(node->get_logger(),"No parameters ~/config_file");
        return 1;
    }

    if(!node->get_parameter("drawing_frame_rpy",drawing_frame_rpy_str)){
        RCLCPP_ERROR(node->get_logger(),"No parameters /drawing_frame_rpy");
        return 1;
    }

    if(!node->get_parameter("drawing_frame_xyz",drawing_frame_xyz_str)){
        RCLCPP_ERROR(node->get_logger(),"No parameters /drawing_frame_xyz");
        return 1;
    }

    if(!node->get_parameter("base_frame", base_frame)){
        RCLCPP_ERROR(node->get_logger(),"No parameters base_frame");
    }

    std::array<double,3> drawing_frame_rpy = stringArrayToDoubleArray(drawing_frame_rpy_str);
    std::array<double,3> drawing_frame_xyz = stringArrayToDoubleArray(drawing_frame_xyz_str);

    RCLCPP_INFO(node->get_logger(),"Ergodic sketching: path=%s", path.c_str());
    RCLCPP_INFO(node->get_logger(),"Ergodic sketching: config_file=%s", config_file.c_str());

    interface = std::make_shared<sackmesser::runtime::Interface>(path, config_file);
    ergodic_sketcher = std::make_unique<sketching::ErgodicSketching>(interface);
    robot_drawing = std::make_unique<sketching::RobotDrawing>(interface, "ergodic_sketching/robot_drawing/",drawing_frame_xyz,drawing_frame_rpy);

    rclcpp::Service<ergodic_sketching_msgs::srv::Sketch>::SharedPtr service = node->create_service<ergodic_sketching_msgs::srv::Sketch>(
        "sketch",
        &sketch
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
