// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <Eigen/Geometry>
#include <algorithm>
#include <ergodic_sketching/RobotDrawing.hpp>
#include <ergodic_sketching/TrajectoryUtils.hpp>

#include <fstream>
#include <functional>
#include <limits>
#include <sackmesser/Interface.hpp>

template <typename T, size_t array_size>
std::array<T, array_size> stdVectorToStdArray(const std::vector<T>& vec) {
    std::array<T, array_size> arr;
    std::copy_n(vec.begin(), array_size, arr.begin());

    return arr;
}

namespace sketching {
bool RobotDrawing::Configuration::load(const std::shared_ptr<sackmesser::ConfigurationServer>& server) {
    bool loaded = server->loadParameter(ns_ + "dt", &dt, false) && server->loadParameter(ns_ + "init_phase_length", &init_phase_length, false) &&
                  server->loadParameter(ns_ + "max_cart_vel", &max_cart_vel, false) && server->loadParameter(ns_ + "offset", &offset, false) &&
                  server->loadParameter(ns_ + "sheet_x_size", &sheet_x_size, false) && server->loadParameter(ns_ + "sheet_y_size", &sheet_y_size, false) &&
                  server->loadParameter(ns_ + "sheet_margin_x", &sheet_margin_x, false) && server->loadParameter(ns_ + "sheet_margin_y", &sheet_margin_y, false) &&
                  server->loadParameter(ns_ + "signature_width_ratio", &signature_width_ratio, false);

    int num_drawing_zone;
    server->loadParameter(ns_ + "num_drawing_zone", &num_drawing_zone, false);

    for (int i = 1; i <= num_drawing_zone; i++) {
        double dw, dx, dy, dz;
        loaded = loaded && server->loadParameter(ns_ + "drawing_zone_" + std::to_string(i) + "/drawing_orientation/w", &dw, false) &&
                 server->loadParameter(ns_ + "drawing_zone_" + std::to_string(i) + "/drawing_orientation/x", &dx, false) &&
                 server->loadParameter(ns_ + "drawing_zone_" + std::to_string(i) + "/drawing_orientation/y", &dy, false) &&
                 server->loadParameter(ns_ + "drawing_zone_" + std::to_string(i) + "/drawing_orientation/z", &dz, false);

        Eigen::Vector4d drawing_orientation;
        drawing_orientation << dw, dx, dy, dz;

        RobotDrawing::DrawingZone zone;
        zone.orientation = drawing_orientation;
        drawing_zones.push_back(zone);
    }

    return loaded;
}

std::vector<RobotDrawing::DrawingZone> RobotDrawing::getDrawingZonesTransforms() {
    return config_.drawing_zones;
}

RobotDrawing::RobotDrawing(const std::shared_ptr<sackmesser::Interface>& interface, const std::string& name, const std::array<double, 3> transform_xyz, const std::array<double, 3> transform_rpy)
    : config_(name), interface_(interface) {
    if (!config_.load(interface->configServer())) {
        throw "RobotDrawing: failed to configure";
    }

    /*
        <arg name="drawing_frame_xyz" default="0.4 0.15 0.5"/>
        <arg name="drawing_frame_rpy" default="0.52 0 -1.5708"/>
                        rpy:
                        r: 0
                        p: -0.52
                        y: -1.5708
                    position:
                        x: 0.4
                        y: 0.15
                        z: 0.5

    */

    Eigen::AngleAxis<double> rollAngle(transform_rpy[1], Eigen::Vector3d::UnitX());
    Eigen::AngleAxis<double> pitchAngle(-transform_rpy[0], Eigen::Vector3d::UnitY());
    Eigen::AngleAxis<double> yawAngle(transform_rpy[2], Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

    Eigen::Matrix4d transform;
    transform = Eigen::Matrix4d::Identity();
    transform.topLeftCorner(3, 3) = q.matrix();
    transform.rightCols(1) << transform_xyz[0], transform_xyz[1], transform_xyz[2], 1;

    for (auto& drawing_zone : config_.drawing_zones) {
        drawing_zone.transform = transform;
    }
}

std::vector<Eigen::Matrix<double, 7, 1>> RobotDrawing::process_signature(const std::vector<std::vector<Eigen::Vector2d>>& signature, const RobotDrawing::DrawingZone& drawing_zone) {
    std::array<Eigen::Vector2d, 2> min_max = TrajectoryUtils::getMinMax(signature);

    // Min/Max normalization
    Eigen::Vector3d max_value;
    Eigen::Vector3d min_value;

    max_value << min_max[1], 1;
    min_value << min_max[0], 0;

    double x_over_y = (min_max[1](0) - min_max[0](0)) / (min_max[1](1) - min_max[0](1));

    std::vector<std::vector<Eigen::Vector3d>> paths_3d;
    for (auto path_2d : signature) {
        std::vector<Eigen::Vector3d> path_3d;
        for (auto point_2d : path_2d) {
            Eigen::Vector3d point_3d;
            point_3d << point_2d, 0;
            path_3d.push_back(point_3d);
        }
        paths_3d.push_back(path_3d);
    }

    TrajectoryUtils::min_max_norm(paths_3d, max_value, min_value);

    const double x_size = config_.sheet_x_size - 2 * config_.sheet_margin_x;
    double signature_width = config_.signature_width_ratio * x_size;
    double signature_height = signature_width * 1 / x_over_y;

    Eigen::Matrix4d scale_transform = Eigen::Matrix4d::Identity();
    scale_transform(0, 0) = signature_width;
    scale_transform(1, 1) = -signature_height;
    scale_transform(0, 3) = config_.sheet_x_size - config_.sheet_margin_x - signature_width - 0.005;
    scale_transform(1, 3) = config_.sheet_margin_y + signature_height + 0.005;

    for (auto& stroke : paths_3d) {
        // Make the trajectory fit on the desired paper size
        TrajectoryUtils::apply_transformation_to_trajectory(stroke, scale_transform);
    }

    paths_3d.at(0).at(0)(2) += 0.01;

    // Apply table transformation matrix
    for (auto& stroke : paths_3d) {
        TrajectoryUtils::apply_transformation_to_trajectory(stroke, drawing_zone.transform);
    }

    Eigen::Vector3d last_point = paths_3d.at(paths_3d.size() - 1).at(paths_3d.at(paths_3d.size() - 1).size() - 1);
    last_point = last_point + 0.05 * drawing_zone.transform.col(2).head(3).normalized();
    paths_3d.at(paths_3d.size() - 1).push_back(last_point);

    return TrajectoryUtils::trajectory_list_to_continuous(paths_3d, config_.offset, drawing_zone.orientation, drawing_zone.transform, config_.max_cart_vel, config_.dt);
}

std::vector<Eigen::Matrix<double, 7, 1>> RobotDrawing::process(const std::vector<std::vector<Eigen::Vector2d>>& paths, const RobotDrawing::DrawingZone& drawing_zone) {
    std::array<Eigen::Vector2d, 2> min_max = TrajectoryUtils::getMinMax(paths);

    // Min/Max normalization
    Eigen::Vector3d max_value;
    Eigen::Vector3d min_value;

    max_value << min_max[1], 1;
    min_value << min_max[0], 0;

    double x_over_y = (min_max[1](0) - min_max[0](0)) / (min_max[1](1) - min_max[0](1));
    double sheet_x_over_y = ((double)(config_.sheet_x_size - 2 * config_.sheet_margin_x)) / (config_.sheet_y_size - 2 * config_.sheet_margin_y);

    if (x_over_y < sheet_x_over_y) {
        x_over_y = sheet_x_over_y;
    }

    std::vector<std::vector<Eigen::Vector3d>> paths_3d;
    for (auto path_2d : paths) {
        std::vector<Eigen::Vector3d> path_3d;
        for (auto point_2d : path_2d) {
            Eigen::Vector3d point_3d;
            point_3d << point_2d, 0;
            path_3d.push_back(point_3d);
        }
        paths_3d.push_back(path_3d);
    }

    TrajectoryUtils::min_max_norm(paths_3d, max_value, min_value);

    const double x_ratio = config_.sheet_x_size - 2 * config_.sheet_margin_x;

    Eigen::Matrix4d scale_transform = Eigen::Matrix4d::Identity();

    scale_transform(0, 0) = x_ratio;
    scale_transform(1, 1) = x_ratio * 1.0 / x_over_y;

    for (auto& stroke : paths_3d) {
        // Make the trajectory fit on the desired paper size
        TrajectoryUtils::apply_transformation_to_trajectory(stroke, scale_transform);

        // Apply margin on paper sheet
        Eigen::Vector3d sheet_margin;
        sheet_margin << config_.sheet_margin_x, config_.sheet_margin_y, 0;

        for (auto& point : stroke) {
            point.head(3) = point.head(3) + sheet_margin;
        }
    }

    // Draw the frame on the paper sheet
    /*std::vector<Eigen::Vector3d> frame;
    Eigen::Vector3d corner_1;
    corner_1 << config_.sheet_margin_x, config_.sheet_margin_y, 0;
    frame.push_back(corner_1);

    Eigen::Vector3d corner_2;
    corner_2 << config_.sheet_x_size - config_.sheet_margin_x, config_.sheet_margin_y, 0;
    frame.push_back(corner_2);

    Eigen::Vector3d corner_3;
    corner_3 << config_.sheet_x_size - config_.sheet_margin_x, config_.sheet_y_size - config_.sheet_margin_y, 0;
    frame.push_back(corner_3);

    Eigen::Vector3d corner_4;
    corner_4 << config_.sheet_margin_x, config_.sheet_y_size - config_.sheet_margin_y, 0;
    frame.push_back(corner_4);
    frame.push_back(corner_1);

    Eigen::Vector3d end_position = corner_1;
    end_position(2) += 0.01;
    frame.push_back(end_position);*/

    // Apply table transformation matrix
    for (auto& stroke : paths_3d) {
        TrajectoryUtils::apply_transformation_to_trajectory(stroke, drawing_zone.transform);
    }

    // Optimize & Bridge
    int max_stroke_idx = 0;
    int current_index = 0;
    Eigen::Vector3d max_starting_point = Eigen::Vector3d::Zero();

    for (auto const& path : paths_3d) {
        if (path[0].norm() > max_starting_point.norm()) {
            max_starting_point = path[0];
            max_stroke_idx = current_index;
        }
        current_index++;
    }

    std::vector<std::vector<Eigen::Vector3d>> paths_3d_optimized = TrajectoryUtils::optimize_trajectory_list(paths_3d, max_stroke_idx);

    // TrajectoryUtils::apply_transformation_to_trajectory(frame, drawing_zone.transform);
    // paths_3d_optimized.push_back(frame);

    Eigen::Vector3d last_point = paths_3d_optimized.at(paths_3d_optimized.size() - 1).at(paths_3d_optimized.at(paths_3d_optimized.size() - 1).size() - 1);
    last_point = last_point + 0.05 * drawing_zone.transform.col(2).head(3).normalized();
    paths_3d_optimized.at(paths_3d_optimized.size() - 1).push_back(last_point);

    return TrajectoryUtils::trajectory_list_to_continuous(paths_3d_optimized, config_.offset, drawing_zone.orientation, drawing_zone.transform, config_.max_cart_vel, config_.dt);
}
}  // namespace sketching

// namespace sketching
