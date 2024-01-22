// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <Eigen/Core>
#include <array>
#include <vector>

namespace sketching {
class TrajectoryUtils {
public:
    static std::vector<Eigen::Matrix<double, 7, 1>> saturate_speed(const std::vector<Eigen::Matrix<double, 7, 1>>& trajectory, const double& max_speed = 0.1, const double& dt = 0.01);

    static std::vector<std::vector<Eigen::Vector3d>> optimize_trajectory_list(std::vector<std::vector<Eigen::Vector3d>>& trajectories, const int& start_idx = 0);

    static std::vector<Eigen::Matrix<double, 7, 1>> trajectory_list_to_continuous(const std::vector<std::vector<Eigen::Vector3d>>& trajectories,
                                                                                  const double& offset,
                                                                                  const Eigen::Vector4d& orientation,
                                                                                  const Eigen::Matrix4d& transform,
                                                                                  const double& max_speed,
                                                                                  const double& dt,
                                                                                  const int& min_kp = 15);

    static std::vector<double> generateQuadraticCurve(const int& nb_points, const double& max_height);

    static void min_max_norm(std::vector<std::vector<Eigen::Vector3d>>& paths_3d, const Eigen::Vector3d& max_value, const Eigen::Vector3d& min_value);

    static void min_max_norm_1d(std::vector<Eigen::Vector3d>& path_3d, const Eigen::Vector3d& max_value, const Eigen::Vector3d& min_value);

    static void apply_transformation_to_trajectory(std::vector<Eigen::Vector3d>& trajectory, const Eigen::Matrix4d& transform);

    static void interp_between_two_points_pos(std::vector<Eigen::Vector3d>& dest, const Eigen::Vector3d& start, const Eigen::Vector3d& end, const int& horizon);

    static std::vector<Eigen::Matrix<double, 7, 1>> vStackOrientation(const std::vector<Eigen::Vector3d>& traj_pos, const Eigen::Vector4d& orientation);

    static std::vector<Eigen::Matrix<double, 7, 1>> vStackOrientations(const std::vector<Eigen::Vector3d>& traj_pos, const std::vector<Eigen::Vector4d>& orientations);

    static std::vector<Eigen::Vector3d> splineInterpolationPosition(const std::vector<Eigen::Vector3d>& points, const std::vector<double>& timesteps, const int& nb_points);

    static Eigen::Vector4d quaternionSlerp(const Eigen::Vector4d& origin, const Eigen::Vector4d& dest, const double& t);

    static std::vector<Eigen::Vector4d> generateQuaternionTrajectory(const std::vector<Eigen::Vector4d>& orientations, const std::vector<double>& timesteps, const int& nb_points);

    static std::vector<std::vector<Eigen::Vector2d>> buildSignature(const std::string& input_path);

    static std::array<Eigen::Vector2d, 2> getMinMax(const std::vector<std::vector<Eigen::Vector2d>>& strokes);

    static void applyOffset2d(std::vector<std::vector<Eigen::Vector2d>>& paths, const Eigen::Vector2d& offset);
};
}  // namespace sketching
