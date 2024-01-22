// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <cmath>
#include <ctime>
#include <limits>

#include <Eigen/Geometry>

#include <unsupported/Eigen/Splines>

#include <ergodic_sketching/Serialization.hpp>
#include <ergodic_sketching/TrajectoryUtils.hpp>

namespace sketching {
std::vector<Eigen::Matrix<double, 7, 1>> TrajectoryUtils::saturate_speed(const std::vector<Eigen::Matrix<double, 7, 1>>& trajectory, const double& max_speed, const double& dt) {
    std::vector<Eigen::Matrix<double, 7, 1>> trajectory_saturated;
    for (unsigned idx = 0; idx < trajectory.size() - 1; idx++) {
        trajectory_saturated.push_back(trajectory.at(idx));

        Eigen::Vector3d delta_traj = trajectory.at(idx + 1).head(3) - trajectory.at(idx).head(3);

        double speed = delta_traj.norm() / dt;

        if (speed > max_speed) {
            int nb_kp_needed = static_cast<int>(std::ceil(speed / max_speed));

            Eigen::Vector3d delta_traj_step = delta_traj / nb_kp_needed;
            Eigen::Vector4d orientation = trajectory.at(idx).tail(4);
            Eigen::Vector4d orientation_p1 = trajectory.at(idx + 1).tail(4);

            for (unsigned idx_saturate = 0; idx_saturate < nb_kp_needed; idx_saturate++) {
                double t = (idx_saturate + 1.0) / (nb_kp_needed + 1.0);
                Eigen::Vector4d orientation_t = TrajectoryUtils::quaternionSlerp(orientation, orientation_p1, t);

                Eigen::Matrix<double, 7, 1> state_saturated_t;
                state_saturated_t << trajectory.at(idx).head(3) + (idx_saturate + 1) * delta_traj_step, orientation_t;

                trajectory_saturated.push_back(state_saturated_t);
            }
        }
    }

    trajectory_saturated.push_back(trajectory.at(trajectory.size() - 1));
    return trajectory_saturated;
}

void TrajectoryUtils::min_max_norm_1d(std::vector<Eigen::Vector3d>& path_3d, const Eigen::Vector3d& max_value, const Eigen::Vector3d& min_value) {
    for (int i = 0; i < path_3d.size(); i++) {
        path_3d.at(i) = (path_3d.at(i) - min_value).array() / (max_value - min_value).array();
    }
}

void TrajectoryUtils::min_max_norm(std::vector<std::vector<Eigen::Vector3d>>& paths_3d, const Eigen::Vector3d& max_value, const Eigen::Vector3d& min_value) {
    for (int i = 0; i < paths_3d.size(); i++) {
        TrajectoryUtils::min_max_norm_1d(paths_3d.at(i), max_value, min_value);
    }
}

std::vector<std::vector<Eigen::Vector3d>> TrajectoryUtils::optimize_trajectory_list(std::vector<std::vector<Eigen::Vector3d>>& trajectories, const int& start_idx) {
    std::vector<Eigen::Vector3d> traj_to_process = trajectories.at(start_idx);
    int index = start_idx;

    int nb_processed = 1;
    int nb_trajectories = trajectories.size();

    std::vector<std::vector<Eigen::Vector3d>> trajectories_sorted;

    while (nb_processed < nb_trajectories) {
        trajectories.erase(trajectories.begin() + index);
        Eigen::Vector3d ending_point = traj_to_process.at(traj_to_process.size() - 1);

        double min_distance = std::numeric_limits<double>::max();
        unsigned min_index = 0;

        for (unsigned i = 0; i < trajectories.size(); i++) {
            std::vector<Eigen::Vector3d> traj = trajectories.at(i);
            double distance = (traj.at(0) - ending_point).norm();

            if (distance < min_distance) {
                min_distance = distance;
                min_index = i;
            }
        }

        trajectories_sorted.push_back(traj_to_process);
        traj_to_process = trajectories.at(min_index);
        index = min_index;
        nb_processed += 1;
    }

    trajectories_sorted.push_back(trajectories.at(0));
    return trajectories_sorted;
}

std::vector<Eigen::Matrix<double, 7, 1>> TrajectoryUtils::vStackOrientation(const std::vector<Eigen::Vector3d>& traj_pos, const Eigen::Vector4d& orientation) {
    std::vector<Eigen::Matrix<double, 7, 1>> trajectory;

    for (auto pos_t : traj_pos) {
        Eigen::Matrix<double, 7, 1> state_t;
        state_t << pos_t, orientation;
        trajectory.push_back(state_t);
    }

    return trajectory;
}

std::vector<Eigen::Matrix<double, 7, 1>> TrajectoryUtils::vStackOrientations(const std::vector<Eigen::Vector3d>& traj_pos, const std::vector<Eigen::Vector4d>& orientations) {
    std::vector<Eigen::Matrix<double, 7, 1>> trajectory;

    int idx = 0;
    for (auto pos_t : traj_pos) {
        Eigen::Matrix<double, 7, 1> state_t;
        Eigen::Vector4d orientation = orientations.at(idx);
        state_t << pos_t, orientation;
        trajectory.push_back(state_t);
        idx++;
    }

    return trajectory;
}

std::vector<double> TrajectoryUtils::generateQuadraticCurve(const int& nb_points, const double& max_height) {
    std::vector<double> curve;
    double scaling_factor = max_height / ((nb_points * nb_points) / 4);
    for (int t = 0; t <= nb_points; t++) {
        double value = -1 * scaling_factor * t * (t - nb_points);
        curve.push_back(value);
    }
    return curve;
}

std::vector<Eigen::Matrix<double, 7, 1>> TrajectoryUtils::trajectory_list_to_continuous(const std::vector<std::vector<Eigen::Vector3d>>& trajectories,
                                                                                        const double& offset,
                                                                                        const Eigen::Vector4d& orientation,
                                                                                        const Eigen::Matrix4d& transform,
                                                                                        const double& max_speed,
                                                                                        const double& dt,
                                                                                        const int& min_kp) {
    std::vector<Eigen::Matrix<double, 7, 1>> trajectory;

    for (int traj_idx = 1; traj_idx < trajectories.size(); traj_idx++) {
        std::vector<Eigen::Vector3d> traj_t = trajectories.at(traj_idx);
        std::vector<Eigen::Vector3d> traj_tm1 = trajectories.at(traj_idx - 1);

        Eigen::Vector3d gap_start = traj_tm1.at(traj_tm1.size() - 1);
        Eigen::Vector3d gap_end = traj_t.at(0);

        std::vector<Eigen::Matrix<double, 7, 1>> traj_tm1_full = TrajectoryUtils::vStackOrientation(traj_tm1, orientation);
        std::vector<Eigen::Matrix<double, 7, 1>> traj_tm1_full_saturated = TrajectoryUtils::saturate_speed(traj_tm1_full, max_speed, dt);
        trajectory.insert(trajectory.end(), traj_tm1_full_saturated.begin(), traj_tm1_full_saturated.end());

        int nb_kp = (int)((gap_end - gap_start).norm() / max_speed);
        if (nb_kp < min_kp) {
            nb_kp = min_kp;
        }

        double offset_t = ((gap_end - gap_start).norm() / 0.025) * offset;
        if (offset_t > 0.075) {
            offset_t = 0.075;
        } else if (offset_t < 0.005) {
            offset_t = 0.005;
        }

        std::vector<double> z_evolution = TrajectoryUtils::generateQuadraticCurve(nb_kp, offset_t);
        std::vector<Eigen::Vector3d> xy_evolution;
        Eigen::Vector3d normal_z = transform.col(2).head(3).normalized();
        TrajectoryUtils::interp_between_two_points_pos(xy_evolution, gap_start, gap_end, nb_kp);

        std::vector<Eigen::Matrix<double, 7, 1>> gap_trajectory_full;

        for (int i = 0; i < z_evolution.size(); i++) {
            Eigen::Matrix<double, 7, 1> gap_t;
            Eigen::Vector3d gap_pos = xy_evolution.at(i) + z_evolution.at(i) * normal_z;
            gap_t << gap_pos, orientation;
            gap_trajectory_full.push_back(gap_t);
        }
        std::vector<Eigen::Matrix<double, 7, 1>> gap_trajectory_full_saturated = TrajectoryUtils::saturate_speed(gap_trajectory_full, max_speed, dt);
        trajectory.insert(trajectory.end(), gap_trajectory_full_saturated.begin(), gap_trajectory_full_saturated.end());
    }

    std::vector<Eigen::Vector3d> last_traj = trajectories.at(trajectories.size() - 1);
    std::vector<Eigen::Matrix<double, 7, 1>> last_traj_full = TrajectoryUtils::vStackOrientation(last_traj, orientation);
    std::vector<Eigen::Matrix<double, 7, 1>> last_traj_full_saturated = TrajectoryUtils::saturate_speed(last_traj_full, max_speed, dt);

    trajectory.insert(trajectory.end(), last_traj_full_saturated.begin(), last_traj_full_saturated.end());
    return trajectory;
}

void TrajectoryUtils::apply_transformation_to_trajectory(std::vector<Eigen::Vector3d>& trajectory, const Eigen::Matrix4d& transform) {
    for (int i = 0; i < trajectory.size(); i++) {
        trajectory.at(i) = transform.topLeftCorner<3, 3>() * trajectory.at(i) + transform.rightCols<1>().head<3>();
        // TODO: Do the same for orientation
    }
}

void TrajectoryUtils::interp_between_two_points_pos(std::vector<Eigen::Vector3d>& dest, const Eigen::Vector3d& start, const Eigen::Vector3d& end, const int& horizon) {
    Eigen::Vector3d delta_pos = (end - start) / horizon;
    for (int i = 0; i <= horizon; i++) {
        dest.push_back(start + i * delta_pos);
    }
}

std::vector<Eigen::Vector3d> TrajectoryUtils::splineInterpolationPosition(const std::vector<Eigen::Vector3d>& points, const std::vector<double>& timesteps, const int& nb_points) {
    std::vector<Eigen::Vector3d> points_up;
    int num_point_down = points.size();

    Eigen::VectorXd x = Eigen::VectorXd::Zero(num_point_down);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(num_point_down);
    Eigen::VectorXd z = Eigen::VectorXd::Zero(num_point_down);
    Eigen::VectorXd t = Eigen::VectorXd::Zero(num_point_down);

    int idx = 0;
    for (auto const& p : points) {
        x(idx) = p(0);
        y(idx) = p(1);
        z(idx) = p(2);
        t(idx) = timesteps.at(idx);

        idx++;
    }

    auto fit_x = Eigen::SplineFitting<Eigen::Spline<double, 1, 2>>::Interpolate(x.transpose(), 2, t);
    auto fit_y = Eigen::SplineFitting<Eigen::Spline<double, 1, 2>>::Interpolate(y.transpose(), 2, t);
    auto fit_z = Eigen::SplineFitting<Eigen::Spline<double, 1, 2>>::Interpolate(z.transpose(), 2, t);

    Eigen::Spline<double, 1, 2> spline_x(fit_x);
    Eigen::Spline<double, 1, 2> spline_y(fit_y);
    Eigen::Spline<double, 1, 2> spline_z(fit_z);

    for (int i = 0; i <= nb_points; i++) {
        double t_i = static_cast<double>(i) / nb_points;
        Eigen::Vector3d point_t(spline_x(t_i).coeff(0), spline_y(t_i).coeff(0), spline_z(t_i).coeff(0));
        points_up.push_back(point_t);
    }

    return points_up;
}

Eigen::Vector4d TrajectoryUtils::quaternionSlerp(const Eigen::Vector4d& origin, const Eigen::Vector4d& dest, const double& t) {
    Eigen::Quaterniond origin_q(origin(0), origin(1), origin(2), origin(3));
    Eigen::Quaterniond dest_q(dest(0), dest(1), dest(2), dest(3));
    Eigen::Quaterniond quat_t_q = origin_q.slerp(t, dest_q);
    Eigen::Vector4d quat_t;
    quat_t << quat_t_q.w(), quat_t_q.x(), quat_t_q.y(), quat_t_q.z();
    return quat_t;
}

std::vector<Eigen::Vector4d> TrajectoryUtils::generateQuaternionTrajectory(const std::vector<Eigen::Vector4d>& orientations, const std::vector<double>& timesteps, const int& nb_points) {
    std::vector<Eigen::Vector4d> quat_traj;
    for (int i = 0; i <= nb_points; i++) {
        double t = ((double)i) / nb_points;
        double min_dist = 1.0;
        int anchor = 1;

        for (int j = 1; j < timesteps.size(); j++) {
            double dist = timesteps.at(j) - t;
            if (dist >= 0 && dist < min_dist) {
                min_dist = dist;
                anchor = j;
            }
        }

        Eigen::Vector4d quat_t =
            TrajectoryUtils::quaternionSlerp(orientations.at(anchor - 1), orientations.at(anchor), (t - timesteps.at(anchor - 1)) / (timesteps.at(anchor) - timesteps.at(anchor - 1)));
        quat_traj.push_back(quat_t);
    }

    return quat_traj;
}

std::vector<std::vector<Eigen::Vector2d>> TrajectoryUtils::buildSignature(const std::string& input_path) {
    std::vector<std::vector<Eigen::Vector2d>> signature;
    time_t current_time_std = std::time(0);
    auto current_time = std::localtime(&current_time_std);

    int month = 1 + current_time->tm_mon;
    int day = current_time->tm_mday;

    auto number_decomposition = [](const int& number) {
        int decade = number / 10;
        int unit = number - (decade * 10);
        return std::make_tuple(decade, unit);
    };

    std::array<std::tuple<int, int>, 2> numbers = {number_decomposition(day), number_decomposition(month)};

    Eigen::Vector2d offset = Eigen::Vector2d::Zero();
    double x_margin = 12;

    std::pair<std::string, std::vector<std::vector<Eigen::Vector2d>>> sep_csv;
    sketching::serial::loadText(input_path + "writing/sep.txt", sep_csv);
    std::array<Eigen::Vector2d, 2> min_max_sep = TrajectoryUtils::getMinMax(sep_csv.second);

    for (auto number : numbers) {
        std::pair<std::string, std::vector<std::vector<Eigen::Vector2d>>> decade_csv;
        sketching::serial::loadText(input_path + "writing/" + std::to_string(std::get<0>(number)) + ".txt", decade_csv);
        std::array<Eigen::Vector2d, 2> min_max_decade = TrajectoryUtils::getMinMax(decade_csv.second);
        TrajectoryUtils::applyOffset2d(decade_csv.second, offset);
        signature.insert(signature.end(), decade_csv.second.begin(), decade_csv.second.end());

        offset(0) += min_max_decade[1](0) + x_margin;

        std::pair<std::string, std::vector<std::vector<Eigen::Vector2d>>> unit_csv;
        sketching::serial::loadText(input_path + "writing/" + std::to_string(std::get<1>(number)) + ".txt", unit_csv);
        std::array<Eigen::Vector2d, 2> min_max_unit = TrajectoryUtils::getMinMax(unit_csv.second);
        TrajectoryUtils::applyOffset2d(unit_csv.second, offset);
        signature.insert(signature.end(), unit_csv.second.begin(), unit_csv.second.end());

        offset(0) += min_max_unit[1](0) + x_margin;

        std::vector<std::vector<Eigen::Vector2d>> sep = sep_csv.second;
        TrajectoryUtils::applyOffset2d(sep, offset);
        signature.insert(signature.end(), sep.begin(), sep.end());

        offset(0) += min_max_sep[1](0) + x_margin;
    }

    std::pair<std::string, std::vector<std::vector<Eigen::Vector2d>>> signature_csv;
    sketching::serial::loadText(input_path + "writing/signature.txt", signature_csv);
    TrajectoryUtils::applyOffset2d(signature_csv.second, offset);
    signature.insert(signature.end(), signature_csv.second.begin(), signature_csv.second.end());

    return signature;
}

void TrajectoryUtils::applyOffset2d(std::vector<std::vector<Eigen::Vector2d>>& paths, const Eigen::Vector2d& offset) {
    for (auto& path : paths) {
        /*for (auto& point_2d : path) {
            point_2d += offset;
        }*/
        std::transform(path.cbegin(), path.cend(), path.begin(), [&](const Eigen::Vector2d& p) { return p + offset; });
    }
}

std::array<Eigen::Vector2d, 2> TrajectoryUtils::getMinMax(const std::vector<std::vector<Eigen::Vector2d>>& strokes) {
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = 0;
    double max_y = 0;

    // Add a third dimension to the points & Look for maximal & minimal x/y
    for (std::vector<Eigen::Vector2d> path_2d : strokes) {
        for (Eigen::Vector2d point_2d : path_2d) {
            if (point_2d.x() < min_x) {
                min_x = point_2d.x();
            }
            if (point_2d.y() < min_y) {
                min_y = point_2d.y();
            }

            if (point_2d.x() > max_x) {
                max_x = point_2d.x();
            }

            if (point_2d.y() > max_y) {
                max_y = point_2d.y();
            }
        }
    }

    Eigen::Vector2d min_p(min_x, min_y);
    Eigen::Vector2d max_p(max_x, max_y);
    return {min_p, max_p};
}

}  // namespace sketching
