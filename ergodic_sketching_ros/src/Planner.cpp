// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ilqr_planner/solver/ILQRRecursive.h>
#include <ilqr_planner/system/PosOrnKeypointDistFunct.h>
#include <ilqr_planner/system/PosOrnPlannerSys.h>

#include <ergodic_sketching_ros/Planner.hpp>

#include <fstream>

Planner::Planner(const Configuration& config) {
    config_ = config;
    sim_.reset(new ilqr_planner::sim::KDLRobot(config_.urdf, config_.base_frame, config_.tip_frame, config_.q0, config_.dq0, Eigen::VectorXd::Zero(3), Eigen::Vector3d::Zero(3),false));
}

Planner::~Planner() {}

void Planner::solve(const std::vector<Eigen::Matrix<double, 7, 1>>& drawing_states,
                    const std::vector<Eigen::Matrix<double, 6, 6>>& drawing_precisions,
                    const std::function<void(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const double&)>& cb) {
    sim_->setConfiguration(config_.q0, config_.dq0);
    std::vector<Eigen::Matrix<double, 7, 1>> states;

    Eigen::Vector3d initial_pos_start = sim_->getEEPosition();
    Eigen::Vector4d initial_orn = sim_->getEEOrnQuat();
    Eigen::Vector3d initial_pos_end = drawing_states.at(0).head(3);

    Eigen::Vector3d delta_pos = (initial_pos_end - initial_pos_start) / config_.init_phase_length;

    Eigen::Matrix<double, 7, 1> state_0;
    state_0 << initial_pos_start, initial_orn;
    states.push_back(state_0);

    for (int t = 1; t < config_.init_phase_length; t++) {
        Eigen::Matrix<double, 7, 1> state_t;
        state_t << initial_pos_start + t * delta_pos, initial_orn;
        states.push_back(state_t);
    }

    Eigen::Matrix<double, 6, 6> precision_t = Eigen::Matrix<double, 6, 6>::Identity();
    precision_t(3, 3) = 0.01;
    precision_t(4, 4) = 0.01;
    precision_t(5, 5) = 0.01;

    std::vector<Eigen::Matrix<double, 6, 6>> precisions;
    precisions.resize(states.size(), precision_t);

    // Concatenate init & drawing phases
    states.insert(states.end(), drawing_states.begin(), drawing_states.end());
    precisions.insert(precisions.end(), drawing_precisions.begin(), drawing_precisions.end());

    std::vector<std::shared_ptr<ilqr_planner::sys::Keypoint>> keypoints;

    for (unsigned int i = 0; i < states.size(); i++) {
        if (config_.order == 1) {
            std::shared_ptr<ilqr_planner::sys::Keypoint> kp =
                std::make_shared<ilqr_planner::sys::PosOrnKeypointDistFunct>(states[i].head(3), states[i].tail(4), precisions[i], 0, config_.orn_thresh, i % config_.batch_size);
            keypoints.push_back(kp);
        } else {
            Eigen::MatrixXd precision = Eigen::MatrixXd::Zero(2 * precisions[i].rows(), 2 * precisions[i].cols());
            precision.topLeftCorner(precisions[i].rows(), precisions[i].cols()) = precisions[i];

            std::shared_ptr<ilqr_planner::sys::Keypoint> kp = std::make_shared<ilqr_planner::sys::PosOrnKeypointDistFunct>(
                states[i].head(3), Eigen::VectorXd::Zero(3), states[i].tail(4), Eigen::VectorXd::Zero(4), precision, 0, config_.orn_thresh, i % config_.batch_size);
            keypoints.push_back(kp);
        }
    }

    // Start planning
    Eigen::VectorXd cmd_penalties = Eigen::VectorXd::Ones(config_.nb_ctrl_var) * config_.rfactor;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    std::vector<Eigen::Matrix4d> transforms;
    transforms.resize(config_.batch_size, transform);

    Eigen::VectorXd q0_tmp = sim_->getJointsPos();
    Eigen::VectorXd dq0_tmp = sim_->getJointsVel();
    Eigen::VectorXd u0_t = Eigen::VectorXd::Zero(config_.dof);

    std::vector<Eigen::VectorXd> joint_positions;
    std::vector<Eigen::VectorXd> joint_velocities;
    std::vector<Eigen::VectorXd> ts_positions;

    for (int start_idx = 0; start_idx < states.size(); start_idx += config_.batch_size) {
        sim_->setConfiguration(q0_tmp, dq0_tmp);

        int batch_size = config_.batch_size;
        if (start_idx + batch_size > states.size()) {
            batch_size = states.size() - start_idx - 1;
        }

        std::vector<std::shared_ptr<ilqr_planner::sys::Keypoint>> keypoints_tmp(keypoints.begin() + start_idx, keypoints.begin() + start_idx + batch_size);

        Eigen::VectorXd q_max = config_.q_max;
        Eigen::VectorXd q_min = config_.q_min;
        Eigen::VectorXd dq_max = config_.dq_max;
        Eigen::VectorXd dq_min = config_.dq_min;

        std::shared_ptr<ilqr_planner::sys::PosOrnPlannerSys> sys =
            std::make_shared<ilqr_planner::sys::PosOrnPlannerSys>(sim_, keypoints_tmp, cmd_penalties, q_max, q_min, dq_max, dq_min, keypoints_tmp.size(), config_.order, config_.dt);

        std::vector<Eigen::VectorXd> u0;
        u0.resize(keypoints_tmp.size(), u0_t);
        ilqr_planner::solver::ILQRRecursive planner(sys);
        auto result = planner.solve(u0, config_.nb_iter, true, true);

        std::vector<Eigen::VectorXd> joint_positions_tmp;
        std::vector<Eigen::VectorXd> joint_velocities_tmp;

        if (config_.order == 1) {
            joint_positions_tmp = std::get<0>(result);
            joint_velocities_tmp = std::get<2>(result);
            joint_velocities_tmp.insert(joint_velocities_tmp.begin(), Eigen::VectorXd::Zero(config_.dof));
        } else {
            for (auto const& q_dq : std::get<0>(result)) {
                Eigen::VectorXd q = q_dq.head(config_.dof);
                Eigen::VectorXd dq = q_dq.tail(config_.dof);

                joint_positions_tmp.push_back(q);
                joint_velocities_tmp.push_back(dq);
            }
        }

        joint_positions.insert(joint_positions.end(), joint_positions_tmp.begin(), joint_positions_tmp.end());
        joint_velocities.insert(joint_velocities.end(), joint_velocities_tmp.begin(), joint_velocities_tmp.end());
        ts_positions.insert(ts_positions.end(), std::get<1>(result).begin(), std::get<1>(result).end());

        q0_tmp = joint_positions.at(joint_positions.size() - 1);

        cb(joint_positions_tmp, joint_velocities_tmp, std::get<5>(result));
    }
}
