// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <ilqr_planner/sim/KDLRobot.h>

#include <Eigen/Dense>

#include <functional>
#include <string>
#include <tuple>

class KDLRobot;

class Planner {
public:
    struct Configuration {
        std::string urdf;
        std::string base_frame;
        std::string tip_frame;

        int dof;
        int nb_ctrl_var;
        int nb_state_var;
        int nb_iter;
        int batch_size;
        int order;

        double rfactor;
        double dt;

        Eigen::VectorXd q_max;
        Eigen::VectorXd q_min;
        Eigen::VectorXd dq_max;
        Eigen::VectorXd dq_min;
        Eigen::VectorXd q0;
        Eigen::VectorXd dq0;

        Eigen::Vector3d orn_thresh;
        int init_phase_length;
    };

    explicit Planner(const Configuration& config);

    ~Planner();
    void solve(const std::vector<Eigen::Matrix<double, 7, 1>>& states,
               const std::vector<Eigen::Matrix<double, 6, 6>>& precisions,
               const std::function<void(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const double&)>& cb);

protected:
    Configuration config_;
    std::shared_ptr<ilqr_planner::sim::KDLRobot> sim_;
};
