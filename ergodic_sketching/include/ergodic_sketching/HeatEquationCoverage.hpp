// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <ergodic_sketching/ErgodicControl.hpp>

namespace sackmesser {
class Interface;
}

namespace sketching {
class HeatEquationCoverage : public ErgodicControl {
public:
    class Configuration;

public:
    explicit HeatEquationCoverage(const std::shared_ptr<sackmesser::Interface>& interface, const std::string& name = "ergodic_control/");

    virtual ~HeatEquationCoverage();

    using ErgodicControl::compute;

private:
    Eigen::MatrixXd compute(const Eigen::MatrixXd& distribution, const std::vector<Agent::Ptr>& agents, const unsigned& timesteps);

    Eigen::Vector2d calculateGradient(const cv::Mat& gradient_x, const cv::Mat& gradient_y, const Eigen::Vector2d& position, const int& width, const int& height, const double& dx);

private:
    std::unique_ptr<Configuration> config_;
};

}  // namespace sketching
