// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <ergodic_sketching/ErgodicControl.hpp>

namespace sackmesser {
class ConfigurationServer;
}

namespace sketching {
class SpectralMultiscaleCoverage : public ErgodicControl {
public:
    class Configuration;

public:
    explicit SpectralMultiscaleCoverage(const std::shared_ptr<sackmesser::Interface>& interface);

    virtual ~SpectralMultiscaleCoverage();

    using ErgodicControl::compute;

private:
    Eigen::MatrixXd compute(const Eigen::MatrixXd& distribution, const std::vector<Agent::Ptr>& agents, const unsigned& timesteps);

private:
    std::unique_ptr<Configuration> config_;

    int num_coefficients_;

    double omega_;

    Eigen::MatrixXd phi_inv_;

    Eigen::MatrixXd weighting_vector_;
    Eigen::MatrixXd basis_index_;
    Eigen::MatrixXd k_index_x_;
    Eigen::MatrixXd k_index_y_;
};

}  // namespace sketching
