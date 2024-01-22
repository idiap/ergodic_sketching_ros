// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ergodic_sketching/SpectralMultiscaleCoverage.hpp>
#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser/Interface.hpp>

namespace sketching {
class SpectralMultiscaleCoverage::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& server);

    double dt;

    int num_basis_functions;

    double size;
};

bool SpectralMultiscaleCoverage::Configuration::load(const std::shared_ptr<sackmesser::ConfigurationServer>& server) {
    return server->loadParameter(ns_ + "dt", &dt, false, 0.001, 10.0) &&                               //
           server->loadParameter(ns_ + "num_basis_functions", &num_basis_functions, false, 2, 200) &&  //
           server->loadParameter(ns_ + "size", &size, false, 1.0, 100.0);
}

SpectralMultiscaleCoverage::SpectralMultiscaleCoverage(const std::shared_ptr<sackmesser::Interface>& interface)
    : ErgodicControl(interface, "ergodic_control/smc/"), config_(std::make_unique<Configuration>("ergodic_control/smc/")) {
    if (!config_->load(interface->configServer())) {
        throw "SpectralMultiscaleCoverage: failed to configure";
    }

    num_coefficients_ = static_cast<int>(std::pow(config_->num_basis_functions, 2.0));

    omega_ = 2.0 * M_PI / config_->size;

    basis_index_ = Eigen::VectorXd::LinSpaced(config_->num_basis_functions, 0, config_->num_basis_functions - 1);
    k_index_x_ = basis_index_.replicate(config_->num_basis_functions, 1);
    k_index_y_ = Eigen::Map<Eigen::MatrixXd>(Eigen::MatrixXd((basis_index_.replicate(1, config_->num_basis_functions)).transpose()).data(), num_coefficients_, 1);

    weighting_vector_ = (k_index_x_.array().pow(2.0) + k_index_y_.array().pow(2.0) + 1.0).array().pow(-(1.8) / 2.0);

    int im_dim = 256;

    Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(im_dim, 0.0, 1.0);

    Eigen::MatrixXd x1 = x.replicate(im_dim, 1);
    Eigen::MatrixXd x2 = Eigen::Map<Eigen::MatrixXd>(Eigen::MatrixXd((x.replicate(1, im_dim)).transpose()).data(), im_dim * im_dim, 1);

    double norm = std::pow(config_->size, 2.0);

    phi_inv_ = ((k_index_x_ * x1.transpose() * omega_).array().cos() *  //
                (k_index_y_ * x2.transpose() * omega_).array().cos())   //
               / (norm * std::pow(im_dim, 2.0));
}

SpectralMultiscaleCoverage::~SpectralMultiscaleCoverage() {}

Eigen::MatrixXd SpectralMultiscaleCoverage::compute(const Eigen::MatrixXd& distribution, const std::vector<Agent::Ptr>& agents, const unsigned& timesteps) {
    int dimension = static_cast<int>(distribution.cols());

    Eigen::MatrixXd p = Eigen::Map<const Eigen::MatrixXd>(distribution.data(), 1, dimension * dimension);
    p *= std::pow(dimension, 2.0) / distribution.sum();

    Eigen::MatrixXd w_hat = phi_inv_ * p.transpose();

    Eigen::MatrixXd wt = Eigen::MatrixXd::Zero(num_coefficients_, 1);

    double norm = std::pow(config_->size, 2.0);

    for (unsigned t = 0; t < timesteps && isComputing(); ++t) {
        std::vector<Eigen::MatrixXd> dphi;

        for (unsigned j = 0; j < agents.size(); ++j) {
            const Agent::Position x = agents[j]->getPosition();

            Eigen::MatrixXd cx = ((x * basis_index_.transpose() * omega_).array().cos());
            Eigen::MatrixXd dcx = (-(x * basis_index_.transpose() * omega_).array().sin() * basis_index_.transpose().replicate(2, 1).array() * omega_);

            Eigen::MatrixXd tmp = Eigen::Map<Eigen::MatrixXd>(Eigen::MatrixXd(cx.row(1).replicate(config_->num_basis_functions, 1)).data(), 1, num_coefficients_);
            Eigen::MatrixXd tmp2 = Eigen::Map<Eigen::MatrixXd>(Eigen::MatrixXd(dcx.row(1).replicate(config_->num_basis_functions, 1)).data(), 1, num_coefficients_);

            Eigen::MatrixXd dphi_tmp = Eigen::MatrixXd::Zero(2, num_coefficients_);
            dphi_tmp.row(0) = (dcx.row(0).replicate(1, config_->num_basis_functions).array() * tmp.array()).matrix();
            dphi_tmp.row(1) = (cx.row(0).replicate(1, config_->num_basis_functions).array() * tmp2.array()).matrix();
            dphi.push_back(dphi_tmp);

            wt += Eigen::MatrixXd(cx.row(0).replicate(1, config_->num_basis_functions).array() * tmp.array()).transpose() / (double(agents.size()) * norm);
        }

        Eigen::MatrixXd dw = Eigen::MatrixXd(weighting_vector_.array() * (wt / static_cast<double>(t + 1) - w_hat).array());

        for (unsigned j = 0; j < agents.size(); ++j) {
            agents[j]->updateState(-(dphi[j] * dw) * config_->dt);
        }
    }

    return Eigen::MatrixXd::Zero(dimension, dimension);
}

// void SpectralMultiscaleCoverage::compute(const std::vector<Gaussian> &gaussians, const std::vector<Agent::Ptr> &agents)
// {
//     Eigen::MatrixXd w = Eigen::MatrixXd(2, num_coefficients_);
//     w.row(0) = Eigen::Map<Eigen::MatrixXd>(Eigen::MatrixXd(k_index_x_ * omega_).data(), 1, num_coefficients_);
//     w.row(1) = Eigen::Map<Eigen::MatrixXd>(Eigen::MatrixXd(k_index_y_ * omega_).data(), 1, num_coefficients_);

//     Eigen::MatrixXd w_hat = fourierTransform(gaussians, w);

//     compute(w_hat, std::pow(config_->size, gaussians.size()), agents);
// }

// Eigen::MatrixXd SpectralMultiscaleCoverage::fourierTransform(const std::vector<Gaussian> &gaussians, const Eigen::MatrixXd &w)
// {
//     Eigen::Matrix<double, 2, 4> op;
//     op << -1, 1, -1, 1, -1, -1, 1, 1;

//     Eigen::MatrixXd priors = Eigen::MatrixXd::Ones(1, static_cast<int>(gaussians.size())) / static_cast<double>(gaussians.size());

//     Eigen::MatrixXd w_hat = Eigen::MatrixXd::Zero(num_coefficients_, 1);
//     for (unsigned k = 0; k < gaussians.size(); ++k)
//     {
//         for (int n = 0; n < op.cols(); ++n)
//         {
//             Eigen::DiagonalMatrix<double, 2> diag(op.col(n));

//             Eigen::Matrix<double, 2, 1> mean = diag * gaussians[k].mean();
//             Eigen::Matrix<double, 2, 2> covariance = diag * gaussians[k].covariance() * diag;

//             Eigen::MatrixXd a = Eigen::MatrixXd((w.transpose() * mean).array().cos());
//             Eigen::MatrixXd b = Eigen::MatrixXd((-0.5 * w.transpose() * covariance * w).diagonal().array().exp());

//             w_hat = w_hat + priors(k) * Eigen::MatrixXd(a.array() * b.array());
//         }
//     }
//     w_hat = w_hat / std::pow(config_->size, gaussians.size()) / (static_cast<double>(gaussians.size()));

//     return w_hat;
// }

}  // namespace sketching
