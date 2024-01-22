// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ergodic_sketching/AgentInitialization.hpp>
#include <random>
namespace sketching {
sackmesser::Factory<AgentInitialization> AgentInitialization::factory_ = sackmesser::Factory<AgentInitialization>();

AgentInitialization::AgentInitialization() {}

AgentInitialization::~AgentInitialization() {}

sackmesser::Factory<AgentInitialization>& AgentInitialization::Factory() {
    return factory_;
}

RejectionSampling::~RejectionSampling() {}

void RejectionSampling::initialize(const Eigen::MatrixXd& distribution, const std::vector<ErgodicControl::Agent::Ptr>& agents, const double& step_size) {
    for (const ErgodicControl::Agent::Ptr& agent : agents) {
        bool initialized = false;

        while (!initialized) {
            int x = std::rand() % static_cast<int>(distribution.cols());
            int y = std::rand() % static_cast<int>(distribution.rows());

            if (distribution(y, x) > 0) {
                ErgodicControl::Agent::Position position(x, y);
                agent->initialize(position * step_size);
                initialized = true;
            }
        }
    }
}

GibbsSampling::~GibbsSampling() {}

void GibbsSampling::initialize(const Eigen::MatrixXd& input_distribution, const std::vector<ErgodicControl::Agent::Ptr>& agents, const double& step_size) {
    Eigen::MatrixXd distribution = input_distribution / input_distribution.sum();

    int x = static_cast<int>(distribution.cols() / 2);
    int y = static_cast<int>(distribution.rows() / 2);

    for (unsigned i = 0; i < 50; ++i) {
        sample(distribution, x, y);
    }

    for (const ErgodicControl::Agent::Ptr& agent : agents) {
        sample(distribution, x, y);

        ErgodicControl::Agent::Position position(x, y);
        agent->initialize(position * step_size);
    }
}

void GibbsSampling::sample(const Eigen::MatrixXd& distribution, int& x, int& y) const {
    static std::default_random_engine engine;

    Eigen::VectorXd y_x = distribution.col(x) / distribution.col(x).sum();
    std::vector<double> y_x_val(y_x.data(), y_x.data() + y_x.rows());

    std::discrete_distribution<int> gen_y(y_x_val.begin(), y_x_val.end());
    y = gen_y(engine);

    Eigen::VectorXd x_y = distribution.row(y) / distribution.row(y).sum();
    std::vector<double> x_y_val(x_y.data(), x_y.data() + x_y.rows());

    std::discrete_distribution<int> gen_x(x_y_val.begin(), x_y_val.end());
    x = gen_x(engine);
}

}  // namespace sketching
