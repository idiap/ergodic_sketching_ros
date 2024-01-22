// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ergodic_sketching/AgentConfiguration.hpp>
#include <ergodic_sketching/AgentInitialization.hpp>
#include <ergodic_sketching/Configuration.hpp>
#include <ergodic_sketching/ErgodicAgent.hpp>
#include <ergodic_sketching/ErgodicControl.hpp>
#include <ergodic_sketching/HeatEquationCoverage.hpp>
#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser/Interface.hpp>

namespace sketching {
sackmesser::Factory<ErgodicControl::Agent> ErgodicControl::Agent::factory_ = sackmesser::Factory<ErgodicControl::Agent>();

ErgodicControl::ErgodicControl(const std::shared_ptr<sackmesser::Interface>& interface, const std::string& name)
    : interface_(interface), config_(std::make_unique<Configuration>(name)), is_computing_(false) {
    Agent::Factory().add<const std::string&, const sackmesser::ConfigurationServer::Ptr&>(
        "first_order", [](const std::string& name, const sackmesser::ConfigurationServer::Ptr& config_server) -> Agent::Ptr { return std::make_unique<FirstOrderAgent>(name, config_server); });

    Agent::Factory().add<const std::string&, const sackmesser::ConfigurationServer::Ptr&>(
        "dubins", [](const std::string& name, const sackmesser::ConfigurationServer::Ptr& config_server) -> Agent::Ptr { return std::make_unique<DubinsAgent>(name, config_server); });

    Agent::Factory().add<const std::string&, const sackmesser::ConfigurationServer::Ptr&>(
        "second_order", [](const std::string& name, const sackmesser::ConfigurationServer::Ptr& config_server) -> Agent::Ptr { return std::make_unique<SecondOrderAgent>(name, config_server); });

    if (!config_->load(interface->configServer())) {
        throw "ErgodicControl: failed to configure";
    }

    for (int i = 0; i < config_->num_agents; ++i) {
        agents_.push_back(Agent::Factory().create<const std::string&, const sackmesser::ConfigurationServer::Ptr&>(config_->agent_type, name + "agent/", interface_->configServer()));
    }

    AgentInitialization::Factory().add<>("rejection_sampling", []() -> AgentInitialization::Ptr { return std::make_unique<RejectionSampling>(); });
    AgentInitialization::Factory().add<>("gibbs_sampling", []() -> AgentInitialization::Ptr { return std::make_unique<GibbsSampling>(); });

    initialization_ = AgentInitialization::Factory().create(config_->initialization_method);
}

ErgodicControl::~ErgodicControl() {}

void ErgodicControl::stop() {
    is_computing_ = false;
}

bool ErgodicControl::isComputing() {
    return is_computing_;
}

const double& ErgodicControl::stepSize() const {
    return config_->step_size;
}

std::vector<ErgodicControl::Agent::Path> ErgodicControl::compute(Eigen::MatrixXd& distribution) {
    initialization_->initialize(distribution, agents_, config_->step_size);

    is_computing_ = true;

    Eigen::MatrixXd coverage = compute(distribution, agents_, config_->timesteps);

    distribution -= distribution.maxCoeff() * coverage / coverage.maxCoeff();

    std::vector<ErgodicControl::Agent::Path> paths;

    for (const Agent::Ptr& agent : agents_) {
        paths.push_back(agent->getPath());
    }

    is_computing_ = false;

    return paths;
}

}  // namespace sketching
