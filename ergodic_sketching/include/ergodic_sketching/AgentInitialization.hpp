// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <Eigen/Core>
#include <ergodic_sketching/ErgodicControl.hpp>
#include <sackmesser/ClassFactory.hpp>

namespace sketching {
class AgentInitialization {
public:
    AgentInitialization();

    virtual ~AgentInitialization();

    virtual void initialize(const Eigen::MatrixXd& distribution, const std::vector<ErgodicControl::Agent::Ptr>& agents, const double& step_size) = 0;

public:
    using Ptr = std::unique_ptr<AgentInitialization>;

    static sackmesser::Factory<AgentInitialization>& Factory();

protected:
private:
    static sackmesser::Factory<AgentInitialization> factory_;
};

class RejectionSampling : public AgentInitialization {
public:
    using AgentInitialization::AgentInitialization;

    virtual ~RejectionSampling();

    void initialize(const Eigen::MatrixXd& distribution, const std::vector<ErgodicControl::Agent::Ptr>& agents, const double& step_size) override;

private:
};

class GibbsSampling : public AgentInitialization {
public:
    using AgentInitialization::AgentInitialization;

    virtual ~GibbsSampling();

    void initialize(const Eigen::MatrixXd& distribution, const std::vector<ErgodicControl::Agent::Ptr>& agents, const double& step_size) override;

private:
    void sample(const Eigen::MatrixXd& distribution, int& x, int& y) const;
};

}  // namespace sketching
