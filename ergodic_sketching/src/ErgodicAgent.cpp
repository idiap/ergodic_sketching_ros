// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ergodic_sketching/AgentConfiguration.hpp>
#include <ergodic_sketching/ErgodicAgent.hpp>
#include <ergodic_sketching/ErgodicControl.hpp>

namespace sketching {
ErgodicControl::Agent::Agent() {}

ErgodicControl::Agent::~Agent() {}

sackmesser::Factory<ErgodicControl::Agent>& ErgodicControl::Agent::Factory() {
    return factory_;
}

const ErgodicControl::Agent::Path& ErgodicControl::Agent::getPath() const {
    return path_;
}

FirstOrderAgent::FirstOrderAgent(const std::string& name, const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) : config_(std::make_unique<Configuration>(name)) {
    config_->load(config_server);
}

void FirstOrderAgent::initialize(const Position& position) {
    position_ = position;
    path_.clear();
    path_.push_back(position);
}

void FirstOrderAgent::updateState(const Eigen::Vector2d& gradient) {
    if (config_->normalize_gradient || gradient.norm() > config_->max_velocity) {
        position_ += config_->max_velocity * gradient.normalized();
    } else {
        position_ += gradient;
    }

    path_.push_back(getPosition());
}

ErgodicControl::Agent::Position FirstOrderAgent::getPosition() {
    return position_;
}

SecondOrderAgent::SecondOrderAgent(const std::string& name, const std::shared_ptr<sackmesser::ConfigurationServer>& config_server)
    : config_(std::make_unique<Configuration>(name)), velocity_(Eigen::Vector2d::Zero()) {
    config_->load(config_server);
}

void SecondOrderAgent::initialize(const Position& position) {
    position_ = position;
    velocity_ = Eigen::Vector2d::Zero();
    path_.clear();
    path_.push_back(position);
}

void SecondOrderAgent::updateState(const Eigen::Vector2d& gradient) {
    static double dt = 1.0;

    Eigen::Vector2d acceleration;

    if (gradient.norm() > config_->max_acceleration) {
        acceleration = config_->max_acceleration * gradient.normalized();
    } else {
        acceleration = gradient;
    }

    position_ += dt * velocity_ + 0.5 * dt * dt * acceleration;

    velocity_ += dt * acceleration;

    if (velocity_.norm() > config_->max_velocity) {
        velocity_ = config_->max_velocity * velocity_.normalized();
    }

    path_.push_back(getPosition());
}

ErgodicControl::Agent::Position SecondOrderAgent::getPosition() {
    return position_;
}

DubinsAgent::DubinsAgent(const std::string& name, const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) : config_(std::make_unique<Configuration>(name)) {
    config_->load(config_server);
}

void DubinsAgent::initialize(const Position& position) {
    position_ = position;
    yaw_ = 0.0;
    path_.clear();
    path_.push_back(position);
}

void DubinsAgent::updateState(const Eigen::Vector2d& gradient) {
    double omega_max = config_->max_velocity / config_->turning_radius;

    Eigen::Vector2d normalized_gradient = gradient.normalized();

    double omega = std::atan2(normalized_gradient.y(), normalized_gradient.x()) - yaw_;

    double theta = yaw_ + omega / std::abs(omega) * std::min(std::abs(omega), omega_max);

    position_.block(0, 0, 2, 1) += config_->max_velocity * Eigen::Vector2d(std::cos(theta), std::sin(theta));

    yaw_ = theta;

    path_.push_back(getPosition());
}

ErgodicControl::Agent::Position DubinsAgent::getPosition() {
    return position_;
}

}  // namespace sketching
