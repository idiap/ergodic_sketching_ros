// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <ergodic_sketching/ErgodicControl.hpp>

namespace sketching {
class FirstOrderAgent : public ErgodicControl::Agent {
public:
    class Configuration;

    FirstOrderAgent(const std::string& name, const std::shared_ptr<sackmesser::ConfigurationServer>& config_server);

    void initialize(const Position& position);

    void updateState(const Eigen::Vector2d& gradient);

    Position getPosition();

private:
    std::unique_ptr<Configuration> config_;

    Position position_;

public:
    using Ptr = std::shared_ptr<FirstOrderAgent>;
};

class SecondOrderAgent : public ErgodicControl::Agent {
public:
    class Configuration;

    SecondOrderAgent(const std::string& name, const std::shared_ptr<sackmesser::ConfigurationServer>& config_server);

    void initialize(const Position& position);

    void updateState(const Eigen::Vector2d& gradient);

    Position getPosition();

private:
    std::unique_ptr<Configuration> config_;

    Position position_;

    Eigen::Vector2d velocity_;

public:
    using Ptr = std::shared_ptr<SecondOrderAgent>;
};

class DubinsAgent : public ErgodicControl::Agent {
public:
    class Configuration;

    DubinsAgent(const std::string& name, const std::shared_ptr<sackmesser::ConfigurationServer>& config_server);

    void initialize(const Position& position);

    void updateState(const Eigen::Vector2d& gradient);

    Position getPosition();

private:
    std::unique_ptr<Configuration> config_;

    Position position_;

    double yaw_;

public:
    using Ptr = std::shared_ptr<DubinsAgent>;
};

}  // namespace sketching
