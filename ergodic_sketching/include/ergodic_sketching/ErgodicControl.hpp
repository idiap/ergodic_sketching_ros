// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <any>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sackmesser/ClassFactory.hpp>
#include <vector>

namespace sackmesser {
class Interface;
class ConfigurationServer;
}  // namespace sackmesser

namespace sketching {
class AgentInitialization;

class ErgodicControl {
public:
    class Agent {
    public:
        using Position = Eigen::Vector2d;

        using Path = std::vector<Position>;

    public:
        Agent();

        virtual ~Agent();

        virtual void initialize(const Position& position) = 0;

        virtual void updateState(const Eigen::Vector2d& gradient) = 0;

        virtual Position getPosition() = 0;

        const Path& getPath() const;

    public:
        using Ptr = std::shared_ptr<Agent>;

        static sackmesser::Factory<Agent>& Factory();

        Path path_;

    private:
        static sackmesser::Factory<Agent> factory_;
    };

    class Configuration;

public:
    ErgodicControl(const std::shared_ptr<sackmesser::Interface>& interface, const std::string& name);

    virtual ~ErgodicControl();

    std::vector<Agent::Path> compute(Eigen::MatrixXd& distribution);

    void stop();

    bool isComputing();

    const double& stepSize() const;

protected:
    virtual Eigen::MatrixXd compute(const Eigen::MatrixXd& distribution, const std::vector<Agent::Ptr>& agents, const unsigned& timesteps) = 0;

    std::shared_ptr<sackmesser::Interface> interface_;

private:
    std::unique_ptr<Configuration> config_;

    std::vector<Agent::Ptr> agents_;

    bool is_computing_;

    std::unique_ptr<AgentInitialization> initialization_;
};

}  // namespace sketching
