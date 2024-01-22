// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <ergodic_sketching/ErgodicAgent.hpp>
#include <sackmesser/ConfigurationServer.hpp>

namespace sketching {
class FirstOrderAgent::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const sackmesser::ConfigurationServer::Ptr& config_server) {
        return config_server->loadParameter(ns_ + "max_velocity", &max_velocity, false) &&  //
               config_server->loadParameter(ns_ + "normalize_gradient", &normalize_gradient, false);
    }

    double max_velocity;

    bool normalize_gradient;
};

class SecondOrderAgent::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const sackmesser::ConfigurationServer::Ptr& config_server) {
        return config_server->loadParameter(ns_ + "max_velocity", &max_velocity, false) &&  //
               config_server->loadParameter(ns_ + "max_acceleration", &max_acceleration, false);
    }

    double max_velocity;

    double max_acceleration;
};

class DubinsAgent::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const sackmesser::ConfigurationServer::Ptr& config_server) {
        return config_server->loadParameter(ns_ + "max_velocity", &max_velocity, false) &&  //
               config_server->loadParameter(ns_ + "turning_radius", &turning_radius, false);
    }

    double max_velocity;

    double turning_radius;
};
}  // namespace sketching
