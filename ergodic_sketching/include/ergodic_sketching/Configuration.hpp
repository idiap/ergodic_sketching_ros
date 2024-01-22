// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <ergodic_sketching/ErgodicControl.hpp>
#include <ergodic_sketching/HeatEquationCoverage.hpp>
#include <ergodic_sketching/SketchPipeline.hpp>
#include <sackmesser/ConfigurationServer.hpp>

namespace sketching {
class ErgodicControl::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& server) {
        return server->loadParameter(ns_ + "agent_type", &agent_type) &&                        //
               server->loadParameter(ns_ + "timesteps", &timesteps, true) &&                    //
               server->loadParameter(ns_ + "initialization_method", &initialization_method) &&  //
               server->loadParameter(ns_ + "step_size", &step_size, true) &&                    //
               server->loadParameter(ns_ + "num_agents", &num_agents, true);
    }

    std::string agent_type;

    int num_agents;

    unsigned timesteps = 100;

    std::string initialization_method;

    double step_size;
};

class HeatEquationCoverage::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& server) {
        return server->loadParameter(ns_ + "diffusion", &diffusion, true) &&            //
               server->loadParameter(ns_ + "local_cooling", &local_cooling, true) &&    //
               server->loadParameter(ns_ + "cooling_radius", &cooling_radius, true) &&  //
               server->loadParameter(ns_ + "agent_radius", &agent_radius, true) &&      //
               server->loadParameter(ns_ + "source_strength", &source_strength, true);
    }

    double source_strength = 1.0;
    double diffusion = 1.0;
    double local_cooling = 1.0;
    double agent_radius = 1.0;
    double cooling_radius = 1.0;
};

class ErgodicSketching::Pipeline::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& server) { return server->loadParameter(ns_ + "num_strokes", &num_strokes, true); }

    int num_strokes = 1;
};

}  // namespace sketching
