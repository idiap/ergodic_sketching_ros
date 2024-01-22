// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser/Interface.hpp>

#include <ergodic_sketching/Camera.hpp>

namespace sketching {
namespace acquisition {
struct Camera::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) {
        return config_server->loadParameter(ns_ + "fps", &fps, false) &&                          //
               config_server->loadParameter(ns_ + "image_width", &image_width, false) &&          //
               config_server->loadParameter(ns_ + "image_height", &image_height, false) &&        //
               config_server->loadParameter(ns_ + "camera_index", &camera_index, false) &&        //
               config_server->loadParameter(ns_ + "cv_rotate", &cv_rotate, false) &&              //
               config_server->loadParameter(ns_ + "filter_horizon", &filter_horizon, false) &&    //
               config_server->loadParameter(ns_ + "rectangle_width", &rectangle_width, false) &&  //
               config_server->loadParameter(ns_ + "rectangle_height", &rectangle_height, false);
    }

    int fps;

    int cv_rotate;

    int filter_horizon;

    int camera_index;

    int image_width;

    int image_height;

    int rectangle_width;

    int rectangle_height;
};
}  // namespace acquisition
}  // namespace sketching
