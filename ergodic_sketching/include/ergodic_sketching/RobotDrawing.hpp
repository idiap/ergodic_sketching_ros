// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <Eigen/Core>
#include <sackmesser/ConfigurationServer.hpp>
#include <vector>

namespace sackmesser {
class Interface;
}

namespace sketching {
class RobotDrawing {
public:
    struct DrawingZone {
        Eigen::Matrix4d transform;
        Eigen::Vector4d orientation;
        std::array<double, 7> q;
    };

    struct Configuration : public sackmesser::ConfigurationBase {
        using sackmesser::ConfigurationBase::ConfigurationBase;
        bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& server);

        double offset;

        std::vector<DrawingZone> drawing_zones;

        int init_phase_length;
        double max_cart_vel;
        double sheet_x_size;
        double sheet_y_size;
        double sheet_margin_x;
        double sheet_margin_y;
        double signature_width_ratio;
        double dt;
    };

    RobotDrawing(const std::shared_ptr<sackmesser::Interface>& interface, const std::string& name, const std::array<double, 3> transform_xyz, const std::array<double, 3> transform_rpy);

    std::vector<Eigen::Matrix<double, 7, 1>> process(const std::vector<std::vector<Eigen::Vector2d>>& paths, const DrawingZone& drawing_zone);
    std::vector<Eigen::Matrix<double, 7, 1>> process_signature(const std::vector<std::vector<Eigen::Vector2d>>& signature, const DrawingZone& drawing_zone);

    std::vector<DrawingZone> getDrawingZonesTransforms();

protected:
    Configuration config_;

    std::shared_ptr<sackmesser::Interface> interface_;
};
}  // namespace sketching
