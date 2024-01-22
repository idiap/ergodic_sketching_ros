// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sackmesser/Visualiser.hpp>
#include <sackmesser_runtime/ConfigurationServerYAML.hpp>

namespace sketching {
class Image : public sackmesser::Visualiser::Visualisation<Eigen::MatrixXd> {
    struct Configuration : public sackmesser::ConfigurationBase {
        using sackmesser::ConfigurationBase::ConfigurationBase;

        bool load(const std::shared_ptr<sackmesser::ConfigurationServer>&) { return true; }
    };

public:
    Image(const std::string& name, const std::shared_ptr<sackmesser::ConfigurationServer>& server) : Visualisation(name, server, std::make_unique<Configuration>("")) {}

    void show(const Eigen::MatrixXd& value) const {
        Eigen::MatrixXd matrix = (value / value.maxCoeff());

        cv::Mat image;

        cv::eigen2cv(matrix, image);

        cv::rotate(image, image, cv::ROTATE_180);
        cv::resize(image, image, cv::Size(0.75 * image.rows, 0.75 * image.cols));

        cv::imshow("heat", image);

        cv::waitKey(1);
    }
};

}  // namespace sketching
